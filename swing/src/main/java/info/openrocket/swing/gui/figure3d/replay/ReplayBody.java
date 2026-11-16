package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.rocketcomponent.Parachute;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.Streamer;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * One separately flying part of a replayed flight: the main rocket, or a stage after it separates,
 * following its own flight data branch.
 *
 * <p>Until its first recovery deployment the body flies as one rigid piece with the recorded
 * attitude. Each deployment opens the airframe at the front of the tube the device is packed in,
 * and from then on {@link RecoveryDynamics} moves the pieces, shock cords and canopies.</p>
 *
 * <p>Coordinates: rocket (model) coordinates run aft along +X from the nose tip, in engine units.
 * The body frame is the replay world shifted so the body's pivot point is at the origin.</p>
 */
final class ReplayBody {
	private static final float S = RenderingConstants.WORLD_SCALE;
	private static final double LANDING_SETTLE_HEIGHT = 3.0;
	private static final double EPSILON = 1e-4;
	/** Shock cord length for a design without one, as a multiple of the body length, and its limits. */
	private static final double CORD_PER_LENGTH = 2.0;
	private static final double SHORTEST_CORD = 0.6;
	private static final double LONGEST_CORD = 8.0;
	/** A filled round canopy turns slowly about its axis, radians per second. */
	private static final double SPIN_RATE = 0.35;
	/** Straight pieces each shock cord is drawn with, so a slack cord can sag. */
	private static final int CORD_PIECES = 4;
	private static final double GRAVITY = 9.81;

	/**
	 * A recovery device and where it opens the airframe.
	 *
	 * @param cordLength length of the shock cord across that joint, metres, or NaN when not known
	 */
	record DeviceJoint(double time, RecoveryDevice device, double station, double cordLength) {
	}

	private record Deployment(double time, RecoveryDevice device, int joint, double cord) {
	}

	private final FlightReplayData data;
	private final double front;
	private final double aft;
	private final double radius;
	private final double launchCgHeight;
	private final Vector3f launchAxis;
	/** Interior joint stations in rocket coordinates (metres), ascending. Joint j sits before section j. */
	private final double[] stations;
	private final List<Deployment> deployments = new ArrayList<>();
	/** Pivot position along the body at launch, rocket coordinates (metres). */
	private final double launchPivot;
	/** At each stage separation: time, pivot step along the body (metres), and that step in the world. */
	private final List<double[]> pivotShifts = new ArrayList<>();
	private final RecoveryDynamics dynamics;
	private final RecoveryDynamics.Sample sample = new RecoveryDynamics.Sample();

	// Results of the last update(), in the body frame.
	private final Matrix4f[] sectionMatrices;
	private final Vector3f sitePivot = new Vector3f();
	/** Where this body's pad sits from the launch site's origin, engine units. */
	private final Vector3f siteOffset = new Vector3f();
	private final List<Vector3f[]> cords = new ArrayList<>();
	private Vector3f[] canopyPositions = new Vector3f[0];
	private Quaternionf[] canopyRotations = new Quaternionf[0];
	private float[] canopyCollapse = new float[0];
	private double extent;

	/**
	 * @param data           flight data branch the body follows
	 * @param front          forward end of the body in rocket coordinates, metres
	 * @param aft            aft end of the body in rocket coordinates, metres
	 * @param radius         largest body radius, metres
	 * @param devices        recovery devices this body deploys
	 * @param launchCgHeight height of the launch centre of gravity above the ground, metres
	 * @param launchAxis     nose direction on the pad, engine axes
	 */
	ReplayBody(FlightReplayData data, double front, double aft, double radius, List<DeviceJoint> devices,
			double launchCgHeight, Vector3f launchAxis) {
		this.data = data;
		this.front = front;
		this.aft = aft;
		this.radius = radius;
		this.launchCgHeight = launchCgHeight;
		this.launchAxis = new Vector3f(launchAxis);

		this.stations = devices.stream()
				.mapToDouble(DeviceJoint::station)
				.filter(x -> x > front + EPSILON && x < aft - EPSILON)
				.sorted()
				.distinct()
				.toArray();
		double defaultCord = Math.max(SHORTEST_CORD, Math.min(LONGEST_CORD, CORD_PER_LENGTH * (aft - front)));
		devices.stream()
				.sorted((a, b) -> Double.compare(a.time(), b.time()))
				.forEach(d -> deployments.add(new Deployment(d.time(), d.device(), jointOf(d.station()),
						d.cordLength() > 0 ? d.cordLength() : defaultCord)));

		this.sectionMatrices = new Matrix4f[stations.length + 1];
		for (int i = 0; i < sectionMatrices.length; i++) {
			sectionMatrices[i] = new Matrix4f();
		}

		// The body pivots about its centre of gravity, which jumps when a stage separates. Move the
		// pivot by that step, and the tracked position by the same step in the world, so the parts
		// stay where they were at the moment of separation.
		double cg = data.getValue(FlightDataType.TYPE_CG_LOCATION, data.getStartTime());
		this.launchPivot = Double.isFinite(cg) ? cg : 0.5 * (front + aft);
		for (FlightReplayData.Event event : data.getAllEvents()) {
			if (event.type() == FlightEvent.Type.STAGE_SEPARATION) {
				double step = data.getStepAcross(FlightDataType.TYPE_CG_LOCATION, event.time());
				if (step != 0) {
					Vector3f shift = rigidRotation(event.time()).transform(new Vector3f((float) (step * S), 0, 0));
					pivotShifts.add(new double[] { event.time(), step, shift.x, shift.y, shift.z });
				}
			}
		}
		this.dynamics = deployments.isEmpty() ? null : buildDynamics();
	}

	// ---------- queries ----------

	FlightReplayData getData() {
		return data;
	}

	/** Time the body left the main rocket, or NaN for the main rocket itself. */
	double getSeparationTime() {
		return data.getSeparationTime();
	}

	double getFront() {
		return front;
	}

	double getAft() {
		return aft;
	}

	double getRadius() {
		return radius;
	}

	int getSectionCount() {
		return sectionMatrices.length;
	}

	/** Most straight pieces {@link #getCords()} can hold. */
	int getCordCapacity() {
		return 2 * CORD_PIECES * (sectionMatrices.length + 1);
	}

	/** Section holding the rocket-coordinate position {@code x}, in metres. */
	int sectionOf(double x) {
		int section = 0;
		while (section < stations.length && x > stations[section] + EPSILON) {
			section++;
		}
		return section;
	}

	/** Deployment time of a device, or NaN when this body does not deploy it. */
	double deploymentTime(RecoveryDevice device) {
		for (Deployment deployment : deployments) {
			if (deployment.device() == device) {
				return deployment.time();
			}
		}
		return Double.NaN;
	}

	List<RecoveryDevice> getDeployedDevices() {
		List<RecoveryDevice> devices = new ArrayList<>();
		for (Deployment deployment : deployments) {
			devices.add(deployment.device());
		}
		return devices;
	}

	double getFirstDeploymentTime() {
		return deployments.isEmpty() ? Double.NaN : deployments.get(0).time();
	}

	/** Moves the body's whole flight by a fixed offset, engine units, as when it flies from another pad. */
	void setSiteOffset(Vector3f offset) {
		siteOffset.set(offset);
	}

	/** Pivot position in the launch-site frame from the last update. */
	Vector3f getSitePivot() {
		return sitePivot;
	}

	/** Transform of a section, body frame, from the last update. */
	Matrix4f getSectionMatrix(int section) {
		return sectionMatrices[section];
	}

	/** Straight pieces of shock cord as (from, to) pairs in the body frame, from the last update. */
	List<Vector3f[]> getCords() {
		return cords;
	}

	/** Farthest any part reaches from the pivot, metres, from the last update. */
	double getExtent() {
		return extent;
	}

	/** Canopy origin, where its shroud lines meet, for the i-th deployed device, body frame, or null while it is packed. */
	Vector3f getCanopyPosition(int deployment) {
		return deployment < canopyPositions.length ? canopyPositions[deployment] : null;
	}

	Quaternionf getCanopyRotation(int deployment) {
		return canopyRotations[deployment];
	}

	/** How far the i-th canopy has fallen flat on the ground, 0 to 1, from the last update. */
	float getCanopyCollapse(int deployment) {
		return deployment < canopyCollapse.length ? canopyCollapse[deployment] : 0;
	}

	// ---------- rigid flight ----------

	/** Rotation from rocket coordinates to the replay world at time {@code t}. */
	Quaternionf rigidRotation(double t) {
		Vector3f axis = data.getPose().getAxisDirection(t);
		if (axis == null) {
			axis = new Vector3f(launchAxis);
		}
		// The nose (-X) turns to +Y, then to the flight attitude.
		return new Quaternionf()
				.rotateTo(new Vector3f(0, 1, 0), axis)
				.rotateZ((float) (-Math.PI / 2))
				.rotateX((float) data.getRollAngle(t));
	}

	/** Transform of the whole body flying rigidly at time {@code t}, pivoting about its CG. */
	Matrix4f rigidMatrix(double t) {
		return new Matrix4f().rotate(rigidRotation(t)).translate((float) (-pivot(t) * S), 0, 0);
	}

	/** Point along the body that tracks the simulated position, rocket coordinates (metres). */
	private double pivot(double t) {
		double pivot = launchPivot;
		for (double[] shift : pivotShifts) {
			if (t >= shift[0]) {
				pivot += shift[1];
			}
		}
		return pivot;
	}

	/** Where the rigid body's pivot is at time {@code t}, launch-site frame without the pad offset. */
	private Vector3f rigidPivot(double t) {
		Vector3f pivot = data.getPose().getPosition(t).add(0, (float) (rigidHeightOffset(t) * S), 0);
		// The simulation lands the body at zero altitude, so let the height part of a stage
		// separation's shift go as it touches down.
		float settle = (float) landingSettle(t);
		for (double[] shift : pivotShifts) {
			if (t >= shift[0]) {
				pivot.add((float) shift[2], (float) shift[3] * settle, (float) shift[4]);
			}
		}
		return pivot;
	}

	/**
	 * The point the simulation tracks once the recovery devices are out, launch-site frame without
	 * the pad offset. It keeps the launch height of the CG all the way down, so the parts reach the
	 * ground on their own at their own speed.
	 */
	private Vector3f trackedPoint(double t) {
		Vector3f point = data.getPose().getPosition(t).add(0, (float) (launchCgHeight * S), 0);
		for (double[] shift : pivotShifts) {
			if (t >= shift[0]) {
				point.add((float) shift[2], (float) shift[3], (float) shift[4]);
			}
		}
		return point;
	}

	// ---------- per frame ----------

	/** Works out where every section, canopy and cord is at time {@code t}. */
	void update(double t) {
		cords.clear();
		if (dynamics == null || !(t >= dynamics.getStart())) {
			Matrix4f rigid = rigidMatrix(t);
			for (Matrix4f matrix : sectionMatrices) {
				matrix.set(rigid);
			}
			sitePivot.set(rigidPivot(t)).add(siteOffset);
			canopyPositions = new Vector3f[0];
			canopyRotations = new Quaternionf[0];
			canopyCollapse = new float[0];
			double pivot = pivot(t);
			extent = Math.max(pivot - front, aft - pivot);
			return;
		}

		RecoveryDynamics.Sample now = dynamics.sample(t, sample);
		RecoveryDynamics.Layout layout = now.layout;
		double[] x = now.position;
		double[] centre = now.centre;
		sitePivot.set((float) (centre[0] * S), (float) (centre[1] * S), (float) (centre[2] * S)).add(siteOffset);
		extent = 0;
		for (int p = 0; p < layout.particles(); p++) {
			double dx = x[3 * p] - centre[0], dy = x[3 * p + 1] - centre[1], dz = x[3 * p + 2] - centre[2];
			extent = Math.max(extent, Math.sqrt(dx * dx + dy * dy + dz * dz));
		}

		for (int r = 0; r < layout.rods(); r++) {
			int a = layout.rodFront[r], b = layout.rodAft[r];
			Vector3f along = new Vector3f((float) (x[3 * b] - x[3 * a]), (float) (x[3 * b + 1] - x[3 * a + 1]),
					(float) (x[3 * b + 2] - x[3 * a + 2])).normalize();
			Vector3f up = new Vector3f((float) now.up[3 * r], (float) now.up[3 * r + 1], (float) now.up[3 * r + 2]);
			up.sub(new Vector3f(along).mul(up.dot(along))).normalize();
			Vector3f side = new Vector3f(along).cross(up);
			Vector3f at = bodyPoint(x, a, centre);
			double pieceFront = layout.rodFirst[r] == 0 ? front : stations[layout.rodFirst[r] - 1];
			for (int i = layout.rodFirst[r]; i <= layout.rodLast[r]; i++) {
				sectionMatrices[i].set(along.x, along.y, along.z, 0, up.x, up.y, up.z, 0, side.x, side.y, side.z, 0,
						at.x, at.y, at.z, 1).translate((float) (-pieceFront * S), 0, 0);
			}
		}

		float groundHeight = (float) (-centre[1] * S);
		for (int l = layout.cordStart; l < layout.lineStart; l++) {
			addCord(bodyPoint(x, layout.linkA[l], centre), bodyPoint(x, layout.linkB[l], centre),
					layout.linkLength[l], groundHeight);
		}

		int open = layout.open;
		canopyPositions = new Vector3f[open];
		canopyRotations = new Quaternionf[open];
		canopyCollapse = new float[open];
		for (int d = 0; d < open; d++) {
			Vector3f knot = bodyPoint(x, layout.knot[d], centre);
			Vector3f direction = bodyPoint(x, layout.canopy[d], centre).sub(knot);
			if (direction.lengthSquared() < 1e-8f) {
				direction.set(0, 1, 0);
			}
			double down = dynamics.touchdown(d);
			double turning = (Double.isNaN(down) ? t : Math.min(t, down)) - deployments.get(d).time();
			float spin = (float) (SPIN_RATE * turning * (d % 2 == 0 ? 1 : -1));
			canopyPositions[d] = knot;
			canopyRotations[d] = new Quaternionf().rotateTo(new Vector3f(0, 1, 0), direction.normalize()).rotateY(spin);
			canopyCollapse[d] = (float) dynamics.collapse(d, t);
		}
	}

	/** A particle's position in the body frame, engine units. */
	private static Vector3f bodyPoint(double[] x, int particle, double[] centre) {
		return new Vector3f((float) ((x[3 * particle] - centre[0]) * S), (float) ((x[3 * particle + 1] - centre[1]) * S),
				(float) ((x[3 * particle + 2] - centre[2]) * S));
	}

	/** A shock cord between two points, sagging when it is slack, kept above the ground. */
	private void addCord(Vector3f from, Vector3f to, double length, float groundHeight) {
		double span = from.distance(to) / S;
		float sag = span < length ? (float) (0.5 * Math.sqrt(length * length - span * span) * S) : 0;
		float lowest = groundHeight + 0.003f * S;
		Vector3f previous = from;
		for (int k = 1; k <= CORD_PIECES; k++) {
			float u = (float) k / CORD_PIECES;
			Vector3f point = new Vector3f(from).lerp(to, u);
			point.y = Math.max(lowest, point.y - sag * 4 * u * (1 - u));
			cords.add(new Vector3f[] { previous, point });
			previous = point;
		}
	}

	// ---------- recovery ----------

	private RecoveryDynamics buildDynamics() {
		double[] faces = new double[stations.length + 2];
		faces[0] = front;
		System.arraycopy(stations, 0, faces, 1, stations.length);
		faces[faces.length - 1] = aft;
		List<RecoveryDynamics.Device> devices = new ArrayList<>();
		for (Deployment deployment : deployments) {
			RecoveryDevice device = deployment.device();
			double dragArea = device.getCD() * device.getArea();
			double mass = device.getComponentMass();
			devices.add(new RecoveryDynamics.Device(deployment.time(), deployment.joint(),
					Double.isFinite(dragArea) && dragArea > 0 ? dragArea : 0.05,
					Double.isFinite(mass) && mass > 0 ? mass : 0.02, reach(device), deployment.cord()));
		}
		RecoveryDynamics.Course course = new RecoveryDynamics.Course() {
			@Override
			public void centre(double t, double[] out) {
				set(out, trackedPoint(t).div(S));
			}

			@Override
			public void wind(double t, double[] out) {
				windAt(t, out);
			}

			@Override
			public double airDensity(double t) {
				return densityAt(t);
			}

			@Override
			public void rigidPoint(double station, double t, double[] out) {
				Vector3f along = rigidRotation(t).transform(new Vector3f((float) ((station - pivot(t)) * S), 0, 0));
				set(out, rigidPivot(t).add(along).div(S));
			}

			@Override
			public void rigidUp(double t, double[] out) {
				set(out, rigidRotation(t).transform(new Vector3f(0, 1, 0)));
			}
		};
		return new RecoveryDynamics(faces, radius, devices, bodyMass(devices), course, data.getEndTime(),
				data.getGroundHitTime());
	}

	private static void set(double[] out, Vector3f value) {
		out[0] = value.x;
		out[1] = value.y;
		out[2] = value.z;
	}

	/** Distance from where the shroud lines meet to the canopy's centre of drag, metres. */
	private static double reach(RecoveryDevice device) {
		if (device instanceof Parachute parachute) {
			ReplayMeshes.CanopyShape shape = ReplayMeshes.CanopyShape.of(parachute.getDiameter(),
					parachute.getLineLength(), parachute.getLineCount());
			return (shape.rimHeight() + 0.5 * shape.domeHeight()) / S;
		}
		if (device instanceof Streamer streamer) {
			return Math.max(0.1, 0.5 * streamer.getStripLength());
		}
		return 0.3;
	}

	/**
	 * Mass of the body when the first device opens: from the flight data, or for a measured flight
	 * without it, the mass its canopies hold up at the rate it came down.
	 */
	private double bodyMass(List<RecoveryDynamics.Device> devices) {
		double first = deployments.get(0).time();
		double recorded = data.getValue(FlightDataType.TYPE_MASS, first);
		if (Double.isFinite(recorded) && recorded > 0) {
			return recorded;
		}
		double last = deployments.get(deployments.size() - 1).time();
		double end = Double.isFinite(data.getGroundHitTime()) ? data.getGroundHitTime() : data.getEndTime();
		double from = last + 0.5 * (end - last);
		double to = end - 0.1 * (end - last);
		double drop = data.getValue(FlightDataType.TYPE_ALTITUDE, from) - data.getValue(FlightDataType.TYPE_ALTITUDE, to);
		double rate = drop / (to - from);
		if (to - from > 1 && Double.isFinite(rate) && rate > 0.5) {
			double dragArea = 0;
			for (RecoveryDynamics.Device device : devices) {
				dragArea += device.dragArea();
			}
			double mass = 0.5 * densityAt(0.5 * (from + to)) * dragArea * rate * rate / GRAVITY;
			return Math.max(0.05, Math.min(500, mass));
		}
		return Math.max(0.05, 400 * Math.PI * radius * radius * (aft - front));
	}

	/** Wind from the flight data, or for data without it, the drift of the flight itself. */
	private void windAt(double t, double[] out) {
		double speed = data.getValue(FlightDataType.TYPE_WIND_VELOCITY, t);
		double from = data.getValue(FlightDataType.TYPE_WIND_DIRECTION, t);
		if (Double.isFinite(speed) && Double.isFinite(from)) {
			// Blowing from a bearing: towards the opposite one. Engine axes: +X east, -Z north.
			out[0] = -speed * Math.sin(from);
			out[1] = 0;
			out[2] = speed * Math.cos(from);
			return;
		}
		double a = Math.max(data.getStartTime(), t - 0.5);
		double b = Math.min(data.getEndTime(), t + 0.5);
		if (b - a < 0.1) {
			Arrays.fill(out, 0);
			return;
		}
		Vector3f drift = data.getPose().getPosition(b).sub(data.getPose().getPosition(a)).div((float) ((b - a) * S));
		out[0] = drift.x;
		out[1] = 0;
		out[2] = drift.z;
	}

	private double densityAt(double t) {
		double density = data.getValue(FlightDataType.TYPE_AIR_DENSITY, t);
		if (Double.isFinite(density) && density > 0) {
			return density;
		}
		double altitude = data.getValue(FlightDataType.TYPE_ALTITUDE, t);
		return 1.225 * Math.exp(-(Double.isFinite(altitude) ? Math.max(0, altitude) : 0) / 9000);
	}

	// ---------- helpers ----------

	/** Joint number for a station: 0 at the front, n + 1 at the aft end, otherwise before section j. */
	private int jointOf(double station) {
		if (station <= front + EPSILON) {
			return 0;
		}
		if (station >= aft - EPSILON) {
			return stations.length + 1;
		}
		int index = Arrays.binarySearch(stations, station);
		if (index < 0) {
			index = -index - 1;
		}
		return index + 1;
	}

	/** Height of the pivot above the simulated position before anything deploys, metres. */
	private double rigidHeightOffset(double t) {
		double apogee = data.getApogeeTime();
		if (Double.isNaN(apogee) || t < apogee) {
			return launchCgHeight;
		}
		return radius + (launchCgHeight - radius) * landingSettle(t);
	}

	/** 1 while the body is well above the ground, falling to 0 as it touches down. */
	private double landingSettle(double t) {
		double apogee = data.getApogeeTime();
		if (!Double.isNaN(apogee) && t < apogee) {
			return 1;
		}
		double altitude = data.getValue(FlightDataType.TYPE_ALTITUDE, t);
		return Double.isFinite(altitude) ? Math.max(0, Math.min(1, altitude / LANDING_SETTLE_HEIGHT)) : 1;
	}

	static double smoothstep(double x) {
		double c = Math.max(0, Math.min(1, x));
		return c * c * (3 - 2 * c);
	}
}
