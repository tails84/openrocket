package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * One separately flying part of a replayed flight: the main rocket, or a stage after it separates,
 * following its own flight data branch.
 *
 * <p>Until its first recovery deployment the body flies as one rigid piece with the recorded
 * attitude. Each deployment opens the airframe at the front of the tube the device is packed in.
 * The simulation tracks the body as a single point, so the pieces are laid out for display: they
 * hang on shock cords below the latest canopy, swaying gently, and lie down side by side once they
 * reach the ground.</p>
 *
 * <p>Coordinates: rocket (model) coordinates run aft along +X from the nose tip, in engine units.
 * The body frame is the replay world shifted so the body's pivot point is at the origin.</p>
 */
final class ReplayBody {
	private static final float S = RenderingConstants.WORLD_SCALE;
	private static final double SETTLE_TIME = 2.0;
	private static final double LANDING_SETTLE_HEIGHT = 3.0;
	private static final float CORD_SPLAY = (float) Math.toRadians(25);
	private static final float PIECE_SPLAY = (float) Math.toRadians(12);
	private static final float CANOPY_LEAN = (float) Math.toRadians(30);
	private static final double EPSILON = 1e-4;

	/** A recovery device and where it opens the airframe. */
	record DeviceJoint(double time, RecoveryDevice device, double station) {
	}

	/** One end face of a section. */
	private record Anchor(int section, boolean aftFace) {
	}

	private record Deployment(double time, RecoveryDevice device, int joint) {
	}

	/** Sections {@code first..last}, inclusive, that hang together. */
	private record Piece(int first, int last) {
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

	// Results of the last update(), in the body frame.
	private final Matrix4f[] sectionMatrices;
	private final Vector3f sitePivot = new Vector3f();
	/** Where this body's pad sits from the launch site's origin, engine units. */
	private final Vector3f siteOffset = new Vector3f();
	private final List<Vector3f[]> cords = new ArrayList<>();
	private Vector3f[] canopyPositions = new Vector3f[0];
	private Quaternionf[] canopyRotations = new Quaternionf[0];

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
		devices.stream()
				.sorted((a, b) -> Double.compare(a.time(), b.time()))
				.forEach(d -> deployments.add(new Deployment(d.time(), d.device(), jointOf(d.station()))));

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

	/** Shock cords as (from, to) pairs in the body frame, from the last update. */
	List<Vector3f[]> getCords() {
		return cords;
	}

	/** Canopy origin for the i-th deployed device, body frame, or null while it is packed. */
	Vector3f getCanopyPosition(int deployment) {
		return deployment < canopyPositions.length ? canopyPositions[deployment] : null;
	}

	Quaternionf getCanopyRotation(int deployment) {
		return canopyRotations[deployment];
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

	// ---------- per frame ----------

	/** Works out where every section, canopy and cord is at time {@code t}. */
	void update(double t) {
		int state = 0;
		while (state < deployments.size() && deployments.get(state).time() <= t) {
			state++;
		}

		double offset;
		cords.clear();
		if (state == 0) {
			Matrix4f rigid = rigidMatrix(t);
			for (Matrix4f matrix : sectionMatrices) {
				matrix.set(rigid);
			}
			offset = rigidHeightOffset(t);
			canopyPositions = new Vector3f[0];
			canopyRotations = new Quaternionf[0];
		} else {
			double since = t - deployments.get(state - 1).time();
			float blend = (float) smoothstep(since / SETTLE_TIME);
			Vector3f[] centres = new Vector3f[sectionMatrices.length];
			Quaternionf[] rotations = new Quaternionf[sectionMatrices.length];
			if (state == 1) {
				rigidPoses(deployments.get(0).time(), centres, rotations);
			} else {
				hangingPoses(state - 1, t, centres, rotations);
			}
			Vector3f[] targetCentres = new Vector3f[centres.length];
			Quaternionf[] targetRotations = new Quaternionf[centres.length];
			hangingPoses(state, t, targetCentres, targetRotations);
			blendPoses(centres, rotations, targetCentres, targetRotations, blend);

			float landing = (float) (1 - landingSettle(t));
			if (landing > 0) {
				landedPoses(state, targetCentres, targetRotations);
				blendPoses(centres, rotations, targetCentres, targetRotations, landing);
			}
			for (int i = 0; i < sectionMatrices.length; i++) {
				double middle = 0.5 * (sectionFront(i) + sectionAft(i));
				sectionMatrices[i].translation(centres[i]).rotate(rotations[i])
						.translate((float) (-middle * S), 0, 0);
			}

			offset = Math.max(radius, -lowestPoint() / S + radius);
			if (state == 1) {
				offset = lerp(rigidHeightOffset(t), offset, blend);
			}
			placeCanopiesAndCords(state, blend, t);
		}

		Vector3f position = data.getPose().getPosition(t);
		sitePivot.set(position).add(0, (float) (offset * S), 0).add(siteOffset);
		// The simulation lands the body at zero altitude, so let the height part of the shift go
		// as it touches down.
		float settle = (float) landingSettle(t);
		for (double[] shift : pivotShifts) {
			if (t >= shift[0]) {
				sitePivot.add((float) shift[2], (float) shift[3] * settle, (float) shift[4]);
			}
		}
	}

	private void rigidPoses(double t, Vector3f[] centres, Quaternionf[] rotations) {
		Quaternionf rotation = rigidRotation(t);
		double pivot = pivot(t);
		for (int i = 0; i < centres.length; i++) {
			double middle = 0.5 * (sectionFront(i) + sectionAft(i));
			rotations[i] = new Quaternionf(rotation);
			centres[i] = rotation.transform(new Vector3f((float) ((middle - pivot) * S), 0, 0));
		}
	}

	/**
	 * Lays the pieces out on shock cords below the latest canopy, which holds the body pivot.
	 * Pieces forward of that canopy's joint hang on one side, nose down; pieces aft of it hang on
	 * the other side, nose up.
	 */
	private void hangingPoses(int state, double t, Vector3f[] centres, Quaternionf[] rotations) {
		int anchor = deployments.get(state - 1).joint();
		List<Piece> pieces = pieces(state);
		Vector3f heading = heading();
		float roll = (float) data.getRollAngle(getFirstDeploymentTime());
		Quaternionf sway = sway(t);
		float length = (float) ((aft - front) * S);

		List<Piece> forward = new ArrayList<>();
		List<Piece> aftward = new ArrayList<>();
		for (Piece piece : pieces) {
			(piece.last() < anchor ? forward : aftward).add(piece);
		}
		Collections.reverse(forward);	// start next to the anchor
		boolean twoSides = !forward.isEmpty() && !aftward.isEmpty();
		hangChain(forward, -1, twoSides, heading, roll, length, sway, centres, rotations);
		hangChain(aftward, 1, twoSides, heading, roll, length, sway, centres, rotations);
	}

	private void hangChain(List<Piece> chain, int side, boolean splay, Vector3f heading, float roll,
			float bodyLength, Quaternionf sway, Vector3f[] centres, Quaternionf[] rotations) {
		float cordAngle = splay ? CORD_SPLAY : 0;
		float pieceAngle = splay ? PIECE_SPLAY : 0;
		Vector3f cordDirection = new Vector3f(heading).mul(side * (float) Math.sin(cordAngle))
				.add(0, (float) -Math.cos(cordAngle), 0);
		Vector3f down = new Vector3f(heading).mul(side * (float) Math.sin(pieceAngle))
				.add(0, (float) -Math.cos(pieceAngle), 0);
		float cord = Math.max(0.35f * bodyLength, 0.3f * S);
		float gap = Math.max(0.3f * bodyLength, 0.3f * S);

		Vector3f top = new Vector3f(cordDirection).mul(cord);
		for (Piece piece : chain) {
			double pieceFront = sectionFront(piece.first());
			double pieceAft = sectionAft(piece.last());
			float pieceLength = (float) ((pieceAft - pieceFront) * S);
			// Forward pieces hang from their aft end, nose down; aft pieces from their front, nose up.
			Vector3f nose = side < 0 ? new Vector3f(down) : new Vector3f(down).negate();
			Quaternionf rotation = new Quaternionf(sway).mul(noseRotation(nose, roll));
			Vector3f pieceCentre = new Vector3f(down).mul(pieceLength / 2).add(top);
			sway.transform(pieceCentre);
			double pieceMiddle = 0.5 * (pieceFront + pieceAft);
			for (int i = piece.first(); i <= piece.last(); i++) {
				double middle = 0.5 * (sectionFront(i) + sectionAft(i));
				rotations[i] = new Quaternionf(rotation);
				centres[i] = rotation.transform(new Vector3f((float) ((middle - pieceMiddle) * S), 0, 0))
						.add(pieceCentre);
			}
			top.add(new Vector3f(down).mul(pieceLength + gap));
		}
	}

	/** Pieces lying on the ground in airframe order, nose towards the drift heading. */
	private void landedPoses(int state, Vector3f[] centres, Quaternionf[] rotations) {
		List<Piece> pieces = pieces(state);
		Vector3f heading = heading();
		Quaternionf rotation = noseRotation(heading, (float) data.getRollAngle(getFirstDeploymentTime()));
		double bodyMiddle = 0.5 * (front + aft);
		float gap = (float) (0.2 * (aft - front) * S);
		for (int p = 0; p < pieces.size(); p++) {
			Piece piece = pieces.get(p);
			float along = (float) (-(0.5 * (sectionFront(piece.first()) + sectionAft(piece.last())) - bodyMiddle) * S)
					+ gap * ((pieces.size() - 1) / 2f - p);
			double pieceMiddle = 0.5 * (sectionFront(piece.first()) + sectionAft(piece.last()));
			Vector3f pieceCentre = new Vector3f(heading).mul(along);
			for (int i = piece.first(); i <= piece.last(); i++) {
				double middle = 0.5 * (sectionFront(i) + sectionAft(i));
				rotations[i] = new Quaternionf(rotation);
				centres[i] = rotation.transform(new Vector3f((float) ((middle - pieceMiddle) * S), 0, 0))
						.add(pieceCentre);
			}
		}
	}

	private void placeCanopiesAndCords(int state, float blend, double t) {
		int anchor = deployments.get(state - 1).joint();
		for (int joint : separatedJoints(state)) {
			if (joint == anchor) {
				if (joint > 0) {
					cords.add(new Vector3f[] { new Vector3f(), facePoint(new Anchor(joint - 1, true)) });
				}
				if (joint <= stations.length) {
					cords.add(new Vector3f[] { new Vector3f(), facePoint(new Anchor(joint, false)) });
				}
			} else if (joint > 0 && joint <= stations.length) {
				cords.add(new Vector3f[] { facePoint(new Anchor(joint - 1, true)), facePoint(new Anchor(joint, false)) });
			}
		}

		canopyPositions = new Vector3f[state];
		canopyRotations = new Quaternionf[state];
		Quaternionf sway = sway(t);
		Vector3f heading = heading();
		Vector3f leanAxis = new Vector3f(heading).cross(0, 1, 0).normalize();
		for (int d = 0; d < state; d++) {
			Vector3f now = canopyPosition(d, state);
			if (d < state - 1 && state > 1) {
				now = canopyPosition(d, state - 1).lerp(now, blend);
			}
			canopyPositions[d] = now;
			int joint = deployments.get(d).joint();
			// Lean away from the anchor, towards the side the canopy's pieces hang on.
			float lean = joint == anchor ? 0 : (joint < anchor ? CANOPY_LEAN : -CANOPY_LEAN);
			canopyRotations[d] = new Quaternionf(sway).mul(new Quaternionf().rotateAxis(lean, leanAxis));
		}
	}

	/** A canopy on the anchor joint holds the pivot; others ride on the cord at their own joint. */
	private Vector3f canopyPosition(int deployment, int state) {
		int joint = deployments.get(deployment).joint();
		if (joint == deployments.get(state - 1).joint()) {
			return new Vector3f();
		}
		if (joint <= 0) {
			return facePoint(new Anchor(0, false));
		}
		if (joint > stations.length) {
			return facePoint(new Anchor(stations.length, true));
		}
		return facePoint(new Anchor(joint - 1, true)).add(facePoint(new Anchor(joint, false))).mul(0.5f);
	}

	private Vector3f facePoint(Anchor anchor) {
		double x = anchor.aftFace() ? sectionAft(anchor.section()) : sectionFront(anchor.section());
		return sectionMatrices[anchor.section()].transformPosition(new Vector3f((float) (x * S), 0, 0));
	}

	private float lowestPoint() {
		float lowest = 0;
		for (int i = 0; i < sectionMatrices.length; i++) {
			lowest = Math.min(lowest, facePoint(new Anchor(i, false)).y);
			lowest = Math.min(lowest, facePoint(new Anchor(i, true)).y);
		}
		return lowest;
	}

	// ---------- helpers ----------

	private List<Piece> pieces(int state) {
		List<Integer> separated = separatedJoints(state);
		List<Piece> pieces = new ArrayList<>();
		int first = 0;
		for (int section = 1; section <= stations.length; section++) {
			if (separated.contains(section)) {
				pieces.add(new Piece(first, section - 1));
				first = section;
			}
		}
		pieces.add(new Piece(first, stations.length));
		return pieces;
	}

	private List<Integer> separatedJoints(int state) {
		List<Integer> joints = new ArrayList<>();
		for (int d = 0; d < state; d++) {
			if (!joints.contains(deployments.get(d).joint())) {
				joints.add(deployments.get(d).joint());
			}
		}
		return joints;
	}

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

	private double sectionFront(int section) {
		return section == 0 ? front : stations[section - 1];
	}

	private double sectionAft(int section) {
		return section == stations.length ? aft : stations[section];
	}

	/** Horizontal direction of the nose when the first recovery device opened. */
	Vector3f heading() {
		Vector3f axis = data.getPose().getAxisDirection(getFirstDeploymentTime());
		Vector3f heading = axis == null ? new Vector3f() : new Vector3f(axis.x, 0, axis.z);
		if (heading.lengthSquared() < 1e-6f) {
			heading.set(1, 0, 0);
		}
		return heading.normalize();
	}

	private static Quaternionf noseRotation(Vector3f nose, float roll) {
		return new Quaternionf().rotateTo(new Vector3f(0, 1, 0), nose)
				.rotateZ((float) (-Math.PI / 2))
				.rotateX(roll);
	}

	/** A slow pendulum swing under the canopy. */
	private static Quaternionf sway(double t) {
		return new Quaternionf()
				.rotateZ((float) Math.toRadians(3 * Math.sin(2 * Math.PI * t / 4.3 + 1)))
				.rotateX((float) Math.toRadians(4 * Math.sin(2 * Math.PI * t / 3.1)));
	}

	private static void blendPoses(Vector3f[] centres, Quaternionf[] rotations,
			Vector3f[] targetCentres, Quaternionf[] targetRotations, float blend) {
		for (int i = 0; i < centres.length; i++) {
			centres[i].lerp(targetCentres[i], blend);
			rotations[i].slerp(targetRotations[i], blend);
		}
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

	private static double lerp(double a, double b, double f) {
		return a + (b - a) * f;
	}
}
