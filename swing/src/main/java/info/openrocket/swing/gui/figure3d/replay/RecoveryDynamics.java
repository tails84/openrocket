package info.openrocket.swing.gui.figure3d.replay;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Moves the parts of a body after its recovery devices open: the airframe pieces, the shock cords
 * that join them and the canopies, worked out once for the whole descent and then looked up.
 *
 * <p>Each piece of airframe is a rod with a point mass at each end. A shock cord runs from the
 * piece on one side of an open joint to a knot and on to the piece on the other side, and each
 * canopy hangs from the knot of its joint on its shroud lines. Cords and lines go slack but
 * stretch only a little, so the pieces fly apart on the ejection charge, snatch tight, swing under
 * the canopy and land one after another. Pieces have weight and drag, canopies have drag that
 * grows as they fill and falls away as they collapse on the ground.</p>
 *
 * <p>The flight simulation tracks the body as one point, so while it is in the air the mass
 * centre of all the parts is steered onto that point, and only how the parts move about it comes
 * from here. Near the ground the parts fall on their own.</p>
 *
 * <p>Positions are in metres in the launch-site frame: +X east, +Y up, -Z north, ground at y = 0.</p>
 */
final class RecoveryDynamics {
	/** Time a canopy takes to fill, seconds. */
	static final double INFLATION_TIME = 0.8;
	/** Time a canopy takes to fall flat once it is down, seconds. */
	static final double COLLAPSE_TIME = 1.0;

	private static final double G = 9.81;
	private static final double FRAME = 1.0 / 60;
	/** Substeps per frame around events and on the ground, and in steady descent. */
	private static final int FINE_SUBSTEPS = 8;
	private static final int COARSE_SUBSTEPS = 2;
	private static final int ITERATIONS = 2;
	private static final int CONTACT_ITERATIONS = 6;
	/** Fine steps for this long after each device opens, seconds. */
	private static final double BUSY_TIME = 5;
	/** Relative speed the ejection charge gives the pieces either side of a joint, m/s. */
	private static final double EJECTION_SPEED = 6;
	/** Share of the shock cord on the forward side of the canopy's knot. */
	private static final double KNOT_FRACTION = 0.3;
	/** How hard the mass centre is steered onto the simulated point, rad/s (critically damped). */
	private static final double TRACK_RATE = 6;
	/** Time over which the mass centre moves from where the pieces start onto the simulated point. */
	private static final double CENTRE_BLEND_TIME = 3;
	/** Time over which the steering's last average fades out after the parts are let go. */
	private static final double TRIM_FADE = 1.5;
	private static final double TRIM_SMOOTHING = 0.5;
	/** Keep working this long after touchdown, seconds; the longest run, seconds. */
	private static final double AFTER_LANDING = 20;
	private static final double LONGEST = 900;
	private static final double CROSS_CD = 1.2;
	private static final double AXIAL_CD = 0.6;
	private static final double CORD_DIAMETER = 0.006;
	/** Strain of a shock cord or the shroud lines holding the whole weight. */
	private static final double STATIC_STRETCH = 0.004;
	private static final double CORD_DAMPING = 0.25;
	private static final double STATIC_FRICTION = 0.8;
	private static final double SLIDING_FRICTION = 0.6;
	private static final double KNOT_RADIUS = 0.01;
	/** Closest that the flat ends of two pieces come, metres. */
	private static final double END_GAP = 0.005;
	private static final double CANOPY_RADIUS = 0.03;
	private static final double MAX_SPEED = 400;

	private static final int KNOT_KEY = 10_000;
	private static final int CANOPY_KEY = 20_000;

	/**
	 * A recovery device.
	 *
	 * @param time     when it opens, seconds
	 * @param joint    joint it opens: 0 at the front, the number of sections at the aft end, otherwise before section j
	 * @param dragArea drag coefficient times reference area when filled, square metres
	 * @param mass     canopy mass, kilograms
	 * @param lines    distance from the knot to the canopy's centre of drag, metres
	 * @param cord     shock cord length across its joint, metres
	 */
	record Device(double time, int joint, double dragArea, double mass, double lines, double cord) {
	}

	/** What the flight data says about the body. */
	interface Course {
		/** Where the body's tracked point is at time {@code t}, launch-site metres. */
		void centre(double t, double[] out);

		/** Wind at time {@code t}, m/s. */
		void wind(double t, double[] out);

		/** Air density at time {@code t}, kg/m³. */
		double airDensity(double t);

		/** Where a station along the rigid body is at time {@code t}, launch-site metres. */
		void rigidPoint(double station, double t, double[] out);

		/** The rigid body's rocket-frame +Y axis at time {@code t}. */
		void rigidUp(double t, double[] out);
	}

	/** How the particles are joined while a given number of devices are open. */
	static final class Layout {
		final int open;
		final int[] keys;
		final double[] mass;
		final double[] inverseMass;
		final double[] contact;
		final int[] rodFront;
		final int[] rodAft;
		final int[] rodFirst;
		final int[] rodLast;
		final double[] rodLength;
		/** Links: rods first, then shock cords, then shroud lines. */
		final int[] linkA;
		final int[] linkB;
		final double[] linkLength;
		final double[] linkCompliance;
		final double[] linkDamping;
		final boolean[] linkSlack;
		final int cordStart;
		final int lineStart;
		/** Particle of each device's canopy and knot, or -1 while it is packed. */
		final int[] canopy;
		final int[] knot;
		final double[] dragArea;

		private Layout(int open, int[] keys, double[] mass, double[] contact, int[][] rods, int[] rodSections,
				double[] rodLength, List<double[]> links, int cordStart, int lineStart, int[] canopy, int[] knot,
				double[] dragArea) {
			this.open = open;
			this.keys = keys;
			this.mass = mass;
			this.inverseMass = new double[mass.length];
			for (int i = 0; i < mass.length; i++) {
				inverseMass[i] = 1 / mass[i];
			}
			this.contact = contact;
			this.rodFront = rods[0];
			this.rodAft = rods[1];
			this.rodFirst = Arrays.copyOfRange(rodSections, 0, rodLength.length);
			this.rodLast = Arrays.copyOfRange(rodSections, rodLength.length, 2 * rodLength.length);
			this.rodLength = rodLength;
			int n = links.size();
			linkA = new int[n];
			linkB = new int[n];
			linkLength = new double[n];
			linkCompliance = new double[n];
			linkDamping = new double[n];
			linkSlack = new boolean[n];
			for (int i = 0; i < n; i++) {
				double[] link = links.get(i);
				linkA[i] = (int) link[0];
				linkB[i] = (int) link[1];
				linkLength[i] = link[2];
				linkCompliance[i] = link[3];
				linkSlack[i] = link[4] > 0;
				double effectiveMass = 1 / (inverseMass[linkA[i]] + inverseMass[linkB[i]]);
				linkDamping[i] = link[3] > 0 ? 2 * CORD_DAMPING * Math.sqrt(effectiveMass / link[3]) : 0;
			}
			this.cordStart = cordStart;
			this.lineStart = lineStart;
			this.canopy = canopy;
			this.knot = knot;
			this.dragArea = dragArea;
		}

		int particles() {
			return keys.length;
		}

		int rods() {
			return rodLength.length;
		}

		int indexOf(int key) {
			for (int i = 0; i < keys.length; i++) {
				if (keys[i] == key) {
					return i;
				}
			}
			return -1;
		}
	}

	/** Where everything is at one moment. */
	static final class Sample {
		Layout layout;
		double[] position = new double[0];
		double[] up = new double[0];
		final double[] centre = new double[3];
	}

	private final double[] faces;
	private final double radius;
	private final List<Device> devices;
	private final double totalMass;
	private final Course course;
	private final double groundHit;
	private final double end;
	private final double start;
	private final Layout[] layouts;
	private final List<float[]> frames = new ArrayList<>();
	private final List<Layout> frameLayouts = new ArrayList<>();
	private final double[] touchdown;

	// Working state
	private Layout layout;
	private double[] x = new double[0];
	private double[] v = new double[0];
	private double[] previous = new double[0];
	private double[] axis = new double[0];
	private double[] up = new double[0];
	private double[] lambda = new double[0];
	/** Whether anything may be touching the ground or another piece, so the solver works harder. */
	private boolean contacts;
	/** How far the ground pushed each particle up this step, metres. */
	private double[] lift = new double[0];

	/**
	 * @param faces     section end stations in rocket coordinates, metres: front, each joint, aft
	 * @param radius    body radius, metres
	 * @param devices   recovery devices in the order they open
	 * @param mass      mass of the whole body when the first device opens, kilograms
	 * @param course    flight data of the body
	 * @param end       last time with flight data, seconds
	 * @param groundHit time the simulation reached the ground, or NaN
	 */
	RecoveryDynamics(double[] faces, double radius, List<Device> devices, double mass, Course course, double end,
			double groundHit) {
		this.faces = faces.clone();
		this.radius = radius;
		this.devices = List.copyOf(devices);
		this.totalMass = mass;
		this.course = course;
		this.groundHit = groundHit;
		this.end = end;
		this.start = devices.get(0).time();
		this.touchdown = new double[devices.size()];
		Arrays.fill(touchdown, Double.NaN);
		this.layouts = new Layout[devices.size() + 1];
		for (int open = 1; open <= devices.size(); open++) {
			layouts[open] = buildLayout(open);
		}
		// The pieces settle after the simulation's own flight ends on the ground.
		double stop = Double.isFinite(groundHit) ? groundHit + AFTER_LANDING : end;
		run(Math.max(start, Math.min(stop, start + LONGEST)));
	}

	/** Time the first device opened. */
	double getStart() {
		return start;
	}

	/** How far canopy {@code device} has fallen flat on the ground at time {@code t}, 0 to 1. */
	double collapse(int device, double t) {
		double down = touchdown[device];
		return Double.isNaN(down) ? 0 : ReplayBody.smoothstep((t - down) / COLLAPSE_TIME);
	}

	/** How full canopy {@code device} is at time {@code t}, 0 to 1. */
	double inflation(int device, double t) {
		return ReplayBody.smoothstep((t - devices.get(device).time()) / INFLATION_TIME);
	}

	/** Time canopy {@code device} came to rest on the ground, or NaN. */
	double touchdown(int device) {
		return touchdown[device];
	}

	/** Positions at time {@code t}, which is no earlier than the first device opening. */
	Sample sample(double t, Sample out) {
		int open = 0;
		while (open < devices.size() && devices.get(open).time() <= t) {
			open++;
		}
		open = Math.max(open, 1);
		double f = (t - start) / FRAME;
		int last = frames.size() - 1;
		int i = (int) Math.max(0, Math.min(last, Math.floor(f)));
		int j = Math.min(last, i + 1);
		double blend = Math.max(0, Math.min(1, f - i));
		if (frameLayouts.get(i).open != open && frameLayouts.get(j).open == open) {
			i = j;
		}
		if (frameLayouts.get(j).open != frameLayouts.get(i).open) {
			j = i;
		}
		Layout layout = frameLayouts.get(i);
		float[] a = frames.get(i), b = frames.get(j);
		int n = 3 * layout.particles();
		int r = 3 * layout.rods();
		if (out.position.length != n) {
			out.position = new double[n];
		}
		if (out.up.length != r) {
			out.up = new double[r];
		}
		out.layout = layout;
		for (int k = 0; k < n; k++) {
			out.position[k] = a[k] + (b[k] - a[k]) * blend;
		}
		for (int k = 0; k < r; k++) {
			out.up[k] = a[n + k] + (b[n + k] - a[n + k]) * blend;
		}
		double total = 0;
		Arrays.fill(out.centre, 0);
		for (int p = 0; p < layout.particles(); p++) {
			for (int c = 0; c < 3; c++) {
				out.centre[c] += layout.mass[p] * out.position[3 * p + c];
			}
			total += layout.mass[p];
		}
		for (int c = 0; c < 3; c++) {
			out.centre[c] /= total;
		}
		return out;
	}

	// ---------- layout ----------

	private int sections() {
		return faces.length - 1;
	}

	private static int faceKey(int section, boolean aft) {
		return 2 * section + (aft ? 1 : 0);
	}

	private List<Integer> openJoints(int open) {
		List<Integer> joints = new ArrayList<>();
		for (int d = 0; d < open; d++) {
			int joint = devices.get(d).joint();
			if (!joints.contains(joint)) {
				joints.add(joint);
			}
		}
		return joints;
	}

	private Layout buildLayout(int open) {
		List<Integer> joints = openJoints(open);
		int n = sections();

		// Pieces of airframe between the open joints
		List<int[]> pieces = new ArrayList<>();
		int first = 0;
		for (int section = 1; section < n; section++) {
			if (joints.contains(section)) {
				pieces.add(new int[] { first, section - 1 });
				first = section;
			}
		}
		pieces.add(new int[] { first, n - 1 });

		// Mass: canopies and knots as given, the airframe shared by length
		List<Integer> allJoints = openJoints(devices.size());
		double knotMass = Math.max(0.005, 0.01 * totalMass);
		double canopies = 0;
		for (Device device : devices) {
			canopies += device.mass();
		}
		double airframe = Math.max(0.6 * totalMass, totalMass - canopies - knotMass * allJoints.size());
		double bodyLength = faces[n] - faces[0];

		List<Integer> keys = new ArrayList<>();
		List<Double> masses = new ArrayList<>();
		List<Double> contacts = new ArrayList<>();
		int rods = pieces.size();
		int[][] rodEnds = new int[2][rods];
		int[] rodSections = new int[2 * rods];
		double[] rodLength = new double[rods];
		List<double[]> links = new ArrayList<>();
		for (int r = 0; r < rods; r++) {
			int[] piece = pieces.get(r);
			double length = faces[piece[1] + 1] - faces[piece[0]];
			double half = 0.5 * airframe * length / bodyLength;
			rodEnds[0][r] = keys.size();
			keys.add(faceKey(piece[0], false));
			masses.add(half);
			contacts.add(radius);
			rodEnds[1][r] = keys.size();
			keys.add(faceKey(piece[1], true));
			masses.add(half);
			contacts.add(radius);
			rodSections[r] = piece[0];
			rodSections[rods + r] = piece[1];
			rodLength[r] = length;
			links.add(new double[] { rodEnds[0][r], rodEnds[1][r], length, 0, 0 });
		}

		int cordStart = links.size();
		int[] knotOfJoint = new int[n + 1];
		Arrays.fill(knotOfJoint, -1);
		for (int joint : joints) {
			double cord = 0;
			for (int d = 0; d < open; d++) {
				if (devices.get(d).joint() == joint) {
					cord = Math.max(cord, devices.get(d).cord());
				}
			}
			int knot = keys.size();
			knotOfJoint[joint] = knot;
			keys.add(KNOT_KEY + joint);
			masses.add(knotMass);
			contacts.add(KNOT_RADIUS);
			boolean forward = joint > 0;
			boolean aft = joint < n;
			if (forward) {
				int face = keys.indexOf(faceKey(joint - 1, true));
				double length = (aft ? KNOT_FRACTION : 1) * cord;
				links.add(new double[] { face, knot, length, compliance(length), 1 });
			}
			if (aft) {
				int face = keys.indexOf(faceKey(joint, false));
				double length = (forward ? 1 - KNOT_FRACTION : 1) * cord;
				links.add(new double[] { knot, face, length, compliance(length), 1 });
			}
		}

		int lineStart = links.size();
		int[] canopy = new int[devices.size()];
		int[] knot = new int[devices.size()];
		double[] dragArea = new double[keys.size() + open];
		Arrays.fill(canopy, -1);
		Arrays.fill(knot, -1);
		for (int d = 0; d < open; d++) {
			Device device = devices.get(d);
			canopy[d] = keys.size();
			knot[d] = knotOfJoint[device.joint()];
			keys.add(CANOPY_KEY + d);
			masses.add(Math.max(0.005, device.mass()));
			contacts.add(CANOPY_RADIUS);
			links.add(new double[] { knot[d], canopy[d], device.lines(), compliance(device.lines()), 1 });
		}
		// Drag of each knot stands for its shock cord's drag.
		for (int joint : joints) {
			double cord = 0;
			for (int d = 0; d < open; d++) {
				if (devices.get(d).joint() == joint) {
					cord = Math.max(cord, devices.get(d).cord());
				}
			}
			dragArea[knotOfJoint[joint]] = CROSS_CD * cord * CORD_DIAMETER;
		}

		int[] keyArray = keys.stream().mapToInt(Integer::intValue).toArray();
		double[] massArray = masses.stream().mapToDouble(Double::doubleValue).toArray();
		double[] contactArray = contacts.stream().mapToDouble(Double::doubleValue).toArray();
		return new Layout(open, keyArray, massArray, contactArray, rodEnds, rodSections, rodLength, links, cordStart,
				lineStart, canopy, knot, Arrays.copyOf(dragArea, keyArray.length));
	}

	/** Compliance of a cord of this length: it stretches {@link #STATIC_STRETCH} under the whole weight. */
	private double compliance(double length) {
		return STATIC_STRETCH * Math.max(length, 0.05) / (totalMass * G);
	}

	// ---------- simulation ----------

	private void run(double stop) {
		double[] target = new double[3];
		double[] targetVelocity = new double[3];
		double[] ahead = new double[3];
		double[] behind = new double[3];
		double[] wind = new double[3];
		double[] centre = new double[3];
		double[] centreVelocity = new double[3];
		double[] offset = new double[3];
		double[] steer = new double[3];
		double[] trim = new double[3];

		int next = 0;
		double t = start;
		while (next < devices.size() && devices.get(next).time() <= t) {
			next++;
		}
		begin(next);
		massCentre(centre, centreVelocity);
		course.centre(t, target);
		for (int c = 0; c < 3; c++) {
			offset[c] = centre[c] - target[c];
		}
		boolean released = landed();
		double releaseTime = released ? t : Double.NaN;
		double lastEvent = t;
		record();

		int frame = 0;
		while (t < stop - 1e-9) {
			boolean busy = released || t - lastEvent < BUSY_TIME;
			int substeps = busy ? FINE_SUBSTEPS : COARSE_SUBSTEPS;
			double dt = FRAME / substeps;
			for (int s = 0; s < substeps; s++) {
				while (next < devices.size() && devices.get(next).time() <= t) {
					open(next + 1);
					next++;
					lastEvent = t;
				}
				double rho = course.airDensity(t);
				course.wind(t, wind);

				// Steer the mass centre onto the simulated point while airborne.
				Arrays.fill(steer, 0);
				if (!released) {
					double blend = 1 - ReplayBody.smoothstep((t - start) / CENTRE_BLEND_TIME);
					double later = Math.min(end, t + 0.05);
					double earlier = Math.min(later - 0.02, t - 0.05);
					course.centre(t, target);
					course.centre(later, ahead);
					course.centre(earlier, behind);
					massCentre(centre, centreVelocity);
					for (int c = 0; c < 3; c++) {
						targetVelocity[c] = (ahead[c] - behind[c]) / (later - earlier);
						double error = target[c] + offset[c] * blend - centre[c];
						steer[c] = TRACK_RATE * TRACK_RATE * error
								+ 2 * TRACK_RATE * (targetVelocity[c] - centreVelocity[c]);
						trim[c] += (steer[c] - trim[c]) * Math.min(1, dt / TRIM_SMOOTHING);
					}
				} else {
					double fade = Math.max(0, 1 - (t - releaseTime) / TRIM_FADE);
					for (int c = 0; c < 3; c++) {
						steer[c] = trim[c] * fade;
					}
				}
				contacts = released || lowest() < 2;
				step(t, dt, rho, wind, steer);
				t += dt;
				if (!released && (landed() || Double.isFinite(groundHit) && t >= groundHit)) {
					released = true;
					releaseTime = t;
				}
				if (released) {
					markTouchdowns(t);
				}
			}
			frame++;
			t = start + frame * FRAME;
			record();
		}
	}

	/** Sets up the parts as the rigid body at the first opening. */
	private void begin(int open) {
		layout = layouts[open];
		int n = layout.particles();
		x = new double[3 * n];
		v = new double[3 * n];
		previous = new double[3 * n];
		double h = 0.02;
		double[] now = new double[3];
		double[] before = new double[3];
		for (int p = 0; p < n; p++) {
			double station = stationOf(layout.keys[p]);
			course.rigidPoint(station, start, now);
			course.rigidPoint(station, start - h, before);
			for (int c = 0; c < 3; c++) {
				x[3 * p + c] = now[c];
				v[3 * p + c] = (now[c] - before[c]) / h;
			}
		}
		limitSpin();
		int rods = layout.rods();
		axis = new double[3 * rods];
		up = new double[3 * rods];
		double[] rigidUp = new double[3];
		course.rigidUp(start, rigidUp);
		for (int r = 0; r < rods; r++) {
			System.arraycopy(rigidUp, 0, up, 3 * r, 3);
		}
		refreshAxes();
		for (int joint : openJoints(open)) {
			eject(joint);
		}
	}

	/** Opens the next device: splits the piece at its joint if it is still closed. */
	private void open(int open) {
		Layout old = layout;
		Layout now = layouts[open];
		double[] oldX = x, oldV = v, oldUp = up;
		int n = now.particles();
		x = new double[3 * n];
		v = new double[3 * n];
		previous = new double[3 * n];
		for (int p = 0; p < n; p++) {
			int key = now.keys[p];
			int same = old.indexOf(key);
			if (same >= 0) {
				System.arraycopy(oldX, 3 * same, x, 3 * p, 3);
				System.arraycopy(oldV, 3 * same, v, 3 * p, 3);
			} else {
				// A new canopy starts on its knot, which may itself be new at the joint.
				int knotKey = key >= CANOPY_KEY ? now.keys[now.knot[key - CANOPY_KEY]] : key;
				pointOn(old, oldX, oldV, old.indexOf(knotKey), stationOf(key), p);
			}
		}
		up = new double[3 * now.rods()];
		for (int r = 0; r < now.rods(); r++) {
			int oldRod = rodHolding(old, now.rodFirst[r]);
			System.arraycopy(oldUp, 3 * oldRod, up, 3 * r, 3);
		}
		layout = now;
		axis = new double[3 * now.rods()];
		refreshAxes();
		int joint = devices.get(open - 1).joint();
		if (!openJoints(open - 1).contains(joint)) {
			eject(joint);
		}
	}

	/** Puts particle {@code p} at a station along the old pieces, or on an old particle when given. */
	private void pointOn(Layout old, double[] oldX, double[] oldV, int oldParticle, double station, int p) {
		if (oldParticle >= 0) {
			System.arraycopy(oldX, 3 * oldParticle, x, 3 * p, 3);
			System.arraycopy(oldV, 3 * oldParticle, v, 3 * p, 3);
			return;
		}
		int section = sectionAt(station);
		int rod = rodHolding(old, section);
		int a = old.rodFront[rod], b = old.rodAft[rod];
		double from = faces[old.rodFirst[rod]];
		double f = (station - from) / old.rodLength[rod];
		for (int c = 0; c < 3; c++) {
			x[3 * p + c] = oldX[3 * a + c] + (oldX[3 * b + c] - oldX[3 * a + c]) * f;
			v[3 * p + c] = oldV[3 * a + c] + (oldV[3 * b + c] - oldV[3 * a + c]) * f;
		}
	}

	private int sectionAt(double station) {
		int section = 0;
		while (section < sections() - 1 && station >= faces[section + 1] - 1e-9) {
			section++;
		}
		return section;
	}

	private static int rodHolding(Layout layout, int section) {
		for (int r = 0; r < layout.rods(); r++) {
			if (section >= layout.rodFirst[r] && section <= layout.rodLast[r]) {
				return r;
			}
		}
		return 0;
	}

	private double stationOf(int key) {
		if (key >= CANOPY_KEY) {
			return faces[devices.get(key - CANOPY_KEY).joint()];
		}
		if (key >= KNOT_KEY) {
			return faces[key - KNOT_KEY];
		}
		return faces[key / 2 + (key % 2)];
	}

	/**
	 * The ejection charge at a joint pushes the pieces either side of it apart along their axis,
	 * and throws the knot and packed canopy out with the forward piece.
	 */
	private void eject(int joint) {
		int n = sections();
		int forward = joint > 0 ? layout.indexOf(faceKey(joint - 1, true)) : -1;
		int aft = joint < n ? layout.indexOf(faceKey(joint, false)) : -1;
		int forwardRod = forward >= 0 ? rodHolding(layout, joint - 1) : -1;
		int aftRod = aft >= 0 ? rodHolding(layout, joint) : -1;
		// Unit vector from the joint towards the nose
		double[] noseward = new double[3];
		int rod = forwardRod >= 0 ? forwardRod : aftRod;
		for (int c = 0; c < 3; c++) {
			noseward[c] = -axis[3 * rod + c];
		}
		double forwardMass = forwardRod >= 0 ? rodMass(forwardRod) : 0;
		double aftMass = aftRod >= 0 ? rodMass(aftRod) : 0;
		double canopyMass = 0;
		List<Integer> loose = new ArrayList<>();
		for (int p = 0; p < layout.particles(); p++) {
			int key = layout.keys[p];
			boolean here = key == KNOT_KEY + joint
					|| key >= CANOPY_KEY && devices.get(key - CANOPY_KEY).joint() == joint;
			if (here) {
				loose.add(p);
				canopyMass += layout.mass[p];
			}
		}
		if (forwardRod < 0 || aftRod < 0) {
			// An end joint: the charge throws the canopy out of the open end.
			double sign = forwardRod < 0 ? 1 : -1;
			double bodyMass = forwardRod >= 0 ? forwardMass : aftMass;
			double total = bodyMass + canopyMass;
			push(loose, noseward, sign * EJECTION_SPEED * bodyMass / total);
			pushRod(rod, noseward, -sign * EJECTION_SPEED * canopyMass / total);
			return;
		}
		double total = forwardMass + aftMass;
		double forwardSpeed = EJECTION_SPEED * aftMass / total;
		pushRod(forwardRod, noseward, forwardSpeed);
		pushRod(aftRod, noseward, -EJECTION_SPEED * forwardMass / total);
		push(loose, noseward, forwardSpeed);
	}

	private double rodMass(int rod) {
		return layout.mass[layout.rodFront[rod]] + layout.mass[layout.rodAft[rod]];
	}

	private void pushRod(int rod, double[] direction, double speed) {
		push(List.of(layout.rodFront[rod], layout.rodAft[rod]), direction, speed);
	}

	private void push(List<Integer> particles, double[] direction, double speed) {
		for (int p : particles) {
			for (int c = 0; c < 3; c++) {
				v[3 * p + c] += direction[c] * speed;
			}
		}
	}

	/** Keeps a tumbling rigid body from flinging its pieces apart faster than a few metres a second. */
	private void limitSpin() {
		double[] centre = new double[3];
		double[] velocity = new double[3];
		massCentre(centre, velocity);
		double limit = 15;
		for (int p = 0; p < layout.particles(); p++) {
			double dx = v[3 * p] - velocity[0], dy = v[3 * p + 1] - velocity[1], dz = v[3 * p + 2] - velocity[2];
			double speed = Math.sqrt(dx * dx + dy * dy + dz * dz);
			if (speed > limit) {
				double k = limit / speed;
				v[3 * p] = velocity[0] + dx * k;
				v[3 * p + 1] = velocity[1] + dy * k;
				v[3 * p + 2] = velocity[2] + dz * k;
			}
		}
	}

	private void massCentre(double[] position, double[] velocity) {
		Arrays.fill(position, 0);
		Arrays.fill(velocity, 0);
		double total = 0;
		for (int p = 0; p < layout.particles(); p++) {
			double m = layout.mass[p];
			for (int c = 0; c < 3; c++) {
				position[c] += m * x[3 * p + c];
				velocity[c] += m * v[3 * p + c];
			}
			total += m;
		}
		for (int c = 0; c < 3; c++) {
			position[c] /= total;
			velocity[c] /= total;
		}
	}

	private double lowest() {
		double lowest = Double.MAX_VALUE;
		for (int p = 0; p < layout.particles(); p++) {
			lowest = Math.min(lowest, x[3 * p + 1]);
		}
		return lowest;
	}

	private boolean landed() {
		for (int r = 0; r < layout.rods(); r++) {
			if (x[3 * layout.rodFront[r] + 1] <= radius + 1e-3 || x[3 * layout.rodAft[r] + 1] <= radius + 1e-3) {
				return true;
			}
		}
		return false;
	}

	private void markTouchdowns(double t) {
		for (int d = 0; d < layout.open; d++) {
			int p = layout.canopy[d];
			if (Double.isNaN(touchdown[d]) && x[3 * p + 1] <= CANOPY_RADIUS + 0.01) {
				touchdown[d] = t;
			}
		}
	}

	private void step(double t, double dt, double rho, double[] wind, double[] steer) {
		int n = layout.particles();
		// Forces
		for (int p = 0; p < n; p++) {
			v[3 * p] += steer[0] * dt;
			v[3 * p + 1] += (steer[1] - G) * dt;
			v[3 * p + 2] += steer[2] * dt;
		}
		for (int r = 0; r < layout.rods(); r++) {
			rodDrag(r, dt, rho, wind);
		}
		for (int p = 0; p < n; p++) {
			int key = layout.keys[p];
			double area = layout.dragArea[p];
			if (key >= CANOPY_KEY) {
				int d = key - CANOPY_KEY;
				double fill = 0.15 + 0.85 * inflation(d, t);
				double down = Double.isNaN(touchdown[d]) ? 0 : ReplayBody.smoothstep((t - touchdown[d]) / COLLAPSE_TIME);
				area = devices.get(d).dragArea() * fill * fill * (1 - 0.95 * down);
			}
			if (area > 0) {
				drag(p, 0.5 * rho * area * layout.inverseMass[p], dt, wind);
			}
		}
		// Move
		for (int k = 0; k < 3 * n; k++) {
			previous[k] = x[k];
			x[k] += v[k] * dt;
		}
		// Constraints
		if (lambda.length != layout.linkA.length) {
			lambda = new double[layout.linkA.length];
		}
		Arrays.fill(lambda, 0);
		if (lift.length != n) {
			lift = new double[n];
		}
		Arrays.fill(lift, 0);
		int iterations = contacts ? CONTACT_ITERATIONS : ITERATIONS;
		// Ground and contacts first, rods last, so the pieces never visibly bend or stretch; what is
		// left in the ground is pushed out next step without throwing anything up.
		for (int iteration = 0; iteration < iterations; iteration++) {
			ground();
			collideRods();
			for (int l = 0; l < layout.linkA.length; l++) {
				lambda[l] += solveLink(l, dt, lambda[l]);
			}
		}
		friction();
		// Velocities. The ground stops what hits it; it never throws anything back up.
		for (int k = 0; k < 3 * n; k++) {
			v[k] = (x[k] - previous[k]) / dt;
			if (k % 3 == 1 && lift[k / 3] > 0 && v[k] > 0) {
				v[k] = 0;
			}
			if (v[k] > MAX_SPEED) {
				v[k] = MAX_SPEED;
			} else if (v[k] < -MAX_SPEED) {
				v[k] = -MAX_SPEED;
			}
		}
		stiffenRods();
		refreshAxes();
	}

	/** Takes out any stretching or squashing speed along each rod, keeping its momentum. */
	private void stiffenRods() {
		for (int r = 0; r < layout.rods(); r++) {
			int a = layout.rodFront[r], b = layout.rodAft[r];
			double nx = x[3 * b] - x[3 * a], ny = x[3 * b + 1] - x[3 * a + 1], nz = x[3 * b + 2] - x[3 * a + 2];
			double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
			if (length < 1e-9) {
				continue;
			}
			nx /= length;
			ny /= length;
			nz /= length;
			double rate = (v[3 * b] - v[3 * a]) * nx + (v[3 * b + 1] - v[3 * a + 1]) * ny + (v[3 * b + 2] - v[3 * a + 2]) * nz;
			double wa = layout.inverseMass[a], wb = layout.inverseMass[b];
			double impulse = rate / (wa + wb);
			v[3 * a] += wa * impulse * nx;
			v[3 * a + 1] += wa * impulse * ny;
			v[3 * a + 2] += wa * impulse * nz;
			v[3 * b] -= wb * impulse * nx;
			v[3 * b + 1] -= wb * impulse * ny;
			v[3 * b + 2] -= wb * impulse * nz;
		}
	}

	/** Quadratic drag across and along a rod, at each end, solved implicitly so it never overshoots. */
	private void rodDrag(int r, double dt, double rho, double[] wind) {
		double length = layout.rodLength[r];
		double diameter = 2 * radius;
		double across = 0.5 * rho * CROSS_CD * length * diameter / 2;
		double along = 0.5 * rho * AXIAL_CD * Math.PI * diameter * diameter / 8;
		double ax = axis[3 * r], ay = axis[3 * r + 1], az = axis[3 * r + 2];
		for (int p : new int[] { layout.rodFront[r], layout.rodAft[r] }) {
			double w = layout.inverseMass[p];
			double ux = v[3 * p] - wind[0], uy = v[3 * p + 1] - wind[1], uz = v[3 * p + 2] - wind[2];
			double dot = ux * ax + uy * ay + uz * az;
			double px = dot * ax, py = dot * ay, pz = dot * az;
			double nx = ux - px, ny = uy - py, nz = uz - pz;
			double fa = 1 / (1 + along * w * Math.abs(dot) * dt);
			double fn = 1 / (1 + across * w * Math.sqrt(nx * nx + ny * ny + nz * nz) * dt);
			v[3 * p] = wind[0] + px * fa + nx * fn;
			v[3 * p + 1] = wind[1] + py * fa + ny * fn;
			v[3 * p + 2] = wind[2] + pz * fa + nz * fn;
		}
	}

	private void drag(int p, double k, double dt, double[] wind) {
		double ux = v[3 * p] - wind[0], uy = v[3 * p + 1] - wind[1], uz = v[3 * p + 2] - wind[2];
		double f = 1 / (1 + k * Math.sqrt(ux * ux + uy * uy + uz * uz) * dt);
		v[3 * p] = wind[0] + ux * f;
		v[3 * p + 1] = wind[1] + uy * f;
		v[3 * p + 2] = wind[2] + uz * f;
	}

	/** One XPBD pass over a link, returning the change in its multiplier. */
	private double solveLink(int l, double dt, double lambda) {
		int a = layout.linkA[l], b = layout.linkB[l];
		double dx = x[3 * b] - x[3 * a], dy = x[3 * b + 1] - x[3 * a + 1], dz = x[3 * b + 2] - x[3 * a + 2];
		double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
		double c = distance - layout.linkLength[l];
		if (distance < 1e-9 || layout.linkSlack[l] && c <= 0) {
			return 0;
		}
		double nx = dx / distance, ny = dy / distance, nz = dz / distance;
		double wa = layout.inverseMass[a], wb = layout.inverseMass[b];
		double alpha = layout.linkCompliance[l] / (dt * dt);
		double gamma = layout.linkCompliance[l] * layout.linkDamping[l] / dt;
		// Rate of stretching this step, for damping
		double rate = nx * (x[3 * b] - previous[3 * b] - x[3 * a] + previous[3 * a])
				+ ny * (x[3 * b + 1] - previous[3 * b + 1] - x[3 * a + 1] + previous[3 * a + 1])
				+ nz * (x[3 * b + 2] - previous[3 * b + 2] - x[3 * a + 2] + previous[3 * a + 2]);
		double delta = (-c - alpha * lambda - gamma * rate) / ((1 + gamma) * (wa + wb) + alpha);
		if (layout.linkSlack[l] && lambda + delta > 0) {
			delta = -lambda;	// a cord pulls, never pushes
		}
		x[3 * a] -= wa * delta * nx;
		x[3 * a + 1] -= wa * delta * ny;
		x[3 * a + 2] -= wa * delta * nz;
		x[3 * b] += wb * delta * nx;
		x[3 * b + 1] += wb * delta * ny;
		x[3 * b + 2] += wb * delta * nz;
		return delta;
	}

	/** Keeps pieces of airframe from passing through each other. */
	private void collideRods() {
		int rods = layout.rods();
		double[] closest = new double[2];
		for (int i = 0; i < rods; i++) {
			for (int j = i + 1; j < rods; j++) {
				int a0 = layout.rodFront[i], a1 = layout.rodAft[i];
				int b0 = layout.rodFront[j], b1 = layout.rodAft[j];
				closestPoints(a0, a1, b0, b1, closest);
				double s = closest[0], u = closest[1];
				double dx = 0, dy = 0, dz = 0;
				for (int c = 0; c < 3; c++) {
					double pa = x[3 * a0 + c] + (x[3 * a1 + c] - x[3 * a0 + c]) * s;
					double pb = x[3 * b0 + c] + (x[3 * b1 + c] - x[3 * b0 + c]) * u;
					double d = pb - pa;
					if (c == 0) {
						dx = d;
					} else if (c == 1) {
						dy = d;
					} else {
						dz = d;
					}
				}
				double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
				if (distance > 2 * radius) {
					continue;
				}
				if (distance < 1e-6) {
					// Lined up end to end or one inside the other: part them along the line between their middles.
					for (int c = 0; c < 3; c++) {
						double d = 0.5 * (x[3 * b0 + c] + x[3 * b1 + c] - x[3 * a0 + c] - x[3 * a1 + c]);
						if (c == 0) {
							dx = d;
						} else if (c == 1) {
							dy = d;
						} else {
							dz = d;
						}
					}
					double between = Math.sqrt(dx * dx + dy * dy + dz * dz);
					if (between < 1e-6) {
						continue;
					}
					dx /= between;
					dy /= between;
					dz /= between;
				} else {
					dx /= distance;
					dy /= distance;
					dz /= distance;
				}
				double nx = dx, ny = dy, nz = dz;
				// Tube ends are flat, so pieces meeting end on come closer than their round sides.
				boolean endOn = endOn(i, s, nx, ny, nz) || endOn(j, u, nx, ny, nz);
				double overlap = (endOn ? END_GAP : 2 * radius) - distance;
				if (overlap <= 0) {
					continue;
				}
				overlap = Math.min(overlap, 0.5 * radius);
				double[] weights = { 1 - s, s, -(1 - u), -u };
				int[] particles = { a0, a1, b0, b1 };
				double sum = 0;
				for (int k = 0; k < 4; k++) {
					sum += layout.inverseMass[particles[k]] * weights[k] * weights[k];
				}
				double scale = overlap / sum;
				for (int k = 0; k < 4; k++) {
					double move = -scale * layout.inverseMass[particles[k]] * weights[k];
					x[3 * particles[k]] += move * nx;
					x[3 * particles[k] + 1] += move * ny;
					x[3 * particles[k] + 2] += move * nz;
				}
			}
		}
	}

	private boolean endOn(int rod, double along, double nx, double ny, double nz) {
		boolean end = along <= 1e-6 || along >= 1 - 1e-6;
		return end && Math.abs(nx * axis[3 * rod] + ny * axis[3 * rod + 1] + nz * axis[3 * rod + 2]) > 0.7;
	}

	/** Parameters along two segments of their closest points, each 0 to 1. */
	private void closestPoints(int a0, int a1, int b0, int b1, double[] out) {
		double[] d1 = new double[3], d2 = new double[3], r = new double[3];
		for (int c = 0; c < 3; c++) {
			d1[c] = x[3 * a1 + c] - x[3 * a0 + c];
			d2[c] = x[3 * b1 + c] - x[3 * b0 + c];
			r[c] = x[3 * a0 + c] - x[3 * b0 + c];
		}
		double a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
		double s, u;
		if (a < 1e-12 && e < 1e-12) {
			s = u = 0;
		} else if (a < 1e-12) {
			s = 0;
			u = clamp(f / e);
		} else {
			double c = dot(d1, r);
			if (e < 1e-12) {
				u = 0;
				s = clamp(-c / a);
			} else {
				double b = dot(d1, d2);
				double denominator = a * e - b * b;
				s = denominator > 1e-12 ? clamp((b * f - c * e) / denominator) : 0;
				u = (b * s + f) / e;
				if (u < 0) {
					u = 0;
					s = clamp(-c / a);
				} else if (u > 1) {
					u = 1;
					s = clamp((b - c) / a);
				}
			}
		}
		out[0] = s;
		out[1] = u;
	}

	private static double dot(double[] a, double[] b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	private static double clamp(double value) {
		return Math.max(0, Math.min(1, value));
	}

	/** Holds everything above the ground. */
	private void ground() {
		for (int p = 0; p < layout.particles(); p++) {
			double depth = layout.contact[p] - x[3 * p + 1];
			if (depth > 0) {
				x[3 * p + 1] += depth;
				lift[p] += depth;
			}
		}
	}

	/** Coulomb friction where the ground pushed back: the push limits how much sliding it stops. */
	private void friction() {
		for (int p = 0; p < layout.particles(); p++) {
			double depth = lift[p];
			if (depth <= 0) {
				continue;
			}
			double sx = x[3 * p] - previous[3 * p], sz = x[3 * p + 2] - previous[3 * p + 2];
			double slide = Math.sqrt(sx * sx + sz * sz);
			if (slide < STATIC_FRICTION * depth) {
				x[3 * p] = previous[3 * p];
				x[3 * p + 2] = previous[3 * p + 2];
			} else if (slide > 0) {
				double k = Math.min(1, SLIDING_FRICTION * depth / slide);
				x[3 * p] -= sx * k;
				x[3 * p + 2] -= sz * k;
			}
		}
	}

	/** Updates each rod's axis, carrying its roll reference round with it. */
	private void refreshAxes() {
		for (int r = 0; r < layout.rods(); r++) {
			int a = layout.rodFront[r], b = layout.rodAft[r];
			double ax = x[3 * b] - x[3 * a], ay = x[3 * b + 1] - x[3 * a + 1], az = x[3 * b + 2] - x[3 * a + 2];
			double length = Math.sqrt(ax * ax + ay * ay + az * az);
			if (length < 1e-9) {
				continue;
			}
			ax /= length;
			ay /= length;
			az /= length;
			double ox = axis[3 * r], oy = axis[3 * r + 1], oz = axis[3 * r + 2];
			double ux = up[3 * r], uy = up[3 * r + 1], uz = up[3 * r + 2];
			double cos = ox * ax + oy * ay + oz * az;
			if (ox * ox + oy * oy + oz * oz > 0.5 && cos < 0.9999999) {
				// Rotate the reference by the turn from the old axis to the new one.
				double kx = oy * az - oz * ay, ky = oz * ax - ox * az, kz = ox * ay - oy * ax;
				double sin = Math.sqrt(kx * kx + ky * ky + kz * kz);
				if (sin > 1e-12) {
					kx /= sin;
					ky /= sin;
					kz /= sin;
					double kDotU = kx * ux + ky * uy + kz * uz;
					double cx = ky * uz - kz * uy, cy = kz * ux - kx * uz, cz = kx * uy - ky * ux;
					double rx = ux * cos + cx * sin + kx * kDotU * (1 - cos);
					double ry = uy * cos + cy * sin + ky * kDotU * (1 - cos);
					double rz = uz * cos + cz * sin + kz * kDotU * (1 - cos);
					ux = rx;
					uy = ry;
					uz = rz;
				}
			}
			double along = ux * ax + uy * ay + uz * az;
			ux -= along * ax;
			uy -= along * ay;
			uz -= along * az;
			double norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
			if (norm < 1e-6) {
				// Any direction square to the axis
				double hx = Math.abs(ay) < 0.9 ? 0 : 1, hy = Math.abs(ay) < 0.9 ? 1 : 0;
				ux = hy * az;
				uy = -hx * az;
				uz = hx * ay - hy * ax;
				norm = Math.sqrt(ux * ux + uy * uy + uz * uz);
			}
			axis[3 * r] = ax;
			axis[3 * r + 1] = ay;
			axis[3 * r + 2] = az;
			up[3 * r] = ux / norm;
			up[3 * r + 1] = uy / norm;
			up[3 * r + 2] = uz / norm;
		}
	}

	private void record() {
		int n = 3 * layout.particles();
		float[] frame = new float[n + up.length];
		for (int k = 0; k < n; k++) {
			frame[k] = (float) x[k];
		}
		for (int k = 0; k < up.length; k++) {
			frame[n + k] = (float) up[k];
		}
		frames.add(frame);
		frameLayouts.add(layout);
	}
}
