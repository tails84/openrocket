package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.swing.gui.figure3d.animation.FlightPoseProvider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Time-indexed view of one simulated flight branch for the flight replay: pose,
 * readout values, the roll angle, flight events and the flight phase at any time.
 * Values between data points are linear interpolations.
 */
public final class FlightReplayData {

	/** What the rocket is doing, in flight order. */
	public enum Phase {
		ON_PAD, ON_RAIL, POWERED, COASTING, DESCENDING, UNDER_RECOVERY, LANDED
	}

	/** A flight event shown on the replay timeline. */
	public record Event(double time, FlightEvent.Type type, RocketComponent source) {
	}

	/** Event types worth marking on a replay timeline, in the order they usually happen. */
	private static final Set<FlightEvent.Type> MARKED_EVENTS = EnumSet.of(
			FlightEvent.Type.LIFTOFF,
			FlightEvent.Type.LAUNCHROD,
			FlightEvent.Type.IGNITION,
			FlightEvent.Type.BURNOUT,
			FlightEvent.Type.STAGE_SEPARATION,
			FlightEvent.Type.APOGEE,
			FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,
			FlightEvent.Type.TUMBLE,
			FlightEvent.Type.GROUND_HIT);

	/** Thrust below this is treated as no thrust, in newtons. */
	private static final double THRUST_THRESHOLD = 0.01;

	private final FlightPoseProvider pose;
	private final double[] time;
	private final Map<FlightDataType, double[]> channels = new HashMap<>();
	private final double[] rollAngle;
	private final List<Event> events;
	private final List<Event> deployments;
	private final double liftoffTime;
	private final double railExitTime;
	private final double lastBurnoutTime;
	private final double apogeeTime;
	private final double groundHitTime;

	private FlightReplayData(FlightDataBranch branch) {
		this.pose = FlightPoseProvider.fromFlightDataBranch(branch);
		this.time = toArray(branch.get(FlightDataType.TYPE_TIME));
		for (FlightDataType type : branch.getTypes()) {
			List<Double> values = branch.get(type);
			if (values != null && values.size() >= time.length) {
				double[] filled = fillGaps(toArray(values), time.length);
				if (filled != null) {
					channels.put(type, filled);
				}
			}
		}
		this.rollAngle = integrate(time, channels.get(FlightDataType.TYPE_ROLL_RATE));

		List<Event> marked = new ArrayList<>();
		List<Event> deployed = new ArrayList<>();
		double liftoff = Double.NaN, railExit = Double.NaN, burnout = Double.NaN;
		double apogee = Double.NaN, groundHit = Double.NaN;
		for (FlightEvent event : branch.getEvents()) {
			FlightEvent.Type type = event.getType();
			double t = event.getTime();
			switch (type) {
				case LIFTOFF -> liftoff = Double.isNaN(liftoff) ? t : liftoff;
				case LAUNCHROD -> railExit = Double.isNaN(railExit) ? t : railExit;
				case BURNOUT -> burnout = Double.isNaN(burnout) ? t : Math.max(burnout, t);
				case APOGEE -> apogee = Double.isNaN(apogee) ? t : apogee;
				case GROUND_HIT -> groundHit = Double.isNaN(groundHit) ? t : groundHit;
				default -> { }
			}
			if (!MARKED_EVENTS.contains(type)) {
				continue;
			}
			// Ignition at liftoff is the launch itself; later ignitions are staging or air starts.
			if (type == FlightEvent.Type.IGNITION && (Double.isNaN(liftoff) || t <= liftoff)) {
				continue;
			}
			Event replayEvent = new Event(t, type, event.getSource());
			marked.add(replayEvent);
			if (type == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT && event.getSource() instanceof RecoveryDevice) {
				deployed.add(replayEvent);
			}
		}
		this.events = Collections.unmodifiableList(marked);
		this.deployments = Collections.unmodifiableList(deployed);
		this.liftoffTime = liftoff;
		this.railExitTime = railExit;
		this.lastBurnoutTime = burnout;
		this.apogeeTime = apogee;
		this.groundHitTime = groundHit;
	}

	/**
	 * Builds the replay view of a flight data branch.
	 *
	 * @param branch simulated flight data with time, position and altitude
	 * @return the replay data
	 * @throws IllegalArgumentException if the branch lacks time or position data
	 */
	public static FlightReplayData fromBranch(FlightDataBranch branch) {
		List<Double> time = branch.get(FlightDataType.TYPE_TIME);
		if (time == null || time.size() < 2) {
			throw new IllegalArgumentException("Flight data branch has fewer than two data points");
		}
		return new FlightReplayData(branch);
	}

	public FlightPoseProvider getPose() {
		return pose;
	}

	public double getStartTime() {
		return time[0];
	}

	public double getEndTime() {
		return time[time.length - 1];
	}

	/** Times of the recorded data points, in seconds. */
	public double[] getTimes() {
		return time.clone();
	}

	/**
	 * Returns whether the branch recorded the given data type.
	 */
	public boolean hasChannel(FlightDataType type) {
		return channels.containsKey(type);
	}

	/**
	 * Interpolates a recorded value at the given time.
	 *
	 * @return the value in SI units, or NaN when the branch did not record the type
	 */
	public double getValue(FlightDataType type, double t) {
		double[] values = channels.get(type);
		if (values == null) {
			return Double.NaN;
		}
		return interpolate(time, values, t);
	}

	/** Rotation of the rocket about its own axis since the start of the data, in radians. */
	public double getRollAngle(double t) {
		return rollAngle == null ? 0.0 : interpolate(time, rollAngle, t);
	}

	/** Returns whether any motor produces thrust at the given time. */
	public boolean isThrusting(double t) {
		return getValue(FlightDataType.TYPE_THRUST_FORCE, t) > THRUST_THRESHOLD;
	}

	/** Flight events to mark on a timeline, in time order. */
	public List<Event> getEvents() {
		return events;
	}

	/** Recovery device deployments, in time order. */
	public List<Event> getDeployments() {
		return deployments;
	}

	/** Time of the first recovery device deployment, or NaN when nothing deploys. */
	public double getFirstDeploymentTime() {
		return deployments.isEmpty() ? Double.NaN : deployments.get(0).time();
	}

	public double getLiftoffTime() {
		return liftoffTime;
	}

	public double getApogeeTime() {
		return apogeeTime;
	}

	public double getGroundHitTime() {
		return groundHitTime;
	}

	/** Returns what the rocket is doing at the given time. */
	public Phase getPhase(double t) {
		if (!Double.isNaN(groundHitTime) && t >= groundHitTime) {
			return Phase.LANDED;
		}
		double deployment = getFirstDeploymentTime();
		if (!Double.isNaN(deployment) && t >= deployment) {
			return Phase.UNDER_RECOVERY;
		}
		if (Double.isNaN(liftoffTime) || t < liftoffTime) {
			return Phase.ON_PAD;
		}
		if (!Double.isNaN(railExitTime) && t < railExitTime) {
			return Phase.ON_RAIL;
		}
		if (isThrusting(t) || (!Double.isNaN(lastBurnoutTime) && t < lastBurnoutTime)) {
			return Phase.POWERED;
		}
		if (Double.isNaN(apogeeTime) || t < apogeeTime) {
			return Phase.COASTING;
		}
		return Phase.DESCENDING;
	}

	// ---------- helpers ----------

	private static double[] toArray(List<Double> values) {
		double[] out = new double[values.size()];
		for (int i = 0; i < out.length; i++) {
			Double value = values.get(i);
			out[i] = value == null ? Double.NaN : value;
		}
		return out;
	}

	/**
	 * Truncates to {@code length} and replaces non-finite values with the nearest earlier
	 * finite value, or the first finite value for leading gaps.
	 *
	 * @return the filled values, or null when none is finite
	 */
	static double[] fillGaps(double[] values, int length) {
		double[] out = new double[length];
		double last = Double.NaN;
		int firstFinite = -1;
		for (int i = 0; i < length; i++) {
			if (Double.isFinite(values[i])) {
				last = values[i];
				if (firstFinite < 0) {
					firstFinite = i;
				}
			}
			out[i] = last;
		}
		if (firstFinite < 0) {
			return null;
		}
		for (int i = 0; i < firstFinite; i++) {
			out[i] = out[firstFinite];
		}
		return out;
	}

	private static double[] integrate(double[] t, double[] rate) {
		if (rate == null) {
			return null;
		}
		double[] out = new double[t.length];
		for (int i = 1; i < t.length; i++) {
			out[i] = out[i - 1] + 0.5 * (rate[i - 1] + rate[i]) * (t[i] - t[i - 1]);
		}
		return out;
	}

	static double interpolate(double[] ts, double[] vs, double t) {
		if (t <= ts[0]) {
			return vs[0];
		}
		int last = ts.length - 1;
		if (t >= ts[last]) {
			return vs[last];
		}
		int lo = 0, hi = last;
		while (hi - lo > 1) {
			int mid = (lo + hi) >>> 1;
			if (ts[mid] <= t) {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		double span = ts[hi] - ts[lo];
		if (span <= 0) {
			return vs[hi];
		}
		double u = (t - ts[lo]) / span;
		return vs[lo] + u * (vs[hi] - vs[lo]);
	}
}
