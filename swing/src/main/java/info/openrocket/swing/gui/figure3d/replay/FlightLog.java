package info.openrocket.swing.gui.figure3d.replay;

import java.util.Collections;
import java.util.List;

/**
 * A flight as an altimeter or flight computer recorded it, in SI units: heights in metres above
 * the pad, vertical speed in metres per second, on the logger's own clock in seconds.
 *
 * <p>Optional channels are null when the log does not record them.</p>
 *
 * @param source      file name the log came from
 * @param format      the logger layout recognised, such as "Eggtimer" or "AltOS"
 * @param time        sample times, seconds, increasing
 * @param altitude    height above the pad, metres
 * @param velocity    vertical speed, metres per second (from the log or from the height)
 * @param tilt        angle of the rocket axis from vertical, degrees, or null
 * @param roll        roll angle, degrees, or null
 * @param latitude    GPS latitude, degrees, or null
 * @param longitude   GPS longitude, degrees, or null
 * @param liftoff     liftoff time on the log's clock
 * @param apogee      apogee time on the log's clock
 * @param landing     landing time on the log's clock, or NaN when the log ends first
 * @param deployments recovery deployments the log records, in time order; empty when it records none
 * @param notes       how the units and events were decided, for the user
 */
record FlightLog(String source, String format, double[] time, double[] altitude, double[] velocity,
		double[] tilt, double[] roll, double[] latitude, double[] longitude,
		double liftoff, double apogee, double landing, List<Double> deployments, List<String> notes) {

	FlightLog {
		deployments = Collections.unmodifiableList(deployments);
		notes = Collections.unmodifiableList(notes);
	}

	/** Highest height above the pad, metres. */
	double apogeeHeight() {
		return FlightReplayData.interpolate(time, altitude, apogee);
	}

	boolean hasGps() {
		return latitude != null && longitude != null;
	}
}
