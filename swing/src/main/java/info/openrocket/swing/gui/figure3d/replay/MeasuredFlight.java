package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns a recorded flight log into flight data the replay can show beside the simulation.
 *
 * <p>The log's clock is shifted so its liftoff falls on the simulated liftoff. A log records the
 * height, and maybe the tilt and a GPS track; whatever it lacks comes from the simulation at the
 * same moment: the horizontal track without GPS, the heading always, and the attitude without a
 * tilt. Its recovery deployments are matched in order to the simulation's devices. A log that
 * records none gets them where the simulation's happen: at apogee, or at the height the simulated
 * deployment happens on the way down.</p>
 */
final class MeasuredFlight {
	private static final double EARTH_RADIUS = 6_371_000;
	/** A simulated deployment this close to apogee is taken to be an apogee deployment, seconds. */
	private static final double AT_APOGEE = 1.0;
	/** Half the span of readings averaged to steady the shown height, seconds. */
	private static final double STEADY_SECONDS = 0.25;

	private MeasuredFlight() {
	}

	/**
	 * @param log        the recorded flight
	 * @param simulation the main branch of the simulation to line it up with
	 * @param name       name to give the measured branch
	 */
	static FlightReplayData build(FlightLog log, FlightReplayData simulation, String name) {
		double shift = shift(log, simulation);

		FlightDataBranch branch = new FlightDataBranch(name,
				FlightDataType.TYPE_TIME,
				FlightDataType.TYPE_ALTITUDE,
				FlightDataType.TYPE_POSITION_X,
				FlightDataType.TYPE_POSITION_Y,
				FlightDataType.TYPE_VELOCITY_Z,
				FlightDataType.TYPE_ORIENTATION_THETA,
				FlightDataType.TYPE_ORIENTATION_PHI,
				FlightDataType.TYPE_ROLL_RATE,
				FlightDataType.TYPE_CG_LOCATION);

		double[] time = log.time();
		// Sensor noise of half a metre shows as a shaking rocket and a ragged trail close up.
		double[] altitude = steady(time, log.altitude());
		double padLatitude = log.hasGps() ? padValue(log, log.latitude()) : 0;
		double padLongitude = log.hasGps() ? padValue(log, log.longitude()) : 0;
		for (int i = 0; i < time.length; i++) {
			double t = time[i] + shift;
			branch.addPoint();
			branch.setValue(FlightDataType.TYPE_TIME, t);
			branch.setValue(FlightDataType.TYPE_ALTITUDE, altitude[i]);
			if (log.hasGps()) {
				double north = Math.toRadians(log.latitude()[i] - padLatitude) * EARTH_RADIUS;
				double east = Math.toRadians(log.longitude()[i] - padLongitude) * EARTH_RADIUS
						* Math.cos(Math.toRadians(padLatitude));
				branch.setValue(FlightDataType.TYPE_POSITION_X, east);
				branch.setValue(FlightDataType.TYPE_POSITION_Y, north);
			} else {
				branch.setValue(FlightDataType.TYPE_POSITION_X, simulation.getValue(FlightDataType.TYPE_POSITION_X, t));
				branch.setValue(FlightDataType.TYPE_POSITION_Y, simulation.getValue(FlightDataType.TYPE_POSITION_Y, t));
			}
			branch.setValue(FlightDataType.TYPE_VELOCITY_Z, log.velocity()[i]);
			double elevation = log.tilt() != null
					? Math.toRadians(90 - log.tilt()[i])
					: simulation.getValue(FlightDataType.TYPE_ORIENTATION_THETA, t);
			branch.setValue(FlightDataType.TYPE_ORIENTATION_THETA, elevation);
			branch.setValue(FlightDataType.TYPE_ORIENTATION_PHI, simulation.getValue(FlightDataType.TYPE_ORIENTATION_PHI, t));
			branch.setValue(FlightDataType.TYPE_ROLL_RATE, rollRate(log, i, simulation, t));
			branch.setValue(FlightDataType.TYPE_CG_LOCATION, simulation.getValue(FlightDataType.TYPE_CG_LOCATION, t));
		}

		List<FlightEvent> events = new ArrayList<>();
		events.add(new FlightEvent(FlightEvent.Type.LIFTOFF, log.liftoff() + shift));
		events.add(new FlightEvent(FlightEvent.Type.APOGEE, log.apogee() + shift));
		List<FlightReplayData.Event> simulated = simulation.getDeployments();
		for (int d = 0; d < simulated.size(); d++) {
			RocketComponent device = simulated.get(d).source();
			double at = d < log.deployments().size()
					? log.deployments().get(d) + shift
					: inferredDeployment(log, simulation, simulated.get(d).time(), shift);
			if (Double.isFinite(at)) {
				events.add(new FlightEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT, at, device));
			}
		}
		if (!Double.isNaN(log.landing())) {
			events.add(new FlightEvent(FlightEvent.Type.GROUND_HIT, log.landing() + shift));
		}
		events.sort((a, b) -> Double.compare(a.getTime(), b.getTime()));
		for (FlightEvent event : events) {
			branch.addEvent(event);
		}
		return FlightReplayData.fromBranch(branch);
	}

	/** Seconds to add to the log's clock to put its liftoff on the simulation's. */
	static double shift(FlightLog log, FlightReplayData simulation) {
		double simLiftoff = Double.isNaN(simulation.getLiftoffTime()) ? simulation.getStartTime()
				: simulation.getLiftoffTime();
		return simLiftoff - log.liftoff();
	}

	/** When the log's flight reaches the point where the simulation deployed, on the shifted clock. */
	private static double inferredDeployment(FlightLog log, FlightReplayData simulation, double simTime,
			double shift) {
		if (!Double.isNaN(simulation.getApogeeTime()) && Math.abs(simTime - simulation.getApogeeTime()) < AT_APOGEE) {
			return log.apogee() + shift;
		}
		double height = simulation.getValue(FlightDataType.TYPE_ALTITUDE, simTime);
		double[] time = log.time();
		double[] altitude = log.altitude();
		for (int i = 0; i < time.length; i++) {
			if (time[i] > log.apogee() && altitude[i] <= height) {
				return time[i] + shift;
			}
		}
		return Double.NaN;
	}

	/** Mean of the readings within {@link #STEADY_SECONDS} of each one. */
	private static double[] steady(double[] time, double[] values) {
		double[] out = new double[values.length];
		int from = 0;
		int to = 0;
		double sum = 0;
		for (int i = 0; i < values.length; i++) {
			while (to < values.length && time[to] <= time[i] + STEADY_SECONDS) {
				sum += values[to++];
			}
			while (time[from] < time[i] - STEADY_SECONDS) {
				sum -= values[from++];
			}
			out[i] = sum / (to - from);
		}
		return out;
	}

	private static double rollRate(FlightLog log, int i, FlightReplayData simulation, double t) {
		if (log.roll() == null) {
			return simulation.getValue(FlightDataType.TYPE_ROLL_RATE, t);
		}
		double[] time = log.time();
		int a = Math.max(0, i - 1), b = Math.min(time.length - 1, i + 1);
		return b > a ? Math.toRadians(log.roll()[b] - log.roll()[a]) / (time[b] - time[a]) : 0;
	}

	/** The GPS reading on the pad: the last one before liftoff, or the first there is. */
	private static double padValue(FlightLog log, double[] values) {
		double pad = values[0];
		for (int i = 0; i < values.length && log.time()[i] <= log.liftoff(); i++) {
			pad = values[i];
		}
		return pad;
	}
}
