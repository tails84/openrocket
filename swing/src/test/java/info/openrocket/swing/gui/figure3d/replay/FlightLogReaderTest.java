package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.rocketcomponent.Parachute;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.swing.util.BaseTestCase;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Locale;
import java.util.function.DoubleFunction;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Altimeter logs in each maker's layout, written from one made-up flight with known heights, and
 * the measured flight built from them beside a simulation.
 */
class FlightLogReaderTest extends BaseTestCase {
	private static final double FOOT = 0.3048;

	// The made-up flight: two seconds of boost at 50 m/s², a coast to apogee, a drogue descent at
	// 20 m/s, the main at 150 m and 6 m/s to the ground.
	private static final double BURNOUT = 2;
	private static final double APOGEE = BURNOUT + 100 / 9.81;
	private static final double APOGEE_HEIGHT = 100 + 100 * 100 / (2 * 9.81);
	private static final double MAIN_HEIGHT = 150;
	private static final double MAIN = APOGEE + (APOGEE_HEIGHT - MAIN_HEIGHT) / 20;
	private static final double LANDING = MAIN + MAIN_HEIGHT / 6;

	private static double height(double t) {
		if (t <= 0) {
			return 0;
		}
		if (t <= BURNOUT) {
			return 25 * t * t;
		}
		if (t <= APOGEE) {
			double c = t - BURNOUT;
			return 100 + 100 * c - 9.81 / 2 * c * c;
		}
		if (t <= MAIN) {
			return APOGEE_HEIGHT - 20 * (t - APOGEE);
		}
		return Math.max(0, MAIN_HEIGHT - 6 * (t - MAIN));
	}

	private static double speed(double t) {
		if (t <= 0) {
			return 0;
		}
		if (t <= BURNOUT) {
			return 50 * t;
		}
		if (t <= APOGEE) {
			return 100 - 9.81 * (t - BURNOUT);
		}
		if (t <= MAIN) {
			return -20;
		}
		return t <= LANDING ? -6 : 0;
	}

	/** Pressure in the standard atmosphere at a height above a pad at 100 m, hPa. */
	private static double pressure(double t) {
		return 1013.25 * Math.pow(1 - (100 + height(t)) / 44330.77, 1 / 0.190263);
	}

	private static String rows(String header, double from, double to, DoubleFunction<String> row) {
		StringBuilder text = new StringBuilder(header).append('\n');
		for (int i = 0; from + i * 0.05 <= to; i++) {
			text.append(row.apply(from + i * 0.05)).append('\n');
		}
		return text.toString();
	}

	private static String f(String format, Object... values) {
		return String.format(Locale.ROOT, format, values);
	}

	private static int flag(boolean on) {
		return on ? 1 : 0;
	}

	private static void assertFlight(FlightLog log) {
		assertEquals(APOGEE_HEIGHT, log.apogeeHeight(), 1.0, "apogee height");
		assertEquals(APOGEE, log.apogee(), 0.15, "apogee time");
		assertEquals(0, log.liftoff(), 0.15, "liftoff");
	}

	// ---------- layouts ----------

	@Test
	void readsAltusMetrumWithItsFlightStates() throws Exception {
		String text = rows("#version,serial,flight,call,time,state,state_name,acceleration,pressure,altitude,height,speed",
				-2, LANDING + 4, t -> {
					String state = t < 0 ? "pad" : t < BURNOUT ? "boost" : t < APOGEE ? "coast"
							: t < MAIN ? "drogue" : t < LANDING ? "main" : "landed";
					return f("6, 2098, 13,KD9TQD, %8.2f, 3, %8s, 0.00, %9.2f, %8.2f, %8.2f, %7.2f", t, state,
							pressure(t) * 100, 100 + height(t), height(t), speed(t));
				});
		FlightLog log = FlightLogReader.read("altos.csv", text, Double.NaN);

		assertEquals("Altus Metrum (AltosUI)", log.format());
		assertFlight(log);
		assertEquals(2, log.deployments().size());
		assertEquals(APOGEE, log.deployments().get(0), 0.06);
		assertEquals(MAIN, log.deployments().get(1), 0.06);
		assertEquals(LANDING, log.landing(), 0.06);
		assertEquals(100, max(log.velocity()), 1.0);
	}

	@Test
	void readsBlueRavenInFeetWithTiltAndFiredChannels() throws Exception {
		String text = rows("Year,Month,Day,Time,Flight_Time_(s),Sync,Baro_Altitude_AGL_(feet),Velocity_Up,"
				+ "Tilt_Angle_(deg),Roll_Angle_(deg),Liftoff,Apogee,Apo_fired,Main_fired", -1.9, LANDING + 2,
				t -> f("2026,6,17,09:25:12.187,%.2f,187,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d", t, height(t) / FOOT,
						speed(t) / FOOT, t > 0 ? 5.0 : 0.0, t > 0 ? 90 * t : 0.0, flag(t >= 0), flag(t >= APOGEE),
						flag(t >= APOGEE + 1), flag(t >= MAIN)));
		FlightLog log = FlightLogReader.read("blue_raven.csv", text, Double.NaN);

		assertEquals("Featherweight Blue Raven", log.format());
		assertFlight(log);
		assertEquals(List.of(APOGEE + 1, MAIN), roundedTo(log.deployments(), 0.05, APOGEE + 1, MAIN));
		assertEquals(100, max(log.velocity()), 1.0, "speed column read in feet per second");
		assertNotNull(log.tilt());
		assertEquals(5, log.tilt()[log.tilt().length / 2], 1e-9);
		assertNotNull(log.roll());
	}

	@Test
	void readsEggtimerMillisecondsAndItsMarkerColumns() throws Exception {
		String text = rows("T,Alt,Veloc,FAlt,FVeloc,LDA,LowV,Apogee,N-O,Drogue,Main", 0, LANDING + 2,
				t -> f("%.0f,%.0f,%.2f,%.2f,%.2f,%d,0,%d,0,%d,%d", t * 1000, height(t) / FOOT, speed(t) / FOOT,
						height(t) / FOOT, speed(t) / FOOT, flag(t > 0.3), flag(Math.abs(t - APOGEE) < 0.03),
						flag(t >= APOGEE && t < APOGEE + 1), flag(t >= MAIN && t < MAIN + 1)));
		FlightLog log = FlightLogReader.read("eggtimer.csv", text, Double.NaN);

		assertEquals("Eggtimer", log.format());
		assertEquals(LANDING + 2, log.time()[log.time().length - 1], 0.06, "clock read in milliseconds");
		assertFlight(log);
		assertEquals(2, log.deployments().size());
		assertEquals(APOGEE, log.deployments().get(0), 0.06);
		assertEquals(MAIN, log.deployments().get(1), 0.06);
	}

	@Test
	void readsEggtimerSeconds() throws Exception {
		String text = rows("T,Alt,Veloc,FAlt,FVeloc", 0, LANDING + 2,
				t -> f("%.2f,%.0f,%.2f,%.2f,%.2f", t, height(t) / FOOT, speed(t) / FOOT, height(t) / FOOT,
						speed(t) / FOOT));
		FlightLog log = FlightLogReader.read("eggtimer.csv", text, Double.NaN);

		assertEquals(LANDING + 2, log.time()[log.time().length - 1], 0.06);
		assertFlight(log);
		assertTrue(log.deployments().isEmpty());
	}

	@Test
	void settlesTheRrc3UnitFromThePressure() throws Exception {
		for (double unit : new double[] { 1, FOOT }) {
			String text = rows("time,altitude,pressure,velocity,temperature,events,voltages", 0, LANDING + 2,
					t -> f("%.2f,%.4f,%.2f,%.4f,31.9, -,3.81", t, height(t) / unit, pressure(t), speed(t) / unit));
			FlightLog log = FlightLogReader.read("rrc3.csv", text, Double.NaN);

			assertEquals("Missile Works RRC3 (mDACS)", log.format());
			assertFlight(log);
			assertEquals(100, max(log.velocity()), 1.0);
		}
	}

	@Test
	void findsAnRrc3EventOnARowWithoutItsTemperature() throws Exception {
		String text = rows("time,altitude,pressure,velocity,temperature,events,voltages", 0, LANDING + 2,
				t -> Math.abs(t - APOGEE) < 0.025
						? f("%.2f,%.4f,%.2f,%.4f, Drogue,,", t, height(t), pressure(t), speed(t))
						: f("%.2f,%.4f,%.2f,%.4f,31.9, -,3.81", t, height(t), pressure(t), speed(t)));
		FlightLog log = FlightLogReader.read("rrc3.csv", text, Double.NaN);

		assertEquals(1, log.deployments().size());
		assertEquals(APOGEE, log.deployments().get(0), 0.03);
	}

	@Test
	void readsPerfectFliteInFeet() throws Exception {
		String text = rows("Time,Altitude,Velocity,Temperature (F),Voltage", 0, LANDING + 2,
				t -> f("%.2f,%.0f,%.0f,70.00,9.00", t, height(t) / FOOT, speed(t) / FOOT));
		FlightLog log = FlightLogReader.read("perfectflite.csv", text, Double.NaN);

		assertEquals("PerfectFlite", log.format());
		assertFlight(log);
	}

	@Test
	void readsAnyTableByItsColumnNamesAndUnits() throws Exception {
		String text = rows("sep=;\n\"Time (s)\";\"Altitude (ft)\";\"Velocity (ft/s)\"", -1, LANDING + 2,
				t -> f("%.2f;%.1f;%.1f", t, height(t) / FOOT, speed(t) / FOOT).replace('.', ','));
		FlightLog log = FlightLogReader.read("export.csv", text, Double.NaN);

		assertEquals("a table read by its column names", log.format());
		assertFlight(log);
		assertEquals(100, max(log.velocity()), 1.0);
	}

	@Test
	void picksTheUnstatedUnitCloserToTheSimulatedApogee() throws Exception {
		String text = rows("time,height", -1, LANDING + 2, t -> f("%.2f,%.1f", t, height(t) / FOOT));

		assertEquals(APOGEE_HEIGHT, FlightLogReader.read("a.csv", text, 600).apogeeHeight(), 1.0);
		assertEquals(APOGEE_HEIGHT / FOOT, FlightLogReader.read("a.csv", text, 2000).apogeeHeight(), 3.0);
	}

	@Test
	void ignoresReadingsTheRocketCouldNotReach() throws Exception {
		// One corrupt row, and a second of a disturbed pressure port near apogee.
		String text = rows("T,Alt,Veloc,FAlt,FVeloc", 0, LANDING + 2, t -> {
			double feet = height(t) / FOOT;
			if (Math.abs(t - 5) < 0.01) {
				feet = 99_999;
			} else if (t > APOGEE - 1.5 && t < APOGEE - 0.5) {
				feet += Math.round(t * 20) % 2 == 0 ? 3000 : -2500;
			}
			return f("%.2f,%.0f,0,%.2f,0", t, feet, feet);
		});
		FlightLog log = FlightLogReader.read("spiky.csv", text, Double.NaN);

		assertFlight(log);
		assertTrue(log.notes().stream().anyMatch(n -> n.startsWith("Ignored ")));
		assertTrue(max(log.velocity()) < 110, "speed worked out from the cleaned heights");
	}

	@Test
	void rejectsALogWithoutAFlight() {
		String flat = rows("time,altitude", 0, 20, t -> f("%.2f,%.2f", t, Math.sin(t)));
		assertThrows(FlightLogReader.LogException.class, () -> FlightLogReader.read("pad.csv", flat, Double.NaN));
		assertThrows(FlightLogReader.LogException.class,
				() -> FlightLogReader.read("short.csv", "time,altitude\n0,0\n1,100\n", Double.NaN));
		assertThrows(FlightLogReader.LogException.class,
				() -> FlightLogReader.read("other.csv", "name,value\na,1\n", Double.NaN));
	}

	// ---------- the measured flight beside a simulation ----------

	private static final double SIM_LIFTOFF = 0.5;
	private static final Parachute DROGUE = new Parachute();
	private static final Parachute MAIN_CHUTE = new Parachute();

	/** A simulation of the same flight, a little higher, lifting off half a second into its clock. */
	private static FlightReplayData simulation() {
		FlightDataBranch branch = new FlightDataBranch("Sustainer",
				FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE, FlightDataType.TYPE_POSITION_X,
				FlightDataType.TYPE_POSITION_Y, FlightDataType.TYPE_VELOCITY_Z, FlightDataType.TYPE_ORIENTATION_THETA,
				FlightDataType.TYPE_ORIENTATION_PHI, FlightDataType.TYPE_ROLL_RATE, FlightDataType.TYPE_CG_LOCATION);
		for (int i = 0; i * 0.05 <= LANDING + 1; i++) {
			double t = i * 0.05;
			branch.addPoint();
			branch.setValue(FlightDataType.TYPE_TIME, t);
			branch.setValue(FlightDataType.TYPE_ALTITUDE, 1.1 * height(t - SIM_LIFTOFF));
			branch.setValue(FlightDataType.TYPE_POSITION_X, 0.5 * t);
			branch.setValue(FlightDataType.TYPE_POSITION_Y, 0.0);
			branch.setValue(FlightDataType.TYPE_VELOCITY_Z, 1.1 * speed(t - SIM_LIFTOFF));
			branch.setValue(FlightDataType.TYPE_ORIENTATION_THETA, Math.PI / 2);
			branch.setValue(FlightDataType.TYPE_ORIENTATION_PHI, 0.0);
			branch.setValue(FlightDataType.TYPE_ROLL_RATE, 0.0);
			branch.setValue(FlightDataType.TYPE_CG_LOCATION, 0.3);
		}
		branch.addEvent(new FlightEvent(FlightEvent.Type.LIFTOFF, SIM_LIFTOFF));
		branch.addEvent(new FlightEvent(FlightEvent.Type.APOGEE, SIM_LIFTOFF + APOGEE));
		branch.addEvent(new FlightEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT, SIM_LIFTOFF + APOGEE, DROGUE));
		branch.addEvent(new FlightEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT, SIM_LIFTOFF + MAIN, MAIN_CHUTE));
		branch.addEvent(new FlightEvent(FlightEvent.Type.GROUND_HIT, SIM_LIFTOFF + LANDING));
		return FlightReplayData.fromBranch(branch);
	}

	@Test
	void measuredFlightLiftsOffWithTheSimulationAndKeepsItsOwnDeployments() throws Exception {
		// Logged deployments a little late, as a delayed charge would fire.
		String text = rows("Year,Month,Day,Time,Flight_Time_(s),Sync,Baro_Altitude_AGL_(feet),Velocity_Up,"
				+ "Tilt_Angle_(deg),Roll_Angle_(deg),Liftoff,Apogee,Apo_fired,Main_fired", -1.9, LANDING + 2,
				t -> f("2026,6,17,09:25:12.187,%.2f,187,%.1f,%.1f,0,0,%d,0,%d,%d", t, height(t) / FOOT,
						speed(t) / FOOT, flag(t >= 0), flag(t >= APOGEE + 1), flag(t >= MAIN + 2)));
		FlightLog log = FlightLogReader.read("blue_raven.csv", text, Double.NaN);
		FlightReplayData simulation = simulation();
		FlightReplayData measured = MeasuredFlight.build(log, simulation, "Measured");

		assertEquals(SIM_LIFTOFF, MeasuredFlight.shift(log, simulation), 1e-9);
		assertEquals(SIM_LIFTOFF, measured.getLiftoffTime(), 1e-9);
		// The shown height is steadied over half a second, which takes a little off the sharp peak.
		assertEquals(APOGEE_HEIGHT, measured.getValue(FlightDataType.TYPE_ALTITUDE, SIM_LIFTOFF + APOGEE), 2.0);
		List<FlightReplayData.Event> deployments = measured.getDeployments();
		assertEquals(2, deployments.size());
		assertSame(DROGUE, deployments.get(0).source());
		assertSame(MAIN_CHUTE, deployments.get(1).source());
		assertEquals(SIM_LIFTOFF + APOGEE + 1, deployments.get(0).time(), 0.06);
		assertEquals(SIM_LIFTOFF + MAIN + 2, deployments.get(1).time(), 0.06);
		// Without GPS the track follows the simulation's.
		assertEquals(5.0, measured.getValue(FlightDataType.TYPE_POSITION_X, 10), 1e-6);
	}

	@Test
	void measuredFlightWithoutDeploymentsGetsThemWhereTheSimulationHasThem() throws Exception {
		String text = rows("Time,Altitude,Velocity,Temperature (F),Voltage", 0, LANDING + 2,
				t -> f("%.2f,%.0f,%.0f,70.00,9.00", t, height(t) / FOOT, speed(t) / FOOT));
		FlightLog log = FlightLogReader.read("perfectflite.csv", text, Double.NaN);
		FlightReplayData simulation = simulation();
		FlightReplayData measured = MeasuredFlight.build(log, simulation, "Measured");
		double shift = MeasuredFlight.shift(log, simulation);

		List<FlightReplayData.Event> deployments = measured.getDeployments();
		assertEquals(2, deployments.size());
		// The drogue at the log's own apogee; the main where the log falls to the simulated main's
		// height, which is 10% higher than this flight's.
		assertEquals(log.apogee() + shift, deployments.get(0).time(), 1e-9);
		double mainHeight = 1.1 * MAIN_HEIGHT;
		double expected = APOGEE + (APOGEE_HEIGHT - mainHeight) / 20;
		assertEquals(expected + shift, deployments.get(1).time(), 0.1);
		assertNull(log.tilt());
	}

	// ---------- helpers ----------

	private static double max(double[] values) {
		double max = Double.NEGATIVE_INFINITY;
		for (double v : values) {
			max = Math.max(max, v);
		}
		return max;
	}

	/** Each value replaced by the expected one it lies within the tolerance of, to compare lists. */
	private static List<Double> roundedTo(List<Double> values, double tolerance, double... expected) {
		return values.stream().map(v -> {
			for (double e : expected) {
				if (Math.abs(v - e) <= tolerance) {
					return e;
				}
			}
			return v;
		}).toList();
	}
}
