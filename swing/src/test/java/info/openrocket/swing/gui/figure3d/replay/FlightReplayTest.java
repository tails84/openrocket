package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import info.openrocket.swing.util.BaseTestCase;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Replay data and poses for a simulated Estes Alpha III flight. */
class FlightReplayTest extends BaseTestCase {
	private static Rocket rocket;
	private static FlightDataBranch branch;
	private static FlightReplayData data;

	@BeforeAll
	static void simulateFlight() throws Exception {
		rocket = TestRockets.makeEstesAlphaIII();
		Simulation simulation = new Simulation(rocket);
		simulation.setFlightConfigurationId(TestRockets.TEST_FCID_0);
		simulation.getOptions().setISAAtmosphere(true);
		simulation.getOptions().setWindSpeedAverage(0.0);
		simulation.getOptions().setWindTurbulenceIntensity(0.0);
		simulation.simulate();
		rocket.setSelectedConfiguration(TestRockets.TEST_FCID_0);
		branch = simulation.getSimulatedData().getBranch(0);
		data = FlightReplayData.fromBranch(branch);
	}

	@Test
	void phasesFollowTheSimulatedEvents() {
		double liftoff = branch.getFirstEvent(FlightEvent.Type.LIFTOFF).getTime();
		double burnout = branch.getFirstEvent(FlightEvent.Type.BURNOUT).getTime();
		double apogee = branch.getFirstEvent(FlightEvent.Type.APOGEE).getTime();
		double deployment = branch.getFirstEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT).getTime();
		double groundHit = branch.getFirstEvent(FlightEvent.Type.GROUND_HIT).getTime();

		assertEquals(FlightReplayData.Phase.POWERED, data.getPhase(0.5 * (liftoff + burnout)));
		assertTrue(data.isThrusting(0.5 * (liftoff + burnout)));
		assertFalse(data.isThrusting(0.5 * (burnout + apogee)));
		assertEquals(FlightReplayData.Phase.COASTING, data.getPhase(0.5 * (burnout + Math.min(apogee, deployment))));
		assertEquals(FlightReplayData.Phase.UNDER_RECOVERY, data.getPhase(0.5 * (deployment + groundHit)));
		assertEquals(FlightReplayData.Phase.LANDED, data.getPhase(groundHit));
	}

	@Test
	void marksTimelineEventsInOrderAndKeepsDeployedDevices() {
		List<FlightReplayData.Event> events = data.getEvents();
		assertEquals(FlightEvent.Type.LIFTOFF, events.get(0).type());
		assertEquals(FlightEvent.Type.GROUND_HIT, events.get(events.size() - 1).type());
		for (int i = 1; i < events.size(); i++) {
			assertTrue(events.get(i).time() >= events.get(i - 1).time());
		}
		assertTrue(events.stream().noneMatch(e -> e.type() == FlightEvent.Type.SIMULATION_END));

		assertFalse(data.getDeployments().isEmpty());
		assertInstanceOf(RecoveryDevice.class, data.getDeployments().get(0).source());
		assertEquals(data.getDeployments().get(0).time(), data.getFirstDeploymentTime());
	}

	@Test
	void interpolatesRecordedValues() {
		List<Double> time = branch.get(FlightDataType.TYPE_TIME);
		List<Double> altitude = branch.get(FlightDataType.TYPE_ALTITUDE);
		int i = time.size() / 3;
		double midpoint = 0.5 * (time.get(i) + time.get(i + 1));

		assertEquals(altitude.get(i), data.getValue(FlightDataType.TYPE_ALTITUDE, time.get(i)), 1e-9);
		assertEquals(0.5 * (altitude.get(i) + altitude.get(i + 1)),
				data.getValue(FlightDataType.TYPE_ALTITUDE, midpoint), 1e-9);
	}

	@Test
	void fillsGapsFromNeighbouringValues() {
		double nan = Double.NaN;
		assertArrayEquals(new double[] { 2, 2, 3, 3, 5 },
				FlightReplayData.fillGaps(new double[] { nan, 2, 3, nan, 5, 6 }, 5));
		assertNull(FlightReplayData.fillGaps(new double[] { nan, nan }, 2));
	}

	@Test
	void rocketStandsOnThePadAndNosePointsUpTheRail() {
		FlightReplayScene scene = new FlightReplayScene(data, rocket, 1.0, -27.0);
		double start = data.getStartTime();
		Vector3f axis = data.getPose().getAxisDirection(start);
		double cg = data.getValue(FlightDataType.TYPE_CG_LOCATION, start);
		Matrix4f transform = scene.rocketTransform(start);

		// The nose tip (rocket x = 0) sits on the flight axis, the CG distance ahead of the pivot.
		Vector3f nose = transform.transformPosition(new Vector3f());
		Vector3f expected = new Vector3f(axis).mul((float) (cg * RenderingConstants.WORLD_SCALE));
		assertEquals(expected.x, nose.x, 1e-3f);
		assertEquals(expected.y, nose.y, 1e-3f);
		assertEquals(expected.z, nose.z, 1e-3f);

		// The aft end rests just above the ground: the site origin is ground level.
		double length = rocket.getSelectedConfiguration().getBoundingBox().max.getX();
		Vector3f aft = transform.transformPosition(new Vector3f((float) (length * RenderingConstants.WORLD_SCALE), 0, 0));
		Vector3f cgSite = scene.cgSitePosition(start);
		float aftHeight = (aft.y + cgSite.y) / RenderingConstants.WORLD_SCALE;
		assertTrue(aftHeight >= 0 && aftHeight < 0.05, "aft end height " + aftHeight);
	}

	@Test
	void rocketHangsLevelOnTheGroundAfterLanding() {
		FlightReplayScene scene = new FlightReplayScene(data, rocket, 1.0, -27.0);
		Vector3f axis = scene.attitude(data.getGroundHitTime());
		assertEquals(0.0f, axis.y, 1e-3f);
	}
}
