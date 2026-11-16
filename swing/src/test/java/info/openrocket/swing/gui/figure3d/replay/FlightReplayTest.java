package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.BodyTube;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.NoseCone;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightData;
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

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Replay data and poses for simulated flights: a single-stage Estes Alpha III and a staged rocket. */
class FlightReplayTest extends BaseTestCase {
	private static final float S = RenderingConstants.WORLD_SCALE;

	private static Rocket rocket;
	private static FlightDataBranch branch;
	private static FlightReplayData data;
	private static Rocket stagedRocket;
	private static List<FlightReplayData> stagedBranches;

	@BeforeAll
	static void simulateFlights() throws Exception {
		rocket = TestRockets.makeEstesAlphaIII();
		Simulation simulation = calmSimulation(rocket);
		simulation.setFlightConfigurationId(TestRockets.TEST_FCID_0);
		simulation.simulate();
		rocket.setSelectedConfiguration(TestRockets.TEST_FCID_0);
		branch = simulation.getSimulatedData().getBranch(0);
		data = FlightReplayData.fromBranch(branch);

		stagedRocket = TestRockets.makeMultiStageEventTestRocket();
		Simulation staged = calmSimulation(stagedRocket);
		staged.setFlightConfigurationId(stagedRocket.getSelectedConfiguration().getFlightConfigurationID());
		staged.simulate();
		stagedBranches = new ArrayList<>();
		FlightData flight = staged.getSimulatedData();
		for (int i = 0; i < flight.getBranchCount(); i++) {
			stagedBranches.add(FlightReplayData.fromBranch(flight.getBranch(i)));
		}
	}

	private static Simulation calmSimulation(Rocket rocket) {
		Simulation simulation = new Simulation(rocket);
		simulation.getOptions().setISAAtmosphere(true);
		simulation.getOptions().setWindSpeedAverage(0.0);
		simulation.getOptions().setWindTurbulenceIntensity(0.0);
		return simulation;
	}

	// ---------- flight data ----------

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

	// ---------- single stage ----------

	@Test
	void rocketStandsOnThePadAndNosePointsUpTheRail() {
		FlightReplayScene scene = new FlightReplayScene(List.of(data), rocket, 1.0, -27.0);
		double start = data.getStartTime();
		Vector3f axis = data.getPose().getAxisDirection(start);
		double cg = data.getValue(FlightDataType.TYPE_CG_LOCATION, start);
		NoseCone nose = first(rocket, NoseCone.class);

		// The nose tip sits on the flight axis, the CG distance ahead of the pivot.
		Vector3f tip = sitePoint(scene, nose, start, 0).sub(scene.getBodies().get(0).getSitePivot());
		Vector3f expected = new Vector3f(axis).mul((float) (cg * S));
		assertEquals(expected.x, tip.x, 1e-3f);
		assertEquals(expected.y, tip.y, 1e-3f);
		assertEquals(expected.z, tip.z, 1e-3f);

		// The aft end rests just above the ground, which is at zero height in the site frame.
		FinSet fins = first(rocket, FinSet.class);
		double length = rocket.getSelectedConfiguration().getBoundingBox().max.getX();
		float aftHeight = sitePoint(scene, fins, start, length).y / S;
		assertTrue(aftHeight >= 0 && aftHeight < 0.05, "aft end height " + aftHeight);
	}

	@Test
	void parachuteOpensTheAirframeAtTheTubeItIsPackedIn() {
		FlightReplayScene scene = new FlightReplayScene(List.of(data), rocket, 1.0, -27.0);
		NoseCone nose = first(rocket, NoseCone.class);
		BodyTube tube = first(rocket, BodyTube.class);
		FinSet fins = first(rocket, FinSet.class);

		assertEquals(2, scene.getBodies().get(0).getSectionCount());
		assertEquals(0, scene.partOf(nose).section());
		assertEquals(1, scene.partOf(tube).section());
		assertEquals(1, scene.partOf(fins).section());

		// Together until the parachute deploys, then on a shock cord apart.
		double joint = nose.getLength();
		double deployment = data.getFirstDeploymentTime();
		float closed = sitePoint(scene, nose, deployment - 0.1, joint)
				.distance(sitePoint(scene, tube, deployment - 0.1, joint));
		float open = sitePoint(scene, nose, deployment + 3, joint)
				.distance(sitePoint(scene, tube, deployment + 3, joint));
		assertEquals(0, closed / S, 1e-3);
		assertTrue(open / S > 0.1, "shoulder gap " + open / S);
	}

	@Test
	void piecesRestOnTheGroundAfterLanding() {
		FlightReplayScene scene = new FlightReplayScene(List.of(data), rocket, 1.0, -27.0);
		double landed = data.getGroundHitTime() + 1;
		double radius = first(rocket, BodyTube.class).getOuterRadius();
		for (RocketComponent component : List.of(first(rocket, NoseCone.class), first(rocket, BodyTube.class))) {
			double front = component.getComponentLocations()[0].getX();
			for (double x : new double[] { front, front + component.getLength() }) {
				float height = sitePoint(scene, component, landed, x).y / S;
				assertTrue(height > -1e-3 && height < 3 * radius, component.getName() + " at height " + height);
			}
		}
	}

	// ---------- staging ----------

	@Test
	void separatedStagesStartWhereTheyLeftTheStack() {
		assertTrue(stagedBranches.size() > 1, "the staged rocket separates");
		FlightReplayScene scene = new FlightReplayScene(stagedBranches, stagedRocket, 1.0, -27.0);
		for (int b = 1; b < scene.getBodies().size(); b++) {
			double separation = scene.getBodies().get(b).getSeparationTime();
			RocketComponent part = componentOf(scene, b);
			double x = part.getComponentLocations()[0].getX();

			Vector3f attached = sitePoint(scene, part, separation - 1e-4, x);
			Vector3f released = sitePoint(scene, part, separation + 1e-4, x);
			assertTrue(attached.distance(released) / S < 0.02,
					part.getName() + " jumped " + attached.distance(released) / S + " m at separation");
		}
	}

	@Test
	void separatedStagesFlyTheirOwnBranch() {
		FlightReplayScene scene = new FlightReplayScene(stagedBranches, stagedRocket, 1.0, -27.0);
		for (int b = 1; b < scene.getBodies().size(); b++) {
			ReplayBody body = scene.getBodies().get(b);
			double t = body.getData().getEndTime();
			RocketComponent part = componentOf(scene, b);
			Vector3f position = sitePoint(scene, part, t, part.getComponentLocations()[0].getX());
			Vector3f landing = body.getData().getPose().getPosition(t);
			float horizontalMiss = new Vector3f(position.x - landing.x, 0, position.z - landing.z).length() / S;
			assertTrue(horizontalMiss < 1.0, part.getName() + " lands " + horizontalMiss + " m from its branch");
		}
	}

	// ---------- helpers ----------

	/** Position of a point on a component's axis in the launch-site frame, engine units. */
	private static Vector3f sitePoint(FlightReplayScene scene, RocketComponent component, double t, double x) {
		for (ReplayBody body : scene.getBodies()) {
			body.update(t);
		}
		Matrix4f transform = scene.partTransform(scene.partOf(component), t, new Vector3f());
		return transform.transformPosition(new Vector3f((float) (x * S), 0, 0));
	}

	/** A body tube of the staged rocket that flies with the given body. */
	private static RocketComponent componentOf(FlightReplayScene scene, int body) {
		for (RocketComponent component : stagedRocket) {
			if (component instanceof BodyTube && scene.ownerOf(component) == body) {
				return component;
			}
		}
		throw new AssertionError("no body tube in body " + body);
	}

	private static <T> T first(Rocket rocket, Class<T> type) {
		for (RocketComponent component : rocket) {
			if (type.isInstance(component)) {
				return type.cast(component);
			}
		}
		throw new AssertionError("no " + type.getSimpleName());
	}
}
