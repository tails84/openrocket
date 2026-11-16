package info.openrocket.swing.gui.figure3d.animation;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.util.TestRockets;
import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import info.openrocket.swing.util.BaseTestCase;
import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AnimationTest extends BaseTestCase {
	private static final float EPSILON = 1.0e-4f;

	@Test
	void convertsSimulationAxesAndWorldScale() {
		Vector3f position = SimToWorld.toEngine(2.0f, 3.0f, 4.0f);

		assertEquals(2.0f * RenderingConstants.WORLD_SCALE, position.x, EPSILON);
		assertEquals(4.0f * RenderingConstants.WORLD_SCALE, position.y, EPSILON);
		assertEquals(-3.0f * RenderingConstants.WORLD_SCALE, position.z, EPSILON);
	}

	@Test
	void playbackClockAppliesRateAndClampsAtBothEnds() {
		PlaybackClock clock = new PlaybackClock(2.0, 5.0);
		clock.setRate(2.0);
		clock.update(2.0);
		assertEquals(5.0, clock.getTime());

		clock.setRate(-3.0);
		clock.update(2.0);
		assertEquals(2.0, clock.getTime());

		clock.setTime(4.0);
		assertEquals(4.0, clock.getTime());
	}

	@Test
	void flightPoseInterpolatesPositionAndExplicitOrientation() {
		FlightDataBranch branch = new FlightDataBranch("flight",
				FlightDataType.TYPE_TIME,
				FlightDataType.TYPE_POSITION_X,
				FlightDataType.TYPE_POSITION_Y,
				FlightDataType.TYPE_ALTITUDE,
				FlightDataType.TYPE_ORIENTATION_THETA,
				FlightDataType.TYPE_ORIENTATION_PHI);
		// Theta is elevation above the horizon and phi a bearing clockwise from north.
		addPoint(branch, 0.0, 0.0, 0.0, 10.0, Math.PI / 2.0, 0.0);
		addPoint(branch, 1.0, 2.0, 4.0, 20.0, 0.0, Math.PI / 2.0);
		addPoint(branch, 2.0, 2.0, 4.0, 20.0, 0.0, 0.0);

		FlightPoseProvider provider = FlightPoseProvider.fromFlightDataBranch(branch);
		Vector3f midpoint = provider.getPosition(0.5);
		assertEquals(20.0f, midpoint.x, EPSILON);
		assertEquals(300.0f, midpoint.y, EPSILON);
		assertEquals(-40.0f, midpoint.z, EPSILON);

		assertDirection(0.0f, 1.0f, 0.0f, provider.getOrientation(0.0));	// straight up
		assertDirection(1.0f, 0.0f, 0.0f, provider.getOrientation(1.0));	// level, heading east
		assertDirection(0.0f, 0.0f, -1.0f, provider.getOrientation(2.0));	// level, heading north
	}

	@Test
	void flightPoseInterpolatesHeadingThroughNorthWithoutSwingingSouth() {
		FlightDataBranch branch = new FlightDataBranch("flight",
				FlightDataType.TYPE_TIME,
				FlightDataType.TYPE_POSITION_X,
				FlightDataType.TYPE_POSITION_Y,
				FlightDataType.TYPE_ALTITUDE,
				FlightDataType.TYPE_ORIENTATION_THETA,
				FlightDataType.TYPE_ORIENTATION_PHI);
		addPoint(branch, 0.0, 0.0, 0.0, 0.0, 0.0, Math.toRadians(350.0));
		addPoint(branch, 1.0, 0.0, 0.0, 0.0, 0.0, Math.toRadians(10.0));

		Vector3f direction = FlightPoseProvider.fromFlightDataBranch(branch).getAxisDirection(0.5);
		assertEquals(0.0f, direction.x, EPSILON);
		assertEquals(-1.0f, direction.z, EPSILON);
	}

	@Test
	void flightPoseSupportsPolarLateralPositionFallback() {
		FlightDataBranch branch = new FlightDataBranch("flight",
				FlightDataType.TYPE_TIME,
				FlightDataType.TYPE_POSITION_XY,
				FlightDataType.TYPE_POSITION_DIRECTION,
				FlightDataType.TYPE_ALTITUDE);
		branch.addPoint();
		branch.setValue(FlightDataType.TYPE_TIME, 0.0);
		branch.setValue(FlightDataType.TYPE_POSITION_XY, 2.0);
		// Bearing clockwise from north, so pi/2 is due east.
		branch.setValue(FlightDataType.TYPE_POSITION_DIRECTION, Math.PI / 2.0);
		branch.setValue(FlightDataType.TYPE_ALTITUDE, 3.0);

		Vector3f position = FlightPoseProvider.fromFlightDataBranch(branch).getPosition(0.0);
		assertEquals(40.0f, position.x, EPSILON);
		assertEquals(60.0f, position.y, EPSILON);
		assertEquals(0.0f, position.z, EPSILON);
	}

	@Test
	void flightPoseRejectsMissingLateralPosition() {
		FlightDataBranch branch = new FlightDataBranch("flight",
				FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE);
		branch.addPoint();
		branch.setValue(FlightDataType.TYPE_TIME, 0.0);
		branch.setValue(FlightDataType.TYPE_ALTITUDE, 3.0);

		assertThrows(IllegalArgumentException.class,
				() -> FlightPoseProvider.fromFlightDataBranch(branch));
	}

	@Test
	void flightPoseOfSimulatedLaunchFollowsTheLaunchRod() throws Exception {
		Rocket rocket = TestRockets.makeEstesAlphaIII();
		Simulation simulation = new Simulation(rocket);
		simulation.setFlightConfigurationId(TestRockets.TEST_FCID_0);
		simulation.getOptions().setISAAtmosphere(true);
		simulation.getOptions().setWindSpeedAverage(0.0);
		simulation.getOptions().setWindTurbulenceIntensity(0.0);
		simulation.getOptions().setLaunchRodAngle(Math.toRadians(10.0));
		simulation.getOptions().setLaunchRodDirection(Math.PI / 2.0); // tilted towards east
		simulation.simulate();

		FlightDataBranch branch = simulation.getSimulatedData().getBranch(0);
		FlightPoseProvider provider = FlightPoseProvider.fromFlightDataBranch(branch);

		Vector3f direction = provider.getAxisDirection(provider.getStartTime());
		assertEquals((float) Math.sin(Math.toRadians(10.0)), direction.x, 1.0e-3f);
		assertEquals((float) Math.cos(Math.toRadians(10.0)), direction.y, 1.0e-3f);
		assertEquals(0.0f, direction.z, 1.0e-3f);
	}

	private static void assertDirection(float x, float y, float z,
			org.joml.Quaternionf orientation) {
		Vector3f direction = orientation.transform(new Vector3f(0.0f, 1.0f, 0.0f));
		assertEquals(x, direction.x, EPSILON);
		assertEquals(y, direction.y, EPSILON);
		assertEquals(z, direction.z, EPSILON);
	}

	private static void addPoint(FlightDataBranch branch, double time, double east, double north,
			double altitude, double theta, double phi) {
		branch.addPoint();
		branch.setValue(FlightDataType.TYPE_TIME, time);
		branch.setValue(FlightDataType.TYPE_POSITION_X, east);
		branch.setValue(FlightDataType.TYPE_POSITION_Y, north);
		branch.setValue(FlightDataType.TYPE_ALTITUDE, altitude);
		branch.setValue(FlightDataType.TYPE_ORIENTATION_THETA, theta);
		branch.setValue(FlightDataType.TYPE_ORIENTATION_PHI, phi);
	}
}
