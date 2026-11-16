package info.openrocket.swing.gui.figure3d.replay;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Layout of the launch site's ground, which needs no OpenGL. */
class LaunchSiteTest {
	private static final double HALF = 32_768;

	private static LaunchSite site(List<double[]> landings) {
		return new LaunchSite(new Vector3f(1, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0.4f, 0.77f, -0.5f), 1.5,
				new Vector3f(0, 1, 0), 1.5, 0.0254, 0.05, Math.PI / 2, 3, landings, null);
	}

	@Test
	void groundCoversEverythingOnceWithCloseUpGrassAroundThePads() {
		LaunchSite site = site(List.of());
		double[] areas = site.groundAreas();
		double[] mown = site.mownArea();
		double mownArea = (mown[1] - mown[0]) * (mown[3] - mown[2]);

		assertEquals(4 * HALF * HALF, areas[0] + areas[1] + areas[2] + areas[3] + areas[4], 1.0, "whole ground, no gaps or overlaps");
		assertEquals(mownArea, areas[1], 1e-6, "mown launch area");
		assertEquals(100 * 100 - mownArea, areas[0], 1e-6, "long grass round it, to the fence");
		assertEquals(2048 * 2048 - 100 * 100, areas[2], 1e-6, "the paddock image");
	}

	@Test
	void whereSomethingLandsGetsCloseUpGrassToo() {
		// One landing in the paddock, one out in the country beyond it.
		double[] areas = site(List.of(new double[] { 300, -200 }, new double[] { 2500, 900 })).groundAreas();
		double[] plain = site(List.of()).groundAreas();

		assertEquals(4 * HALF * HALF, areas[0] + areas[1] + areas[2] + areas[3] + areas[4], 1.0);
		assertEquals(plain[0] + 2 * 64 * 64, areas[0], 1e-6, "a 64 m square of long grass at each landing");
		assertEquals(plain[2] - 64 * 64, areas[2], 1e-6);
		assertEquals(plain[3] - 64 * 64, areas[3], 1.0);
	}

	// ---------- the surveyed site at Cedar Grove ----------

	@Test
	void knowsTheCedarGroveSiteAndNoOther() {
		assertNotNull(SiteSurvey.near(-27.85, 152.96), "QRS LA2, as the club's rocket files give it");
		assertNull(SiteSurvey.near(-27.0, 153.0));
		assertNull(SiteSurvey.near(-27.85, 153.1), "14 km east is another place");
	}

	@Test
	void surveyHasTheRoadSouthAndTheRiverNorthOfThePads() {
		SiteSurvey survey = SiteSurvey.near(-27.85, 152.96);
		assertEquals(0, survey.height(0, 0), 1.0, "heights are above the launch area");
		assertTrue(crosses(survey, 0, -200, 0, -350, SiteSurvey.ROAD), "Cedar Grove Road, about 270 m south");
		assertTrue(crosses(survey, -400, 600, -400, 1600, SiteSurvey.WATER), "the Logan River to the north");
		double[] hills = site(survey, -9200, 7100);
		assertTrue(survey.height(hills[0], hills[1]) > 150, "the hills towards Flinders Peak");
	}

	@Test
	void padsLineUpAlongTheTrackAndLaunchToTheWest() {
		SiteSurvey survey = SiteSurvey.near(-27.85, 152.96);
		Vector3f spectators = survey.spectatorDirection();
		assertTrue(spectators.x > 0.95f, "spectators to the east, rockets flying out to the west: " + spectators);
		assertEquals(0, survey.rowDirection().dot(spectators), 1e-6);

		// The dirt track runs behind the line of tents, alongside the pads.
		double[][] track = survey.track();
		for (int i = 0; i + 1 < track.length; i++) {
			double[] a = track[i], b = track[i + 1];
			if (Math.abs(a[0]) > 150 || Math.abs(b[0]) > 150) {
				continue;
			}
			double away = -0.5 * (a[1] + b[1]);
			assertTrue(away > LaunchSite.TENT_LINE + 3 && away < LaunchSite.TENT_LINE + 20, "track " + away + " m behind the pads");
			double skew = Math.toDegrees(Math.atan2(Math.abs(b[1] - a[1]), Math.abs(b[0] - a[0])));
			assertTrue(skew < 3, "track runs " + skew + " degrees off the line of pads");
		}
	}

	/** Site frame coordinates, as the survey takes them, of a point metres east and north of the main pad. */
	private static double[] site(SiteSurvey survey, double east, double north) {
		Vector3f row = survey.rowDirection();
		Vector3f out = survey.spectatorDirection().negate();
		return new double[] { east * row.x - north * row.z, east * out.x - north * out.z };
	}

	private static boolean crosses(SiteSurvey survey, double e0, double n0, double e1, double n1, int kind) {
		for (double t = 0; t <= 1; t += 0.01) {
			double[] at = site(survey, e0 + (e1 - e0) * t, n0 + (n1 - n0) * t);
			if (survey.nearClass(at[0], at[1]) == kind) {
				return true;
			}
		}
		return false;
	}

	@Test
	void surveyedGroundLeavesTheHillsToTheirOwnMesh() {
		SiteSurvey survey = SiteSurvey.near(-27.85, 152.96);
		LaunchSite site = new LaunchSite(new Vector3f(1, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0.4f, 0.77f, -0.5f),
				1.5, new Vector3f(0, 1, 0), 1.5, 0.0254, 0.05, Math.PI / 2, 3,
				List.of(new double[] { 300, -200 }, new double[] { 3000, 0 }), survey);
		double[] areas = site.groundAreas();
		double hills = 2 * survey.farHalf();

		assertEquals(4 * HALF * HALF - hills * hills + 4096 * 4096, areas[0] + areas[1] + areas[2] + areas[3] + areas[4], 1.0);
		assertEquals(100 * 100 + 64 * 64, areas[0] + areas[1] + areas[4], 1e-6,
				"close-up grass and track at the pads and the near landing only");
		assertTrue(areas[4] > 2 * LaunchSite.TENT_LINE * 4, "the dirt track behind the tents, close up");
	}
}
