package info.openrocket.swing.gui.figure3d.replay;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/** Layout of the launch site's ground, which needs no OpenGL. */
class LaunchSiteTest {
	private static final double HALF = 32_768;

	private static LaunchSite site(List<double[]> landings) {
		return new LaunchSite(new Vector3f(1, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0.4f, 0.77f, -0.5f), 1.5,
				new Vector3f(0, 1, 0), 1.5, 0.0254, 0.05, Math.PI / 2, 3, landings);
	}

	@Test
	void groundCoversEverythingOnceWithCloseUpGrassAroundThePads() {
		LaunchSite site = site(List.of());
		double[] areas = site.groundAreas();
		double[] mown = site.mownArea();
		double mownArea = (mown[1] - mown[0]) * (mown[3] - mown[2]);

		assertEquals(4 * HALF * HALF, areas[0] + areas[1] + areas[2] + areas[3], 1.0, "whole ground, no gaps or overlaps");
		assertEquals(mownArea, areas[1], 1e-6, "mown launch area");
		assertEquals(100 * 100 - mownArea, areas[0], 1e-6, "long grass round it, to the fence");
		assertEquals(2048 * 2048 - 100 * 100, areas[2], 1e-6, "the paddock image");
	}

	@Test
	void whereSomethingLandsGetsCloseUpGrassToo() {
		// One landing in the paddock, one out in the country beyond it.
		double[] areas = site(List.of(new double[] { 300, -200 }, new double[] { 2500, 900 })).groundAreas();
		double[] plain = site(List.of()).groundAreas();

		assertEquals(4 * HALF * HALF, areas[0] + areas[1] + areas[2] + areas[3], 1.0);
		assertEquals(plain[0] + 2 * 64 * 64, areas[0], 1e-6, "a 64 m square of long grass at each landing");
		assertEquals(plain[2] - 64 * 64, areas[2], 1e-6);
		assertEquals(plain[3] - 64 * 64, areas[3], 1.0);
	}
}
