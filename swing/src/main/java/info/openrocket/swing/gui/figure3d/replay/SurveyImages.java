package info.openrocket.swing.gui.figure3d.replay;

import java.awt.image.BufferedImage;

import static info.openrocket.swing.gui.figure3d.replay.LaunchSiteImages.mix;
import static info.openrocket.swing.gui.figure3d.replay.LaunchSiteImages.rgb;
import static info.openrocket.swing.gui.figure3d.replay.LaunchSiteImages.scale;
import static info.openrocket.swing.gui.figure3d.replay.LaunchSiteImages.smoothstep;

/**
 * Paints the ground round a surveyed launch site from its land cover: the paddock image near the
 * pads, and the country out to the hills with the slopes shaded for the sun.
 *
 * <p>Site coordinates run along the line of pads (r) and towards the spectators (d) from the main
 * pad, in metres; {@link SiteSurvey} takes them as (r, -d).</p>
 */
final class SurveyImages {
	/** The country image: pixels across, covering the whole survey. */
	static final int TERRAIN_PIXELS = 2048;

	// sRGB, 0..1
	private static final float[] TUSSOCK = { 0.50f, 0.53f, 0.31f };
	private static final float[] TUSSOCK_DRY = { 0.66f, 0.61f, 0.41f };
	private static final float[] TUSSOCK_DARK = { 0.34f, 0.38f, 0.22f };
	private static final float[] PASTURE = { 0.44f, 0.56f, 0.27f };
	private static final float[] CROP = { 0.52f, 0.64f, 0.28f };
	private static final float[] CANOPY = { 0.27f, 0.33f, 0.19f };
	private static final float[] CANOPY_LIGHT = { 0.38f, 0.43f, 0.27f };
	private static final float[] RIVER = { 0.42f, 0.40f, 0.29f };
	private static final float[] RIVER_DEEP = { 0.30f, 0.32f, 0.24f };
	private static final float[] LAKE = { 0.24f, 0.33f, 0.36f };
	private static final float[] GULLY = { 0.33f, 0.42f, 0.21f };
	private static final float[] LAWN = { 0.47f, 0.57f, 0.30f };
	private static final float[] ROOF = { 0.55f, 0.55f, 0.53f };
	private static final float[] BITUMEN = { 0.31f, 0.31f, 0.32f };
	private static final float[] VERGE = { 0.60f, 0.58f, 0.46f };
	private static final float[] BARE = { 0.58f, 0.52f, 0.41f };
	private static final float[] DIRT = { 0.60f, 0.51f, 0.38f };
	private static final float[] PIVOT = { 0.38f, 0.60f, 0.19f };
	private static final float[] PIVOT_DRY = { 0.58f, 0.64f, 0.27f };
	private static final float[] SOIL = { 0.40f, 0.31f, 0.25f };
	private static final float[] SOIL_DARK = { 0.31f, 0.24f, 0.20f };

	private SurveyImages() {
	}

	/** Colour of a land cover class at a point in site coordinates. */
	private static float[] cover(int kind, double r, double d, LaunchSiteImages.ValueNoise[] noise, double scale) {
		double patches = noise[0].fbm(r / 600 + 7, d / 600 + 3, 3);
		double clumps = noise[1].fbm(r / 40, d / 40, 2);
		double speckle = noise[2].periodic(r / 3.5, d / 3.5);
		switch (kind) {
			case SiteSurvey.WOOD -> {
				float[] c = mix(CANOPY, CANOPY_LIGHT, (float) (0.25 + 0.75 * noise[2].periodic(r / 6, d / 6)));
				return scale(c, (float) (0.8 + 0.35 * clumps));
			}
			case SiteSurvey.WATER -> {
				return mix(RIVER, RIVER_DEEP, (float) (0.4 * clumps));
			}
			case SiteSurvey.GULLY -> {
				return scale(GULLY, (float) (0.9 + 0.2 * speckle));
			}
			case SiteSurvey.RESIDENTIAL -> {
				// Lawns and gardens, broken by drives and sheds at this scale.
				float[] c = speckle > 0.72 ? ROOF : mix(LAWN, CANOPY_LIGHT, (float) (0.5 * clumps));
				return scale(c, (float) (0.92 + 0.12 * patches));
			}
			case SiteSurvey.WORKS -> {
				return mix(BARE, TUSSOCK, (float) (0.4 * clumps));
			}
			case SiteSurvey.FARMLAND -> {
				return mix(CROP, PIVOT_DRY, (float) (0.6 * patches));
			}
			case SiteSurvey.PASTURE -> {
				return scale(mix(PASTURE, TUSSOCK_DRY, (float) (0.4 * smoothstep(0.5, 0.8, patches))),
						(float) (0.94 + 0.12 * clumps));
			}
			default -> {
				// The open paddock: tussocky grass, greener and drier in patches, darker in clumps.
				float dry = (float) smoothstep(0.42, 0.75, patches);
				float[] c = mix(TUSSOCK, TUSSOCK_DRY, 0.8f * dry);
				c = mix(c, TUSSOCK_DARK, (float) (0.45 * smoothstep(0.55, 0.9, speckle * scale + clumps * (1 - scale))));
				return scale(c, (float) (0.95 + 0.08 * clumps));
			}
		}
	}

	private static LaunchSiteImages.ValueNoise[] noise(long seed) {
		return new LaunchSiteImages.ValueNoise[] { new LaunchSiteImages.ValueNoise(seed + 31, 64),
				new LaunchSiteImages.ValueNoise(seed + 32, 64), new LaunchSiteImages.ValueNoise(seed + 33, 256),
				new LaunchSiteImages.ValueNoise(seed + 34, 64), new LaunchSiteImages.ValueNoise(seed + 35, 64) };
	}

	/** The paddock image round the pads, from the close land cover and the crops placed by eye. */
	static BufferedImage field(LaunchSiteImages.Plan plan, float[] centre) {
		SiteSurvey survey = plan.survey();
		int n = LaunchSiteImages.FIELD_PIXELS;
		double span = plan.fieldSpan();
		double metres = span / n;
		LaunchSiteImages.ValueNoise[] noise = noise(plan.seed());
		int[] pixels = new int[n * n];
		for (int j = 0; j < n; j++) {
			double d = (j + 0.5) * metres - span / 2;
			for (int i = 0; i < n; i++) {
				double r = (i + 0.5) * metres - span / 2;
				// Wobble the lookup so the edges of the 4 m cells do not show as steps.
				double wr = r + 5 * (noise[3].periodic(r / 9, d / 9) - 0.5);
				double wd = d + 5 * (noise[4].periodic(r / 9, d / 9) - 0.5);
				int kind = survey.nearClass(wr, -wd);
				if (kind == SiteSurvey.ROAD || kind == SiteSurvey.GULLY) {
					kind = SiteSurvey.PADDOCK;	// drawn over the top, below
				}
				float[] colour = cover(kind, r, d, noise, 0.6);
				float[] crop = crop(survey, r, d, noise);
				if (crop != null) {
					colour = crop;
				}
				colour = mix(centre, colour, (float) smoothstep(plan.clearRadius(), plan.clearRadius() * 2.2,
						Math.hypot(r, d)));
				pixels[j * n + i] = rgb(colour);
			}
		}
		gullies(survey, pixels, n, span);
		roads(survey, pixels, n, span);
		track(survey.track(), pixels, n, span);
		shadows(plan, pixels, n, span);
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		image.setRGB(0, 0, n, n, pixels, 0, n);
		return image;
	}

	/** The crop on a hand-placed paddock at a point, or null. */
	private static float[] crop(SiteSurvey survey, double r, double d, LaunchSiteImages.ValueNoise[] noise) {
		double east = r, north = -d;
		for (SiteSurvey.Crop crop : survey.crops()) {
			if (!inside(crop.corners(), east, north)) {
				continue;
			}
			double variation = noise[1].fbm(r / 50, d / 50, 2);
			switch (crop.kind()) {
				case PIVOT -> {
					double[] c = crop.circle();
					double fromCentre = Math.hypot(east - c[0], north - c[1]);
					if (fromCentre > c[2]) {
						return null;
					}
					float[] colour = mix(PIVOT, PIVOT_DRY, (float) (0.7 * smoothstep(0.45, 0.8, variation)));
					// Wheel tracks every 55 m out from the pivot.
					double ring = Math.abs((fromCentre % 55) - 27.5);
					if (ring > 26.5) {
						colour = scale(colour, 0.82f);
					}
					return colour;
				}
				case PLOUGHED -> {
					double furrow = 0.5 + 0.5 * Math.sin(north * 2.2);
					return scale(mix(SOIL, SOIL_DARK, (float) (0.35 * furrow + 0.3 * variation)), 1f);
				}
				default -> {
					double rowLine = 0.5 + 0.5 * Math.sin(east * 1.9);
					return scale(mix(CROP, PASTURE, (float) (0.5 * variation)), (float) (0.94 + 0.08 * rowLine));
				}
			}
		}
		return null;
	}

	private static boolean inside(double[][] polygon, double x, double y) {
		boolean in = false;
		for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
			if ((polygon[i][1] > y) != (polygon[j][1] > y)
					&& x < (polygon[j][0] - polygon[i][0]) * (y - polygon[i][1]) / (polygon[j][1] - polygon[i][1])
					+ polygon[i][0]) {
				in = !in;
			}
		}
		return in;
	}

	/** Creeks and gullies: soft strips of greener, ranker grass along the map's dotted lines. */
	private static void gullies(SiteSurvey survey, int[] pixels, int n, double span) {
		double metres = span / n;
		double cell = 4;
		for (double north = -span / 2; north < span / 2; north += cell) {
			for (double east = -span / 2; east < span / 2; east += cell) {
				if (survey.nearClass(east + cell / 2, north + cell / 2) == SiteSurvey.GULLY) {
					stamp(pixels, n, span, metres, east + cell / 2, -(north + cell / 2), 7, GULLY, 0.45f);
				}
			}
		}
	}

	/** Sealed roads along the land cover's road centre lines: bitumen with a pale verge. */
	private static void roads(SiteSurvey survey, int[] pixels, int n, double span) {
		double metres = span / n;
		double cell = 4;
		for (double north = -span / 2; north < span / 2; north += cell) {
			for (double east = -span / 2; east < span / 2; east += cell) {
				if (survey.nearClass(east + cell / 2, north + cell / 2) != SiteSurvey.ROAD) {
					continue;
				}
				stamp(pixels, n, span, metres, east + cell / 2, -(north + cell / 2), 5.5, VERGE, 0.6f);
			}
		}
		for (double north = -span / 2; north < span / 2; north += cell) {
			for (double east = -span / 2; east < span / 2; east += cell) {
				if (survey.nearClass(east + cell / 2, north + cell / 2) != SiteSurvey.ROAD) {
					continue;
				}
				stamp(pixels, n, span, metres, east + cell / 2, -(north + cell / 2), 3.4, BITUMEN, 1f);
			}
		}
	}

	/** The dirt track into the launch paddock: two wheel ruts with grass between. */
	private static void track(double[][] points, int[] pixels, int n, double span) {
		double metres = span / n;
		for (int k = 0; k + 1 < points.length; k++) {
			double e0 = points[k][0], n0 = points[k][1], e1 = points[k + 1][0], n1 = points[k + 1][1];
			double length = Math.hypot(e1 - e0, n1 - n0);
			for (double s = 0; s <= length; s += metres / 2) {
				double east = e0 + (e1 - e0) * s / length, north = n0 + (n1 - n0) * s / length;
				stamp(pixels, n, span, metres, east, -north, 2.2, DIRT, 0.8f);
			}
		}
	}

	/** Tree and house shade, where the sun throws it. */
	private static void shadows(LaunchSiteImages.Plan plan, int[] pixels, int n, double span) {
		double metres = span / n;
		for (double[] tree : plan.trees()) {
			double cr = tree[0] + plan.shadowR() * tree[2];
			double cd = tree[1] + plan.shadowD() * tree[2];
			shade(pixels, n, span, metres, cr, cd, tree[3] * 1.1, 0.40);
		}
		for (double[] house : plan.houses()) {
			shade(pixels, n, span, metres, house[0] + plan.shadowR() * 2.5, house[1] + plan.shadowD() * 2.5,
					Math.max(house[2], house[3]) * 0.6, 0.35);
		}
	}

	private static void stamp(int[] pixels, int n, double span, double metres, double r, double d, double radius,
			float[] colour, float strength) {
		int i0 = (int) Math.floor((r - radius + span / 2) / metres), i1 = (int) Math.ceil((r + radius + span / 2) / metres);
		int j0 = (int) Math.floor((d - radius + span / 2) / metres), j1 = (int) Math.ceil((d + radius + span / 2) / metres);
		for (int j = Math.max(0, j0); j <= Math.min(n - 1, j1); j++) {
			for (int i = Math.max(0, i0); i <= Math.min(n - 1, i1); i++) {
				double pr = (i + 0.5) * metres - span / 2, pd = (j + 0.5) * metres - span / 2;
				double cover = 1 - smoothstep(radius - metres, radius + metres * 0.5, Math.hypot(pr - r, pd - d));
				if (cover > 0) {
					pixels[j * n + i] = blend(pixels[j * n + i], colour, (float) (cover * strength));
				}
			}
		}
	}

	private static void shade(int[] pixels, int n, double span, double metres, double r, double d, double radius,
			double depth) {
		int i0 = (int) Math.floor((r - radius - 2 + span / 2) / metres), i1 = (int) Math.ceil((r + radius + 2 + span / 2) / metres);
		int j0 = (int) Math.floor((d - radius - 2 + span / 2) / metres), j1 = (int) Math.ceil((d + radius + 2 + span / 2) / metres);
		for (int j = Math.max(0, j0); j <= Math.min(n - 1, j1); j++) {
			for (int i = Math.max(0, i0); i <= Math.min(n - 1, i1); i++) {
				double pr = (i + 0.5) * metres - span / 2, pd = (j + 0.5) * metres - span / 2;
				double amount = 1 - smoothstep(radius * 0.6, radius + 1.5, Math.hypot(pr - r, pd - d));
				if (amount > 0) {
					pixels[j * n + i] = LaunchSiteImages.gain(pixels[j * n + i], (float) (1 - depth * amount));
				}
			}
		}
	}

	private static int blend(int rgb, float[] colour, float amount) {
		float[] base = { ((rgb >> 16) & 0xFF) / 255f, ((rgb >> 8) & 0xFF) / 255f, (rgb & 0xFF) / 255f };
		return rgb(mix(base, colour, amount));
	}

	/**
	 * The country over the whole survey, from the coarse land cover, with every slope lit as the
	 * sun would light it, as the ground is drawn unlit.
	 *
	 * @param toSun unit vector to the sun as (east, up, south)
	 */
	static BufferedImage terrain(LaunchSiteImages.Plan plan, float[] toSun) {
		SiteSurvey survey = plan.survey();
		int n = TERRAIN_PIXELS;
		double span = 2 * survey.farHalf();
		double metres = span / n;
		// The same noise as the paddock image, so the two meet without a seam.
		LaunchSiteImages.ValueNoise[] noise = noise(plan.seed());
		double flat = 0.40 + 0.62 * Math.max(0, toSun[1]) + 0.10;
		double close = survey.nearHalf() - 100;
		int[] pixels = new int[n * n];
		for (int j = 0; j < n; j++) {
			double d = (j + 0.5) * metres - span / 2;
			for (int i = 0; i < n; i++) {
				double r = (i + 0.5) * metres - span / 2;
				double wr = r + 30 * (noise[3].periodic(r / 60, d / 60) - 0.5);
				double wd = d + 30 * (noise[4].periodic(r / 60, d / 60) - 0.5);
				// The finer land cover where there is some, so the country carries on from the paddock image.
				double out = Math.max(Math.abs(r), Math.abs(d));
				boolean near = out < close;
				int kind = near ? survey.nearClass(wr, -wd) : survey.farClass(wr, -wd);
				boolean road = kind == SiteSurvey.ROAD;
				if (road || kind == SiteSurvey.GULLY) {
					kind = near ? SiteSurvey.PADDOCK : survey.farClass(wr + 32, -wd - 32);
				}
				float[] colour = kind == SiteSurvey.WATER && !near ? LAKE : cover(kind, r, d, noise, 0.6);
				// Seen from afar the paddocks' patches fade into one another.
				float fade = (float) (0.4 * smoothstep(close, close + 2500, out));
				colour = mix(colour, TUSSOCK, kind == SiteSurvey.PADDOCK || kind == SiteSurvey.PARK ? fade : 0);
				if (road) {
					colour = mix(colour, BITUMEN, 0.35f);
				}
				// Slope lighting from the heights either side.
				double step = metres;
				double dhdr = (survey.height(r + step, -d) - survey.height(r - step, -d)) / (2 * step);
				double dhdd = (survey.height(r, -(d + step)) - survey.height(r, -(d - step))) / (2 * step);
				double nr = -dhdr, nu = 1, nd = -dhdd;
				double length = Math.sqrt(nr * nr + nu * nu + nd * nd);
				double sun = Math.max(0, (nr * toSun[0] + nu * toSun[1] + nd * toSun[2]) / length);
				double light = (0.40 + 0.62 * sun + 0.10 * nu / length) / flat;
				pixels[j * n + i] = rgb(scale(colour, (float) light));
			}
		}
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		image.setRGB(0, 0, n, n, pixels, 0, n);
		return image;
	}
}
