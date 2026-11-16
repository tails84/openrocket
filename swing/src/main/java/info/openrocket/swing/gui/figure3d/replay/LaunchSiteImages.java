package info.openrocket.swing.gui.figure3d.replay;

import org.lwjgl.BufferUtils;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.util.List;
import java.util.Random;

/**
 * Paints the launch site's textures: grass close up, the paddock seen from the air, and the
 * colour ramps the scenery is shaded with. Pure Java, so it can run off the GL thread; the images
 * come out as uncompressed TGA data, which the texture loader reads directly.
 *
 * <p>Image pixel (i, j) maps to texture coordinates ((i + 0.5) / width, (j + 0.5) / height).</p>
 */
final class LaunchSiteImages {
	/** Close-up grass: pixels across and the metres one tile covers. */
	static final int DETAIL_PIXELS = 1024;
	static final double DETAIL_TILE = 3.0;
	/** The paddock around the site, one image without repeats: pixels across. */
	static final int FIELD_PIXELS = 2048;
	/** The country beyond the paddock, repeating: pixels across and metres one tile covers. */
	static final int FAR_PIXELS = 1024;
	static final double FAR_TILE = 1536;
	/** Shade levels across a colour ramp, and rows per colour variant. */
	static final int RAMP_SHADES = 64;
	static final int RAMP_ROWS = 8;
	/** Highest brightness a ramp reaches, as a multiple of the base colour. */
	static final float RAMP_TOP = 1.1f;
	/** Colour variants across the grass blade image. */
	static final int BLADE_VARIANTS = 8;

	// sRGB, 0..1
	private static final float[] GRASS = { 0.40f, 0.53f, 0.25f };
	private static final float[] GRASS_DARK = { 0.26f, 0.37f, 0.15f };
	private static final float[] GRASS_DRY = { 0.69f, 0.64f, 0.39f };
	private static final float[] GRASS_FRESH = { 0.47f, 0.62f, 0.28f };
	private static final float[] MOWN = { 0.47f, 0.60f, 0.29f };
	private static final float[] SOIL = { 0.55f, 0.46f, 0.34f };
	private static final float[] RUT = { 0.47f, 0.39f, 0.29f };
	private static final float[] WATER = { 0.22f, 0.31f, 0.33f };
	private static final float[] MUD = { 0.50f, 0.43f, 0.31f };
	private static final float[] TUSSOCK_SHADE = { 0.42f, 0.42f, 0.25f };

	/**
	 * Where the paddock's features are, in metres on the site's row (r) and depth (d) axes.
	 *
	 * @param seed        seed for the random grass
	 * @param trees       per tree: r, d, canopy height above the ground and canopy radius
	 * @param track       a dirt track as a polyline of (r, d) points
	 * @param dam         farm dam centre r, d and radius
	 * @param shadowR     where a shadow falls per metre of height, along r
	 * @param shadowD     the same along d
	 * @param clearRadius radius around the site the paddock shows as plain grass, where the
	 *                    close-up grass takes over
	 * @param fieldSpan   metres the paddock image covers
	 * @param survey      what the ground is like round a known launch site, or null for a made-up paddock
	 * @param houses      per house: r, d, width and length, metres
	 * @param toSun       unit vector to the sun as (r, up, d)
	 */
	record Plan(long seed, List<double[]> trees, double[][] track, double[] dam, double shadowR, double shadowD,
			double clearRadius, double fieldSpan, SiteSurvey survey, List<double[]> houses, float[] toSun) {
	}

	/** The images, as TGA file data; the country over a surveyed site's hills, or null. */
	record Images(ByteBuffer longGrass, ByteBuffer mownGrass, ByteBuffer field, ByteBuffer far, ByteBuffer blades,
			ByteBuffer terrain) {
	}

	private LaunchSiteImages() {
	}

	static Images paint(Plan plan) {
		// A surveyed site's long grass is the drier tussock of a grazed paddock.
		float dry = plan.survey() != null ? 0.45f : 0;
		BufferedImage longGrass = longGrass(plan.seed(), dry);
		// Drawn from a distance the close-up grass looks a shade darker than its average.
		float[] mean = scale(mean(longGrass), 0.94f);
		if (plan.survey() != null) {
			return new Images(tga(longGrass), tga(mownGrass(plan.seed() + 1, dry)), tga(SurveyImages.field(plan, mean)),
					tga(far(plan.seed())), tga(blades(dry)), tga(SurveyImages.terrain(plan, plan.toSun())));
		}
		return new Images(tga(longGrass), tga(mownGrass(plan.seed() + 1, dry)), tga(field(plan, mean)), tga(far(plan.seed())),
				tga(blades(dry)), null);
	}

	// ---------- close-up grass ----------

	/** Long grass from above: blades in every direction over dark gaps, a few of them dry. */
	/** @param dry how far the colours lean from green towards dry tussock, 0..1 */
	private static BufferedImage longGrass(long seed, float dry) {
		int n = DETAIL_PIXELS;
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		Graphics2D g = image.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setColor(color(mix(scale(GRASS_DARK, 0.85f), TUSSOCK_SHADE, dry)));
		g.fillRect(0, 0, n, n);
		Random random = new Random(seed);
		// Clumps: patches where the blades are lighter or darker, a few tens of centimetres across.
		ValueNoise clumps = new ValueNoise(seed + 7, 12);
		for (int i = 0; i < 70_000; i++) {
			double x = random.nextDouble() * n, y = random.nextDouble() * n;
			double clump = clumps.periodic(x / n * 12, y / n * 12);
			float[] colour = random.nextDouble() < 0.12 + 0.2 * clump + 0.35 * dry ? GRASS_DRY
					: mix(GRASS_DARK, GRASS_FRESH, (float) (0.25 + 0.6 * random.nextDouble() * (0.6 + 0.6 * clump)));
			colour = scale(mix(colour, TUSSOCK_SHADE, 0.35f * dry), (float) (0.85 + 0.25 * random.nextDouble()));
			double angle = random.nextDouble() * Math.PI * 2;
			double length = 5 + random.nextDouble() * 12;
			strokeWrapped(g, n, x, y, x + Math.cos(angle) * length, y + Math.sin(angle) * length,
					(float) (1.0 + random.nextDouble() * 1.4), colour);
		}
		g.dispose();
		return image;
	}

	/** Mown grass: short speckled blades, and the light and dark stripes the mower leaves. */
	/** @param dry how far the colours lean from green towards dry tussock, 0..1 */
	private static BufferedImage mownGrass(long seed, float dry) {
		int n = DETAIL_PIXELS;
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		Graphics2D g = image.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setColor(color(mix(scale(MOWN, 0.9f), TUSSOCK_SHADE, 0.5f * dry)));
		g.fillRect(0, 0, n, n);
		Random random = new Random(seed);
		for (int i = 0; i < 90_000; i++) {
			double x = random.nextDouble() * n, y = random.nextDouble() * n;
			float[] colour = random.nextDouble() < 0.06 + 0.25 * dry ? mix(MOWN, GRASS_DRY, 0.6f)
					: scale(mix(GRASS_DARK, GRASS_FRESH, (float) (0.45 + 0.55 * random.nextDouble())),
							(float) (0.92 + 0.16 * random.nextDouble()));
			colour = mix(colour, TUSSOCK_SHADE, 0.3f * dry);
			double angle = random.nextDouble() * Math.PI * 2;
			double length = 2 + random.nextDouble() * 4;
			strokeWrapped(g, n, x, y, x + Math.cos(angle) * length, y + Math.sin(angle) * length,
					(float) (1.0 + random.nextDouble()), colour);
		}
		g.dispose();
		// Two mower passes per tile, along the image's columns.
		for (int i = 0; i < n; i++) {
			double stripe = Math.cos(2 * Math.PI * 2 * (i + 0.5) / n);
			float gain = (float) (1 + 0.035 * Math.signum(stripe) * Math.min(1, Math.abs(stripe) * 4));
			for (int j = 0; j < n; j++) {
				image.setRGB(i, j, gain(image.getRGB(i, j), gain));
			}
		}
		return image;
	}

	private static void strokeWrapped(Graphics2D g, int n, double x0, double y0, double x1, double y1, float width,
			float[] colour) {
		g.setColor(color(colour));
		g.setStroke(new BasicStroke(width, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
		for (int dx = -1; dx <= 1; dx++) {
			for (int dy = -1; dy <= 1; dy++) {
				double minX = Math.min(x0, x1) + dx * n, maxX = Math.max(x0, x1) + dx * n;
				double minY = Math.min(y0, y1) + dy * n, maxY = Math.max(y0, y1) + dy * n;
				if (maxX < -4 || minX > n + 4 || maxY < -4 || minY > n + 4) {
					continue;
				}
				g.draw(new java.awt.geom.Line2D.Double(x0 + dx * n, y0 + dy * n, x1 + dx * n, y1 + dy * n));
			}
		}
	}

	// ---------- the paddock ----------

	/** Grass colour of the open paddock at a point, metres; repeats every {@link #FAR_TILE}. */
	private static float[] paddock(ValueNoise[] noise, double r, double d) {
		double x = r / FAR_TILE, y = d / FAR_TILE;
		// Bend the coordinates with more noise so the patches do not line up with the lattice.
		double warpX = noise[3].fbm(x * 16, y * 16, 2) - 0.5;
		double warpY = noise[4].fbm(x * 16, y * 16, 2) - 0.5;
		double patches = noise[0].fbm(x * 8 + 1.6 * warpX, y * 8 + 1.6 * warpY, 3);
		double clumps = noise[1].fbm(x * 48 + 2.0 * warpY, y * 48 + 2.0 * warpX, 3);
		double fine = noise[2].periodic(x * 160, y * 160);
		float dry = (float) smoothstep(0.52, 0.74, patches);
		float[] colour = mix(GRASS, GRASS_DRY, 0.65f * dry);
		colour = mix(colour, GRASS_DARK, (float) (0.35 * smoothstep(0.45, 0.8, clumps)));
		return scale(colour, (float) (0.94 + 0.08 * clumps + 0.04 * fine));
	}

	private static ValueNoise[] paddockNoise(long seed) {
		return new ValueNoise[] { new ValueNoise(seed + 11, 8), new ValueNoise(seed + 12, 48),
				new ValueNoise(seed + 13, 160), new ValueNoise(seed + 14, 16), new ValueNoise(seed + 15, 16) };
	}

	/** The paddock around the site: plain grass in the middle, a track, a dam and the shade of trees. */
	private static BufferedImage field(Plan plan, float[] centre) {
		int n = FIELD_PIXELS;
		double span = plan.fieldSpan();
		double metres = span / n;
		ValueNoise[] noise = paddockNoise(plan.seed());
		ValueNoise edge = new ValueNoise(plan.seed() + 21, 64);
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		double[] dam = plan.dam();
		for (int j = 0; j < n; j++) {
			double d = (j + 0.5) * metres - span / 2;
			for (int i = 0; i < n; i++) {
				double r = (i + 0.5) * metres - span / 2;
				float[] colour = paddock(noise, r, d);
				double fromSite = Math.hypot(r, d);
				colour = mix(centre, colour,
						(float) smoothstep(plan.clearRadius(), plan.clearRadius() * 2.2, fromSite));

				double track = distanceToPolyline(plan.track(), r, d);
				if (track < 3) {
					float bare = (float) (1 - smoothstep(1.4, 2.6, track));
					float[] ground = Math.abs(track - 0.75) < 0.3 ? RUT : SOIL;
					colour = mix(colour, ground, bare * 0.9f);
				}
				double wobble = 1 + 0.25 * (edge.periodic((r - dam[0]) / 8 + 32, (d - dam[1]) / 8 + 32) - 0.5);
				double fromDam = Math.hypot(r - dam[0], d - dam[1]) / wobble;
				if (fromDam < dam[2] + 5) {
					colour = mix(colour, MUD, (float) (1 - smoothstep(dam[2] + 1, dam[2] + 5, fromDam)));
					colour = mix(colour, WATER, (float) (1 - smoothstep(dam[2] - 1, dam[2] + 1, fromDam)));
				}
				image.setRGB(i, j, rgb(colour));
			}
		}
		// Tree shade: soft dark ellipses where the sun throws each canopy's shadow.
		for (double[] tree : plan.trees()) {
			double cr = tree[0] + plan.shadowR() * tree[2];
			double cd = tree[1] + plan.shadowD() * tree[2];
			double radius = tree[3] * 1.15;
			int i0 = (int) Math.floor((cr - radius - 2 + span / 2) / metres);
			int i1 = (int) Math.ceil((cr + radius + 2 + span / 2) / metres);
			int j0 = (int) Math.floor((cd - radius - 2 + span / 2) / metres);
			int j1 = (int) Math.ceil((cd + radius + 2 + span / 2) / metres);
			for (int j = Math.max(0, j0); j <= Math.min(n - 1, j1); j++) {
				for (int i = Math.max(0, i0); i <= Math.min(n - 1, i1); i++) {
					double r = (i + 0.5) * metres - span / 2;
					double d = (j + 0.5) * metres - span / 2;
					double shade = 1 - smoothstep(radius * 0.6, radius + 1.5, Math.hypot(r - cr, d - cd));
					if (shade > 0) {
						image.setRGB(i, j, gain(image.getRGB(i, j), (float) (1 - 0.38 * shade)));
					}
				}
			}
		}
		return image;
	}

	/** The open paddock repeated beyond the field image, matching it where they meet. */
	private static BufferedImage far(long seed) {
		int n = FAR_PIXELS;
		ValueNoise[] noise = paddockNoise(seed);
		BufferedImage image = new BufferedImage(n, n, BufferedImage.TYPE_INT_RGB);
		for (int j = 0; j < n; j++) {
			double d = (j + 0.5) / n * FAR_TILE;
			for (int i = 0; i < n; i++) {
				double r = (i + 0.5) / n * FAR_TILE;
				image.setRGB(i, j, rgb(paddock(noise, r, d)));
			}
		}
		return image;
	}

	// ---------- blades and ramps ----------

	/** Grass blades: one colour variant per column band, dark at the root (v = 0) to light at the tip. */
	private static BufferedImage blades(float dry) {
		int width = BLADE_VARIANTS * 8, height = 64;
		BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		float[][] tips = { GRASS_FRESH, GRASS, mix(GRASS, GRASS_DRY, 0.3f), GRASS_FRESH, GRASS_DRY,
				mix(GRASS_DARK, GRASS_FRESH, 0.6f), GRASS, mix(GRASS_FRESH, GRASS_DRY, 0.5f) };
		for (int v = 0; v < BLADE_VARIANTS; v++) {
			for (int j = 0; j < height; j++) {
				float along = (j + 0.5f) / height;
				float[] tip = mix(tips[v], GRASS_DRY, dry * (v % 3 == 0 ? 0.9f : 0.5f));
				float[] colour = mix(scale(GRASS_DARK, 0.7f), scale(tip, 1.08f), (float) Math.pow(along, 0.7));
				for (int i = 0; i < 8; i++) {
					image.setRGB(v * 8 + i, j, rgb(colour));
				}
			}
		}
		return image;
	}

	/**
	 * A colour ramp for unlit scenery: shade across, from black to {@link #RAMP_TOP} times the
	 * colour, and one band of {@link #RAMP_ROWS} rows for each variant colour.
	 */
	static ByteBuffer ramp(float[]... variants) {
		BufferedImage image = new BufferedImage(RAMP_SHADES, RAMP_ROWS * variants.length, BufferedImage.TYPE_INT_RGB);
		for (int v = 0; v < variants.length; v++) {
			for (int i = 0; i < RAMP_SHADES; i++) {
				int rgb = rgb(scale(variants[v], RAMP_TOP * (i + 0.5f) / RAMP_SHADES));
				for (int j = 0; j < RAMP_ROWS; j++) {
					image.setRGB(i, v * RAMP_ROWS + j, rgb);
				}
			}
		}
		return tga(image);
	}

	// ---------- helpers ----------

	private static double distanceToPolyline(double[][] points, double r, double d) {
		double best = Double.MAX_VALUE;
		for (int k = 0; k + 1 < points.length; k++) {
			double ar = points[k][0], ad = points[k][1];
			double br = points[k + 1][0] - ar, bd = points[k + 1][1] - ad;
			double t = Math.max(0, Math.min(1, ((r - ar) * br + (d - ad) * bd) / (br * br + bd * bd)));
			best = Math.min(best, Math.hypot(r - ar - t * br, d - ad - t * bd));
		}
		return best;
	}

	static double smoothstep(double edge0, double edge1, double x) {
		double t = Math.max(0, Math.min(1, (x - edge0) / (edge1 - edge0)));
		return t * t * (3 - 2 * t);
	}

	private static float[] mean(BufferedImage image) {
		double r = 0, g = 0, b = 0;
		int n = image.getWidth() * image.getHeight();
		for (int j = 0; j < image.getHeight(); j++) {
			for (int i = 0; i < image.getWidth(); i++) {
				int rgb = image.getRGB(i, j);
				r += (rgb >> 16) & 0xFF;
				g += (rgb >> 8) & 0xFF;
				b += rgb & 0xFF;
			}
		}
		return new float[] { (float) (r / n / 255), (float) (g / n / 255), (float) (b / n / 255) };
	}

	static float[] mix(float[] a, float[] b, float t) {
		return new float[] { a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t };
	}

	static float[] scale(float[] colour, float factor) {
		return new float[] { colour[0] * factor, colour[1] * factor, colour[2] * factor };
	}

	private static Color color(float[] colour) {
		return new Color(rgb(colour));
	}

	static int rgb(float[] colour) {
		int r = Math.round(Math.max(0, Math.min(1, colour[0])) * 255);
		int g = Math.round(Math.max(0, Math.min(1, colour[1])) * 255);
		int b = Math.round(Math.max(0, Math.min(1, colour[2])) * 255);
		return (r << 16) | (g << 8) | b;
	}

	static int gain(int rgb, float gain) {
		int r = Math.min(255, Math.round(((rgb >> 16) & 0xFF) * gain));
		int g = Math.min(255, Math.round(((rgb >> 8) & 0xFF) * gain));
		int b = Math.min(255, Math.round((rgb & 0xFF) * gain));
		return (r << 16) | (g << 8) | b;
	}

	/** Uncompressed 24-bit TGA, rows stored bottom first so image row 0 lands at v = 0. */
	private static ByteBuffer tga(BufferedImage image) {
		int width = image.getWidth(), height = image.getHeight();
		ByteBuffer buffer = BufferUtils.createByteBuffer(18 + width * height * 3);
		buffer.put((byte) 0).put((byte) 0).put((byte) 2);
		buffer.put(new byte[5]);
		buffer.putShort((short) 0).putShort((short) 0);
		buffer.put((byte) (width & 0xFF)).put((byte) (width >> 8));
		buffer.put((byte) (height & 0xFF)).put((byte) (height >> 8));
		buffer.put((byte) 24).put((byte) 0);
		// With the bottom-left origin, the first row stored is the bottom row of the picture; the
		// loader flips it to the top of memory and then flips the whole image back, so it ends up first.
		for (int j = 0; j < height; j++) {
			for (int i = 0; i < width; i++) {
				int rgb = image.getRGB(i, j);
				buffer.put((byte) (rgb & 0xFF)).put((byte) ((rgb >> 8) & 0xFF)).put((byte) ((rgb >> 16) & 0xFF));
			}
		}
		buffer.flip();
		return buffer;
	}

	/** Smooth value noise on a lattice that repeats every {@code period} cells. */
	static final class ValueNoise {
		private final float[] lattice;
		private final int period;

		ValueNoise(long seed, int period) {
			this.period = period;
			this.lattice = new float[period * period];
			Random random = new Random(seed);
			for (int i = 0; i < lattice.length; i++) {
				lattice[i] = random.nextFloat();
			}
		}

		/** Noise in 0..1 at lattice coordinates (x, y), repeating every period. */
		double periodic(double x, double y) {
			int x0 = (int) Math.floor(x), y0 = (int) Math.floor(y);
			double fx = x - x0, fy = y - y0;
			fx = fx * fx * fx * (fx * (fx * 6 - 15) + 10);
			fy = fy * fy * fy * (fy * (fy * 6 - 15) + 10);
			double a = at(x0, y0), b = at(x0 + 1, y0), c = at(x0, y0 + 1), e = at(x0 + 1, y0 + 1);
			return a + (b - a) * fx + (c - a) * fy + (a - b - c + e) * fx * fy;
		}

		/**
		 * Sum of octaves at doubling frequency and halving weight, normalised to 0..1. Every octave
		 * repeats with the lattice, as its frequency is a whole multiple; each is shifted by a whole
		 * number of cells so they do not share the lattice's corners.
		 */
		double fbm(double x, double y, int octaves) {
			double sum = 0, weight = 1, total = 0;
			double scale = 1;
			for (int o = 0; o < octaves; o++) {
				sum += weight * periodic(x * scale + 17 * o, y * scale + 31 * o);
				total += weight;
				weight *= 0.5;
				scale *= 2;
			}
			return sum / total;
		}

		private float at(int x, int y) {
			return lattice[Math.floorMod(x, period) + period * Math.floorMod(y, period)];
		}
	}
}
