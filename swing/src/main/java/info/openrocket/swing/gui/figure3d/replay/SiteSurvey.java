package info.openrocket.swing.gui.figure3d.replay;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

/**
 * What the ground is like round a launch site the replay knows: the land cover near the pads and
 * out to the hills, and the height of the ground, on a grid centred on the launch area with
 * rows running north to south. Positions are in metres east and north of the launch area.
 *
 * <p>Built by {@code artwork/replay-sites/<site>/make_site_data.py} from OpenStreetMap
 * (&copy; OpenStreetMap contributors, ODbL) and the Terrain Tiles on AWS Open Data.</p>
 */
final class SiteSurvey {
	// Land cover classes, as the data files store them
	static final int PADDOCK = 0;
	static final int PARK = 1;
	static final int PASTURE = 2;
	static final int WOOD = 3;
	static final int WATER = 4;
	static final int RESIDENTIAL = 5;
	static final int ROAD = 6;
	static final int WORKS = 7;
	static final int FARMLAND = 8;
	static final int GULLY = 9;

	/** What a paddock is planted with, where the map does not say. */
	enum CropKind {
		/** Irrigated under a centre pivot: bright green, with the wheel tracks in rings. */
		PIVOT,
		/** Freshly ploughed: bare brown soil in furrows. */
		PLOUGHED,
		/** A green crop in rows. */
		GREEN
	}

	/**
	 * A cropped paddock: a polygon of (east, north) corners in metres, or for a centre pivot a
	 * circle as its centre and radius, cut by the polygon.
	 */
	record Crop(CropKind kind, double[][] corners, double[] circle) {
	}

	/**
	 * QRS launch site LA2 at Cedar Grove, Queensland. The crops south of Cedar Grove Road and the
	 * track into the launch paddock are placed by eye from aerial photographs.
	 */
	private static final Site CEDAR_GROVE = new Site("cedar-grove", -27.84968, 152.96084, 3200, 4, 16_384, 32, 64,
			new double[][] { { 165, -376 }, { 190, -200 }, { 215, -30 }, { 245, 180 }, { 270, 363 }, { 363, 426 } },
			List.of(
					new Crop(CropKind.PIVOT, new double[][] { { -40, -400 }, { 620, -500 }, { 620, -1180 }, { -160, -1180 } },
							new double[] { 230, -790, 360 }),
					new Crop(CropKind.PLOUGHED, new double[][] { { -480, -310 }, { -62, -370 }, { -128, -785 },
							{ -480, -785 } }, null),
					new Crop(CropKind.GREEN, new double[][] { { -860, -335 }, { -495, -315 }, { -495, -600 },
							{ -860, -600 } }, null),
					new Crop(CropKind.GREEN, new double[][] { { -784, -980 }, { -500, -980 }, { -500, -1300 },
							{ -784, -1300 } }, null),
					new Crop(CropKind.PLOUGHED, new double[][] { { -495, -1000 }, { -250, -1000 }, { -260, -1250 },
							{ -495, -1250 } }, null)));
	/** A launch site this close to a surveyed one is taken to be it, metres. */
	private static final double SAME_SITE = 3000;
	private static final double HEIGHT_OFFSET = 1000;

	private record Site(String folder, double latitude, double longitude, double nearHalf, double nearCell,
			double farHalf, double farCell, double heightCell, double[][] track, List<Crop> crops) {
	}

	private final Site site;
	private final byte[] near;
	private final int nearSize;
	private final byte[] far;
	private final int farSize;
	private final float[] heights;
	private final int heightSize;
	private final double padHeight;

	private SiteSurvey(Site site) throws IOException {
		this.site = site;
		Raster nearRaster = read(site, "landcover_near.png").getRaster();
		this.nearSize = nearRaster.getWidth();
		this.near = bytes(nearRaster);
		Raster farRaster = read(site, "landcover_far.png").getRaster();
		this.farSize = farRaster.getWidth();
		this.far = bytes(farRaster);
		Raster heightRaster = read(site, "heights.png").getRaster();
		this.heightSize = heightRaster.getWidth();
		this.heights = new float[heightSize * heightSize];
		for (int y = 0; y < heightSize; y++) {
			for (int x = 0; x < heightSize; x++) {
				heights[y * heightSize + x] = (float) (heightRaster.getSample(x, y, 0) - HEIGHT_OFFSET);
			}
		}
		this.padHeight = rawHeight(0, 0);
	}

	/**
	 * The survey of the launch site at a latitude and longitude, or null when the replay knows
	 * no site there or its data cannot be read.
	 */
	static SiteSurvey near(double latitude, double longitude) {
		double north = (latitude - CEDAR_GROVE.latitude()) * 110_574;
		double east = (longitude - CEDAR_GROVE.longitude()) * 111_320 * Math.cos(Math.toRadians(latitude));
		if (!Double.isFinite(north) || !Double.isFinite(east) || Math.hypot(north, east) > SAME_SITE) {
			return null;
		}
		try {
			return new SiteSurvey(CEDAR_GROVE);
		} catch (IOException | RuntimeException e) {
			return null;
		}
	}

	private static BufferedImage read(Site site, String name) throws IOException {
		String path = "/datafiles/replay/" + site.folder() + "/" + name;
		try (InputStream in = SiteSurvey.class.getResourceAsStream(path)) {
			if (in == null) {
				throw new IOException("Missing " + path);
			}
			BufferedImage image = ImageIO.read(in);
			if (image == null) {
				throw new IOException("Unreadable " + path);
			}
			return image;
		}
	}

	private static byte[] bytes(Raster raster) {
		int width = raster.getWidth(), height = raster.getHeight();
		byte[] out = new byte[width * height];
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				out[y * width + x] = (byte) raster.getSample(x, y, 0);
			}
		}
		return out;
	}

	/** The vehicle track into the launch paddock, as (east, north) points in metres. */
	double[][] track() {
		return site.track();
	}

	/** Cropped paddocks the land cover does not show. */
	List<Crop> crops() {
		return site.crops();
	}

	/** Half the width of the close land cover, metres. */
	double nearHalf() {
		return site.nearHalf();
	}

	/** Half the width of the far land cover and the heights, metres. */
	double farHalf() {
		return site.farHalf();
	}

	/** Land cover class close to the launch area, from the finer data. */
	int nearClass(double east, double north) {
		return classAt(near, nearSize, site.nearHalf(), site.nearCell(), east, north);
	}

	/** Land cover class anywhere in the survey, from the coarser data. */
	int farClass(double east, double north) {
		return classAt(far, farSize, site.farHalf(), site.farCell(), east, north);
	}

	private static int classAt(byte[] grid, int size, double half, double cell, double east, double north) {
		int x = (int) Math.floor((east + half) / cell);
		int y = (int) Math.floor((half - north) / cell);
		if (x < 0 || y < 0 || x >= size || y >= size) {
			return PADDOCK;
		}
		return grid[y * size + x] & 0xFF;
	}

	/** Height of the ground above the launch area, metres. */
	double height(double east, double north) {
		return rawHeight(east, north) - padHeight;
	}

	private double rawHeight(double east, double north) {
		double cell = site.heightCell();
		double x = (east + site.farHalf()) / cell - 0.5;
		double y = (site.farHalf() - north) / cell - 0.5;
		x = Math.max(0, Math.min(heightSize - 1.001, x));
		y = Math.max(0, Math.min(heightSize - 1.001, y));
		int x0 = (int) x, y0 = (int) y;
		double fx = x - x0, fy = y - y0;
		float a = heights[y0 * heightSize + x0], b = heights[y0 * heightSize + x0 + 1];
		float c = heights[(y0 + 1) * heightSize + x0], d = heights[(y0 + 1) * heightSize + x0 + 1];
		return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
	}
}
