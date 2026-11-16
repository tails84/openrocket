package info.openrocket.swing.gui.figure3d.replay;

import org.joml.Vector3f;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * What the ground is like round a launch site the replay knows: the land cover near the pads and
 * out to the hills, and the height of the ground, from grids with rows running north to south.
 *
 * <p>Positions are in the launch site's own frame, in metres from the main pad: x along the line
 * of pads, y away from the spectators, out past the pads. The site's {@link #rowDirection()} and
 * {@link #spectatorDirection()} place that frame on the map.</p>
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
	 * QRS launch site LA2 at Cedar Grove, Queensland. The flight line runs beside the dirt track
	 * into the launch paddock, with a line of tents between it and the track and the rockets
	 * flying out to the west. The crops south of Cedar Grove Road and the track are placed by eye
	 * from aerial photographs, as metres east and north of the grids' centre.
	 */
	private static final Site CEDAR_GROVE = new Site("cedar-grove", -27.84968, 152.96084, -27.850615, 152.962562,
			8.5, 3200, 4, 16_384, 32, 64,
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
	private static final double METRES_PER_DEGREE_LATITUDE = 110_574;
	private static final double METRES_PER_DEGREE_LONGITUDE = 111_320;

	/**
	 * @param latitude    centre of the grids
	 * @param padLatitude where the main pad stands
	 * @param rowBearing  bearing of the line of pads, degrees clockwise from north, with the
	 *                    spectators on its right-hand side
	 */
	private record Site(String folder, double latitude, double longitude, double padLatitude, double padLongitude,
			double rowBearing, double nearHalf, double nearCell, double farHalf, double farCell, double heightCell,
			double[][] track, List<Crop> crops) {
	}

	private final Site site;
	private final byte[] near;
	private final int nearSize;
	private final byte[] far;
	private final int farSize;
	private final float[] heights;
	private final int heightSize;
	private final double padHeight;
	/** The main pad from the grids' centre, metres east and north. */
	private final double padEast;
	private final double padNorth;
	private final double sin;
	private final double cos;

	private SiteSurvey(Site site) throws IOException {
		this.site = site;
		this.padNorth = (site.padLatitude() - site.latitude()) * METRES_PER_DEGREE_LATITUDE;
		this.padEast = (site.padLongitude() - site.longitude()) * METRES_PER_DEGREE_LONGITUDE
				* Math.cos(Math.toRadians(site.latitude()));
		this.sin = Math.sin(Math.toRadians(site.rowBearing()));
		this.cos = Math.cos(Math.toRadians(site.rowBearing()));
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
		this.padHeight = rawHeight(padEast, padNorth);
	}

	/**
	 * The survey of the launch site at a latitude and longitude, or null when the replay knows
	 * no site there or its data cannot be read.
	 */
	static SiteSurvey near(double latitude, double longitude) {
		double north = (latitude - CEDAR_GROVE.padLatitude()) * METRES_PER_DEGREE_LATITUDE;
		double east = (longitude - CEDAR_GROVE.padLongitude()) * METRES_PER_DEGREE_LONGITUDE
				* Math.cos(Math.toRadians(latitude));
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

	/** Level unit vector along the line of pads, engine axes (+X east, -Z north). */
	Vector3f rowDirection() {
		return new Vector3f((float) sin, 0, (float) -cos);
	}

	/** Level unit vector from the pads towards the spectators, engine axes. */
	Vector3f spectatorDirection() {
		return new Vector3f((float) cos, 0, (float) sin);
	}

	/** The vehicle track into the launch paddock, as (x, y) points in metres. */
	double[][] track() {
		double[][] points = new double[site.track().length][];
		for (int i = 0; i < points.length; i++) {
			points[i] = fromMap(site.track()[i][0], site.track()[i][1]);
		}
		return points;
	}

	/** Cropped paddocks the land cover does not show. */
	List<Crop> crops() {
		List<Crop> crops = new ArrayList<>();
		for (Crop crop : site.crops()) {
			double[][] corners = new double[crop.corners().length][];
			for (int i = 0; i < corners.length; i++) {
				corners[i] = fromMap(crop.corners()[i][0], crop.corners()[i][1]);
			}
			double[] circle = null;
			if (crop.circle() != null) {
				double[] centre = fromMap(crop.circle()[0], crop.circle()[1]);
				circle = new double[] { centre[0], centre[1], crop.circle()[2] };
			}
			crops.add(new Crop(crop.kind(), corners, circle));
		}
		return crops;
	}

	/** Half the width of the square round the pad that the close land cover fills, metres. */
	double nearHalf() {
		return inside(site.nearHalf());
	}

	/**
	 * Half the width of the square round the pad that the far land cover and the heights fill,
	 * metres: a whole number of 512 m, so the ground's squares meet its edge.
	 */
	double farHalf() {
		return Math.floor(inside(site.farHalf()) / 512) * 512;
	}

	/** Half the largest square in the site frame that fits in a grid of this half width. */
	private double inside(double gridHalf) {
		return (gridHalf - Math.max(Math.abs(padEast), Math.abs(padNorth))) / (Math.abs(sin) + Math.abs(cos));
	}

	/** Land cover class close to the launch area, from the finer data. */
	int nearClass(double x, double y) {
		return classAt(near, nearSize, site.nearHalf(), site.nearCell(), east(x, y), north(x, y));
	}

	/** Land cover class anywhere in the survey, from the coarser data. */
	int farClass(double x, double y) {
		return classAt(far, farSize, site.farHalf(), site.farCell(), east(x, y), north(x, y));
	}

	/** Metres east of the grids' centre of a point in the site frame. */
	private double east(double x, double y) {
		return padEast + x * sin - y * cos;
	}

	/** Metres north of the grids' centre of a point in the site frame. */
	private double north(double x, double y) {
		return padNorth + x * cos + y * sin;
	}

	/** A point given east and north of the grids' centre, in the site frame. */
	private double[] fromMap(double east, double north) {
		double de = east - padEast, dn = north - padNorth;
		return new double[] { de * sin + dn * cos, -de * cos + dn * sin };
	}

	private static int classAt(byte[] grid, int size, double half, double cell, double east, double north) {
		int x = (int) Math.floor((east + half) / cell);
		int y = (int) Math.floor((half - north) / cell);
		if (x < 0 || y < 0 || x >= size || y >= size) {
			return PADDOCK;
		}
		return grid[y * size + x] & 0xFF;
	}

	/** Height of the ground above the main pad, metres. */
	double height(double x, double y) {
		return rawHeight(east(x, y), north(x, y)) - padHeight;
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
