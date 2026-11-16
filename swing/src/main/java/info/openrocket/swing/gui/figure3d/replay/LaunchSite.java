package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import info.openrocket.swing.gui.figure3d.geometry.IntList;
import info.openrocket.swing.gui.figure3d.geometry.Mesh;
import info.openrocket.swing.gui.figure3d.geometry.Vertex;
import info.openrocket.swing.gui.figure3d.materials.Appearance3D;
import info.openrocket.swing.gui.figure3d.materials.Texture;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

/**
 * The launch site the replay flies from: a mown launch area with a line of pads and rails, the
 * launch control tent and a windsock, in a grass paddock with gum trees, a track and a dam, and
 * low hills on the horizon.
 *
 * <p>The site is laid out on its own axes: along the line of pads ("row") and from the pads
 * towards the spectators ("depth"), with the main pad at the origin. Everything is drawn unlit,
 * shaded for a fixed sun, so none of it changes how the rocket's own shadows are fitted.</p>
 */
final class LaunchSite {
	private static final Logger log = LoggerFactory.getLogger(LaunchSite.class);
	private static final float S = RenderingConstants.WORLD_SCALE;
	private static final int OUTSIDE = RenderingConstants.SURFACE_ID_OUTSIDE;
	private static final long SEED = 20_261_116L;

	/** Height of the blast deflector above the ground, metres. */
	static final double PLATE_HEIGHT = 0.42;
	/** Gap between the deflector and the rocket's aft end, metres. */
	static final double TAIL_STANDOFF = 0.05;
	/** Distance from the rail's surface to the rocket's, the height of a rail button, metres. */
	static final double BUTTON_HEIGHT = 0.006;
	/** Pads before and after the main pad along the row. */
	private static final int PADS_BEFORE = 1;
	private static final int PADS_AFTER = 2;
	/** Half width of the square of close-up grass around the pads, metres. */
	private static final double NEAR_HALF = 50;
	/** Half width of the square of close-up grass where something lands, metres. */
	private static final double LANDING_HALF = 32;
	/** Smallest ground square, metres; patches of close-up grass line up with it. */
	private static final double GRID = 2;
	/** Half width of the whole ground, metres: a power of two times the grid, so squares halve evenly. */
	private static final double GROUND_HALF = 32_768;

	// sRGB colours of the scenery
	private static final float[] RAIL = { 0.74f, 0.75f, 0.78f };
	private static final float[] PLATE = { 0.33f, 0.33f, 0.34f };
	private static final float[] STAND = { 0.20f, 0.20f, 0.22f };
	private static final float[] TENT_ROOF = { 0.05f, 0.33f, 0.60f };
	private static final float[] TENT_FRAME = { 0.82f, 0.82f, 0.82f };
	private static final float[] TABLE = { 0.90f, 0.90f, 0.88f };
	private static final float[] CONTROLLER = { 0.12f, 0.12f, 0.13f };
	private static final float[] SOCK_ORANGE = { 0.95f, 0.42f, 0.08f };
	private static final float[] SOCK_WHITE = { 0.93f, 0.93f, 0.92f };
	private static final float[] POLE = { 0.60f, 0.61f, 0.60f };
	private static final float[][] BARK = { { 0.72f, 0.69f, 0.62f }, { 0.64f, 0.60f, 0.53f }, { 0.78f, 0.76f, 0.71f },
			{ 0.58f, 0.52f, 0.44f } };
	private static final float[][] LEAVES = { { 0.33f, 0.41f, 0.23f }, { 0.30f, 0.39f, 0.27f }, { 0.38f, 0.44f, 0.25f },
			{ 0.27f, 0.35f, 0.20f }, { 0.36f, 0.42f, 0.30f }, { 0.31f, 0.37f, 0.22f } };
	private static final float[] HILLS = { 0.38f, 0.46f, 0.40f };
	private static final float[] FENCE = { 0.52f, 0.47f, 0.40f };
	private static final float[] WIRE = { 0.55f, 0.56f, 0.57f };

	/** A mesh, in site coordinates, and how to draw it. */
	record Piece(Mesh mesh, Appearance3D appearance) {
	}

	/** A gum tree: base position along row and depth, height, canopy radius and trunk radius, metres. */
	private record Tree(double r, double d, double height, double canopy, double trunk, int variant) {
	}

	private final Vector3f row;
	private final Vector3f depth;
	private final Vector3f toSun;
	private final double padSpacing;
	private final Vector3f railAxis;
	private final double railLength;
	private final double railWidth;
	private final double railOffset;
	private final double windFrom;
	private final double windSpeed;
	private final List<Tree> trees = new ArrayList<>();
	private final double[][] track;
	private final double[] dam;
	private final double[] mown;
	/** Squares of close-up grass: first r, last r, first d, last d, metres. */
	private final List<double[]> patches = new ArrayList<>();
	private final List<double[]> landings;
	private final CompletableFuture<LaunchSiteImages.Images> images;

	/**
	 * Lays out the site and starts painting its textures in the background.
	 *
	 * @param row        level unit vector along the line of pads
	 * @param depth      level unit vector from the pads towards the spectators
	 * @param toSun      unit vector towards the sun
	 * @param padSpacing distance between neighbouring pads, metres
	 * @param railAxis   unit vector up the launch rail
	 * @param railLength rail length, metres
	 * @param railWidth  width of the rail's square section, metres
	 * @param railOffset distance from the pad centre to the rail's axis, away from the spectators, metres
	 * @param windFrom   bearing the wind blows from, radians clockwise from north, or NaN for calm
	 * @param windSpeed  wind speed at the ground, m/s
	 * @param landings   where things come down, as (r, d) in metres, to get close-up grass too
	 */
	LaunchSite(Vector3f row, Vector3f depth, Vector3f toSun, double padSpacing, Vector3f railAxis, double railLength,
			double railWidth, double railOffset, double windFrom, double windSpeed, List<double[]> landings) {
		this.row = new Vector3f(row).normalize();
		this.depth = new Vector3f(depth).normalize();
		this.toSun = new Vector3f(toSun).normalize();
		this.padSpacing = padSpacing;
		this.railAxis = new Vector3f(railAxis).normalize();
		this.railLength = railLength;
		this.railWidth = railWidth;
		this.railOffset = railOffset;
		this.windFrom = windFrom;
		this.windSpeed = Double.isFinite(windSpeed) ? windSpeed : 0;

		// The mown launch area: the pads with room around them, and the way to the tent.
		double first = -PADS_BEFORE * padSpacing - 8;
		double last = PADS_AFTER * padSpacing + 8;
		this.mown = new double[] { snap(first, -1), snap(last, 1), -8, 30 };
		this.landings = new ArrayList<>();
		patches.add(new double[] { -NEAR_HALF, NEAR_HALF, -NEAR_HALF, NEAR_HALF });
		for (double[] landing : landings) {
			if (Math.abs(landing[0]) < GROUND_HALF - LANDING_HALF && Math.abs(landing[1]) < GROUND_HALF - LANDING_HALF) {
				this.landings.add(landing);
				patches.add(new double[] { snap(landing[0] - LANDING_HALF, -1), snap(landing[0] + LANDING_HALF, 1),
						snap(landing[1] - LANDING_HALF, -1), snap(landing[1] + LANDING_HALF, 1) });
			}
		}
		// A farm track across the paddock, passing the launch area's gate.
		this.track = new double[][] { { -1100, 420 }, { -760, 330 }, { -420, 240 }, { -180, 150 }, { -40, 110 },
				{ 120, 96 }, { 330, 130 }, { 600, 230 }, { 1100, 260 } };
		this.dam = new double[] { 310, -370, 24 };
		plantTrees();

		Vector2f shadow = new Vector2f(-this.toSun.dot(this.row), -this.toSun.dot(this.depth)).div(this.toSun.y);
		LaunchSiteImages.Plan plan = new LaunchSiteImages.Plan(SEED, treeShadows(), track, dam, shadow.x, shadow.y,
				NEAR_HALF + 5);
		this.images = CompletableFuture.supplyAsync(() -> LaunchSiteImages.paint(plan));
	}

	private static double snap(double value, int direction) {
		return direction < 0 ? Math.floor(value / GRID) * GRID : Math.ceil(value / GRID) * GRID;
	}

	// ---------- layout ----------

	/** Where pad {@code index} stands, engine units: 0 is the main pad, 1 the next along. */
	Vector3f padPosition(int index) {
		return site(index * padSpacing, 0, 0).mul(S);
	}

	/** Site position in metres from row, height and depth coordinates. */
	private Vector3f site(double r, double h, double d) {
		return new Vector3f(row).mul((float) r).add(new Vector3f(depth).mul((float) d)).add(0, (float) h, 0);
	}

	private void plantTrees() {
		Random random = new Random(SEED);
		// Clumps along the paddock, and single trees between them.
		double[][] clumps = { { -180, -150 }, { 140, -260 }, { 420, 60 }, { -380, -40 }, { 60, 420 }, { -520, 380 },
				{ 700, -420 }, { -760, -520 }, { 880, 300 }, { 250, 760 }, { -150, 900 }, { -950, 120 } };
		for (double[] clump : clumps) {
			int count = 3 + random.nextInt(6);
			for (int i = 0; i < count; i++) {
				addTree(random, clump[0] + random.nextGaussian() * 14, clump[1] + random.nextGaussian() * 14);
			}
		}
		for (int i = 0; i < 26; i++) {
			double angle = random.nextDouble() * 2 * Math.PI;
			double distance = 90 + random.nextDouble() * 1100;
			addTree(random, distance * Math.cos(angle), distance * Math.sin(angle));
		}
	}

	private void addTree(Random random, double r, double d) {
		if (Math.hypot(r, d) < 80 || Math.hypot(r - dam[0], d - dam[1]) < dam[2] + 12) {
			return;
		}
		double height = 9 + random.nextDouble() * 10;
		trees.add(new Tree(r, d, height, height * (0.28 + random.nextDouble() * 0.14), 0.18 + height * 0.018,
				random.nextInt(LEAVES.length)));
	}

	private List<double[]> treeShadows() {
		List<double[]> shadows = new ArrayList<>();
		for (Tree tree : trees) {
			shadows.add(new double[] { tree.r(), tree.d(), tree.height() * 0.75, tree.canopy() });
		}
		return shadows;
	}

	// ---------- meshes ----------

	/**
	 * Builds the site's meshes and textures. Call on the GL thread; waits for the textures to be
	 * painted if they are not ready yet.
	 */
	List<Piece> build() {
		LaunchSiteImages.Images painted;
		try {
			painted = images.get();
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			return List.of();
		} catch (ExecutionException e) {
			log.warn("Could not paint the launch site", e.getCause());
			return List.of();
		}
		List<Piece> pieces = new ArrayList<>();
		addGround(pieces, painted);
		addGrassBlades(pieces, painted.blades());
		addPads(pieces);
		addTent(pieces);
		addWindsock(pieces);
		addFence(pieces);
		addTrees(pieces);
		addHills(pieces);
		return pieces;
	}

	/**
	 * Lays the ground as squares that halve wherever they meet a patch of close-up grass or the
	 * edge of the paddock image, so each square carries one texture and none overlap.
	 */
	private void addGround(List<Piece> pieces, LaunchSiteImages.Images painted) {
		GroundBuilder longGrass = new GroundBuilder(1 / LaunchSiteImages.DETAIL_TILE, 0);
		GroundBuilder mownGrass = new GroundBuilder(1 / LaunchSiteImages.DETAIL_TILE, 0);
		GroundBuilder paddock = new GroundBuilder(1 / LaunchSiteImages.FIELD_SPAN, LaunchSiteImages.FIELD_SPAN / 2);
		GroundBuilder country = new GroundBuilder(1 / LaunchSiteImages.FAR_TILE, 0);
		layGround(-GROUND_HALF, -GROUND_HALF, 2 * GROUND_HALF, longGrass, mownGrass, paddock, country);
		pieces.add(new Piece(longGrass.build(), ground(painted.longGrass(), Appearance3D.TextureMode.REPEAT)));
		pieces.add(new Piece(mownGrass.build(), ground(painted.mownGrass(), Appearance3D.TextureMode.REPEAT)));
		pieces.add(new Piece(paddock.build(), ground(painted.field(), Appearance3D.TextureMode.STRETCH)));
		pieces.add(new Piece(country.build(), ground(painted.far(), Appearance3D.TextureMode.REPEAT)));
	}

	/** Ground area under each texture, square metres: long grass, mown grass, paddock, country. */
	double[] groundAreas() {
		GroundBuilder[] builders = { new GroundBuilder(1, 0), new GroundBuilder(1, 0), new GroundBuilder(1, 0),
				new GroundBuilder(1, 0) };
		layGround(-GROUND_HALF, -GROUND_HALF, 2 * GROUND_HALF, builders[0], builders[1], builders[2], builders[3]);
		return new double[] { builders[0].area, builders[1].area, builders[2].area, builders[3].area };
	}

	/** The mown launch area: first r, last r, first d, last d, metres. */
	double[] mownArea() {
		return mown.clone();
	}

	private void layGround(double r, double d, double size, GroundBuilder longGrass, GroundBuilder mownGrass,
			GroundBuilder paddock, GroundBuilder country) {
		double field = LaunchSiteImages.FIELD_SPAN / 2;
		boolean straddlesField = r < field && r + size > -field && d < field && d + size > -field
				&& !(r >= -field && r + size <= field && d >= -field && d + size <= field);
		boolean touchesPatch = false;
		boolean insidePatch = false;
		for (double[] patch : patches) {
			if (r < patch[1] && r + size > patch[0] && d < patch[3] && d + size > patch[2]) {
				touchesPatch = true;
				insidePatch |= r >= patch[0] && r + size <= patch[1] && d >= patch[2] && d + size <= patch[3];
			}
		}
		if (size > GRID && (straddlesField || touchesPatch && !insidePatch)) {
			double half = size / 2;
			layGround(r, d, half, longGrass, mownGrass, paddock, country);
			layGround(r + half, d, half, longGrass, mownGrass, paddock, country);
			layGround(r, d + half, half, longGrass, mownGrass, paddock, country);
			layGround(r + half, d + half, half, longGrass, mownGrass, paddock, country);
			return;
		}
		GroundBuilder target;
		if (insidePatch) {
			boolean isMown = r >= mown[0] && r + size <= mown[1] && d >= mown[2] && d + size <= mown[3];
			if (!isMown && size > GRID && r < mown[1] && r + size > mown[0] && d < mown[3] && d + size > mown[2]) {
				// Part mown: split until each square is one or the other.
				double half = size / 2;
				layGround(r, d, half, longGrass, mownGrass, paddock, country);
				layGround(r + half, d, half, longGrass, mownGrass, paddock, country);
				layGround(r, d + half, half, longGrass, mownGrass, paddock, country);
				layGround(r + half, d + half, half, longGrass, mownGrass, paddock, country);
				return;
			}
			target = isMown ? mownGrass : longGrass;
		} else {
			target = r >= -field && r + size <= field && d >= -field && d + size <= field ? paddock : country;
		}
		target.rectangle(r, d, r + size, d + size);
	}

	private static Appearance3D ground(ByteBuffer image, Appearance3D.TextureMode mode) {
		Appearance3D appearance = new Appearance3D(new Texture(image), Appearance3D.RenderStyle.TEXTURED);
		appearance.setUnlit(true);
		appearance.setTextureMode(mode);
		return appearance;
	}

	/** Level ground quads in site coordinates, with texture coordinates from position. */
	private final class GroundBuilder {
		private final List<Vertex> vertices = new ArrayList<>();
		private final IntList indices = new IntList();
		private final double perMetre;
		private final double offset;
		private double area;

		/** Texture coordinates are (r + offset, d + offset) times {@code perMetre}. */
		GroundBuilder(double perMetre, double offset) {
			this.perMetre = perMetre;
			this.offset = offset;
		}

		void rectangle(double r0, double d0, double r1, double d1) {
			area += (r1 - r0) * (d1 - d0);
			int base = vertices.size();
			double[][] corners = { { r0, d0 }, { r1, d0 }, { r1, d1 }, { r0, d1 } };
			for (double[] corner : corners) {
				Vector3f position = site(corner[0], 0, corner[1]).mul(S);
				Vector2f uv = new Vector2f((float) ((corner[0] + offset) * perMetre),
						(float) ((corner[1] + offset) * perMetre));
				vertices.add(new Vertex(position, new Vector3f(0, 1, 0), uv, OUTSIDE));
			}
			// Face up whichever way round row and depth turn.
			boolean upward = new Vector3f(row).cross(depth).y < 0;
			if (upward) {
				indices.addTriangle(base, base + 2, base + 1);
				indices.addTriangle(base, base + 3, base + 2);
			} else {
				indices.addTriangle(base, base + 1, base + 2);
				indices.addTriangle(base, base + 2, base + 3);
			}
		}

		Mesh build() {
			return new Mesh(vertices, indices);
		}
	}

	/** Tufts of long grass standing out of the close-up grass, around the launch area and where things land. */
	private void addGrassBlades(List<Piece> pieces, ByteBuffer image) {
		Random random = new Random(SEED + 3);
		List<Vertex> vertices = new ArrayList<>();
		IntList indices = new IntList();
		scatterTufts(random, 0, 0, NEAR_HALF - 4, 22, 26_000, 1, vertices, indices);
		// Grazed shorter out in the paddock, so what lands there stays in sight.
		for (double[] landing : landings) {
			scatterTufts(random, landing[0], landing[1], LANDING_HALF - 4, 12, 8_000, 0.45, vertices, indices);
		}
		Appearance3D appearance = new Appearance3D(new Texture(image), Appearance3D.RenderStyle.TEXTURED);
		appearance.setUnlit(true);
		pieces.add(new Piece(new Mesh(vertices, indices), appearance));
	}

	/**
	 * Scatters tufts over a square around a point: all of them within {@code thick} metres of it,
	 * thinning out to none at {@code reach}, where a tuft is too far off to make out.
	 *
	 * @param tallness height of the grass as a share of the long grass by the pads
	 */
	private void scatterTufts(Random random, double centreR, double centreD, double reach, double thick, int tries,
			double tallness, List<Vertex> vertices, IntList indices) {
		Vector3f up = new Vector3f(0, 1, 0);
		for (int i = 0; i < tries; i++) {
			double r = centreR + (random.nextDouble() * 2 - 1) * reach;
			double d = centreD + (random.nextDouble() * 2 - 1) * reach;
			double distance = Math.hypot(r - centreR, d - centreD);
			boolean inMown = r > mown[0] - 0.3 && r < mown[1] + 0.3 && d > mown[2] - 0.3 && d < mown[3] + 0.3;
			if (inMown || random.nextDouble() < LaunchSiteImages.smoothstep(thick, reach, distance)) {
				continue;
			}
			float u = (random.nextInt(LaunchSiteImages.BLADE_VARIANTS) + 0.5f) / LaunchSiteImages.BLADE_VARIANTS;
			int blades = 3 + random.nextInt(3);
			for (int b = 0; b < blades; b++) {
				double angle = random.nextDouble() * Math.PI;
				double lean = random.nextDouble() * 0.5;
				double leanAngle = random.nextDouble() * 2 * Math.PI;
				double height = (0.16 + random.nextDouble() * 0.32) * tallness;
				double halfWidth = 0.006 + random.nextDouble() * 0.006;
				double br = r + random.nextGaussian() * 0.04, bd = d + random.nextGaussian() * 0.04;
				Vector3f across = site(Math.cos(angle) * halfWidth, 0, Math.sin(angle) * halfWidth);
				Vector3f root = site(br, 0, bd);
				Vector3f tip = site(br + Math.cos(leanAngle) * lean * height, height * Math.cos(lean),
						bd + Math.sin(leanAngle) * lean * height);
				int base = vertices.size();
				vertices.add(new Vertex(new Vector3f(root).sub(across).mul(S), up, new Vector2f(u, 0.02f), OUTSIDE));
				vertices.add(new Vertex(new Vector3f(root).add(across).mul(S), up, new Vector2f(u, 0.02f), OUTSIDE));
				vertices.add(new Vertex(tip.mul(S), up, new Vector2f(u, 0.98f), OUTSIDE));
				// Both faces, as a blade is seen from either side.
				indices.addTriangle(base, base + 1, base + 2);
				indices.addTriangle(base, base + 2, base + 1);
			}
		}
	}

	/** A blast deflector on a tripod and a rail at each pad. */
	private void addPads(List<Piece> pieces) {
		ToneMeshBuilder rails = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder plates = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder stands = new ToneMeshBuilder(toSun, 1);
		float plate = (float) PLATE_HEIGHT;
		// The rail stands off the pad centre square to its own axis, away from the spectators.
		Vector3f away = new Vector3f(depth).negate();
		away.sub(new Vector3f(railAxis).mul(away.dot(railAxis))).normalize();
		Vector3f side = new Vector3f(railAxis).cross(away).normalize();
		float halfRail = (float) (railWidth / 2);
		for (int pad = -PADS_BEFORE; pad <= PADS_AFTER; pad++) {
			double r = pad * padSpacing;
			Vector3f centre = site(r, 0, 0);
			plates.levelBox(site(r, plate - 0.003, 0), row, 0.17f, 0.17f, 0.003f);

			// Tripod: a hub under the plate and three splayed legs.
			Vector3f hub = site(r, plate - 0.09, 0);
			stands.cylinder(hub, site(r, plate - 0.006, 0), 0.03f, 0.03f, 8, true);
			for (int leg = 0; leg < 3; leg++) {
				double angle = Math.PI / 2 + leg * 2 * Math.PI / 3;
				Vector3f foot = new Vector3f(centre).add(site(Math.cos(angle) * 0.55, 0, Math.sin(angle) * 0.55));
				stands.cylinder(foot, hub, 0.012f, 0.012f, 6, false);
			}

			// Rail from the deflector up, square section, with a block holding it to the plate.
			Vector3f base = site(r, plate, 0).add(new Vector3f(away).mul((float) railOffset));
			Vector3f middle = new Vector3f(base).add(new Vector3f(railAxis).mul((float) (railLength / 2)));
			rails.box(middle, new Vector3f(side).mul(halfRail), new Vector3f(railAxis).mul((float) (railLength / 2)),
					new Vector3f(away).mul(halfRail));
			stands.box(new Vector3f(base).add(new Vector3f(railAxis).mul(0.03f)),
					new Vector3f(side).mul(halfRail * 1.6f), new Vector3f(railAxis).mul(0.03f),
					new Vector3f(away).mul(halfRail * 1.6f));
		}
		pieces.add(toned(rails, RAIL));
		pieces.add(toned(plates, PLATE));
		pieces.add(toned(stands, STAND));
	}

	/** The launch control tent with its table, towards the spectators and off to one side. */
	private void addTent(List<Piece> pieces) {
		double r = -PADS_BEFORE * padSpacing - 4, d = 24;
		ToneMeshBuilder frame = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder roof = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder table = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder controller = new ToneMeshBuilder(toSun, 1);
		float half = 1.5f, eave = 2.25f, peak = 2.95f;
		for (int corner = 0; corner < 4; corner++) {
			double cr = r + ((corner & 1) == 0 ? -half : half), cd = d + ((corner & 2) == 0 ? -half : half);
			frame.levelBox(site(cr, eave / 2, cd), row, 0.018f, 0.018f, eave / 2);
		}
		Vector3f top = site(r, peak, d);
		Vector3f[] corners = { site(r - half - 0.05, eave, d - half - 0.05), site(r + half + 0.05, eave, d - half - 0.05),
				site(r + half + 0.05, eave, d + half + 0.05), site(r - half - 0.05, eave, d + half + 0.05) };
		for (int i = 0; i < 4; i++) {
			Vector3f a = corners[i], b = corners[(i + 1) % 4];
			// Roof panels face out and down from inside; the valance hangs below the eave.
			roof.twoSidedQuad(a, b, top, top);
			Vector3f below = new Vector3f(0, -0.25f, 0);
			roof.twoSidedQuad(a, b, new Vector3f(b).add(below), new Vector3f(a).add(below));
		}
		table.levelBox(site(r, 0.72, d - 0.4), row, 0.9f, 0.37f, 0.02f);
		for (int leg = 0; leg < 4; leg++) {
			double lr = r + ((leg & 1) == 0 ? -0.82 : 0.82), ld = d - 0.4 + ((leg & 2) == 0 ? -0.3 : 0.3);
			table.levelBox(site(lr, 0.35, ld), row, 0.015f, 0.015f, 0.35f);
		}
		controller.levelBox(site(r - 0.3, 0.80, d - 0.45), row, 0.13f, 0.08f, 0.06f);
		controller.levelBox(site(r + 0.35, 0.84, d - 0.35), row, 0.11f, 0.09f, 0.10f);
		pieces.add(toned(frame, TENT_FRAME));
		pieces.add(toned(roof, TENT_ROOF));
		pieces.add(toned(table, TABLE));
		pieces.add(toned(controller, CONTROLLER));
	}

	/** A windsock past the end of the pads, streaming downwind and drooping as the wind drops. */
	private void addWindsock(List<Piece> pieces) {
		double r = PADS_AFTER * padSpacing + 6, d = -5;
		float height = 4.5f;
		ToneMeshBuilder pole = new ToneMeshBuilder(toSun, 1);
		pole.cylinder(site(r, 0, d), site(r, height + 0.1, d), 0.04f, 0.03f, 8, true);
		pieces.add(toned(pole, POLE));

		double bearing = Double.isNaN(windFrom) ? Math.PI / 2 : windFrom + Math.PI;
		Vector3f downwind = new Vector3f((float) Math.sin(bearing), 0, (float) -Math.cos(bearing));
		double droop = Math.toRadians(75) * (1 - Math.min(1, windSpeed / 8));
		Vector3f along = new Vector3f(downwind).mul((float) Math.cos(droop)).add(0, (float) -Math.sin(droop), 0);
		Vector3f mouth = site(r, height, d).add(new Vector3f(downwind).mul(0.06f));
		ToneMeshBuilder orange = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder white = new ToneMeshBuilder(toSun, 1);
		int bands = 5;
		float length = 1.3f;
		for (int band = 0; band < bands; band++) {
			Vector3f from = new Vector3f(mouth).add(new Vector3f(along).mul(length * band / bands));
			Vector3f to = new Vector3f(mouth).add(new Vector3f(along).mul(length * (band + 1) / bands));
			float r0 = 0.19f - 0.11f * band / bands, r1 = 0.19f - 0.11f * (band + 1) / bands;
			ToneMeshBuilder target = band % 2 == 0 ? orange : white;
			target.sleeve(from, to, r0, r1, 12);
		}
		pieces.add(toned(orange, SOCK_ORANGE));
		pieces.add(toned(white, SOCK_WHITE));
	}

	/** Gum trees: pale trunks forking into a few loose clumps of leaves. */
	private void addTrees(List<Piece> pieces) {
		Random random = new Random(SEED + 5);
		ToneMeshBuilder bark = new ToneMeshBuilder(toSun, BARK.length);
		ToneMeshBuilder leaves = new ToneMeshBuilder(toSun, LEAVES.length);
		for (Tree tree : trees) {
			Vector3f base = site(tree.r(), 0, tree.d());
			float h = (float) tree.height(), trunk = (float) tree.trunk(), canopy = (float) tree.canopy();
			bark.variant(random.nextInt(BARK.length));
			leaves.variant(tree.variant());
			Vector3f fork = new Vector3f(base).add((float) random.nextGaussian() * 0.3f, h * 0.45f,
					(float) random.nextGaussian() * 0.3f);
			bark.cylinder(new Vector3f(base).add(0, -0.2f, 0), fork, trunk, trunk * 0.7f, 7, false);
			int clumps = 3 + random.nextInt(3);
			for (int c = 0; c < clumps; c++) {
				double angle = 2 * Math.PI * (c + random.nextDouble() * 0.6) / clumps;
				float reach = canopy * (0.35f + 0.45f * random.nextFloat());
				Vector3f centre = new Vector3f(base).add((float) Math.cos(angle) * reach,
						h * (0.68f + 0.2f * random.nextFloat()), (float) Math.sin(angle) * reach);
				bark.cylinder(fork, new Vector3f(centre).add(0, -canopy * 0.2f, 0), trunk * 0.5f, trunk * 0.2f, 5, false);
				float size = canopy * (0.5f + 0.25f * random.nextFloat());
				leaves.blob(centre, size, size * (0.45f + 0.15f * random.nextFloat()), size * (0.85f + 0.3f * random.nextFloat()),
						9, 6);
			}
		}
		pieces.add(toned(bark, BARK));
		pieces.add(toned(leaves, LEAVES));
	}

	/** Low hills on the horizon, far enough out that the haze softens them. */
	private void addHills(List<Piece> pieces) {
		Random random = new Random(SEED + 9);
		int segments = 240;
		double[] phases = new double[6];
		for (int i = 0; i < phases.length; i++) {
			phases[i] = random.nextDouble() * 2 * Math.PI;
		}
		double[] radii = { 5_000, 6_500, 8_000, 10_000 };
		double[] heights = { 0, 0.6, 1, 0.2 };
		ToneMeshBuilder hills = new ToneMeshBuilder(toSun, 1);
		Vector3f[][] grid = new Vector3f[radii.length][segments + 1];
		for (int i = 0; i <= segments; i++) {
			double angle = 2 * Math.PI * i / segments;
			double crest = 20;
			for (int k = 0; k < phases.length; k++) {
				crest += 60.0 / (k + 1) * (0.5 + 0.5 * Math.sin((k + 2) * angle + phases[k]));
			}
			for (int j = 0; j < radii.length; j++) {
				grid[j][i] = new Vector3f((float) (Math.cos(angle) * radii[j]), (float) (crest * heights[j]),
						(float) (Math.sin(angle) * radii[j]));
			}
		}
		for (int j = 0; j + 1 < radii.length; j++) {
			for (int i = 0; i < segments; i++) {
				// Faces towards the site, which sits inside the ring.
				hills.quad(grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]);
			}
		}
		pieces.add(toned(hills, HILLS));
	}

	/**
	 * A post and wire fence round the launch area's paddock, with a gate facing the track, where
	 * the close-up grass gives way to the paddock beyond.
	 */
	private void addFence(List<Piece> pieces) {
		ToneMeshBuilder posts = new ToneMeshBuilder(toSun, 1);
		ToneMeshBuilder wires = new ToneMeshBuilder(toSun, 1);
		double edge = NEAR_HALF - 0.5;
		double spacing = 3.3;
		double[][] corners = { { -edge, -edge }, { edge, -edge }, { edge, edge }, { -edge, edge } };
		for (int side = 0; side < 4; side++) {
			double[] a = corners[side], b = corners[(side + 1) % 4];
			double length = Math.hypot(b[0] - a[0], b[1] - a[1]);
			int count = (int) Math.round(length / spacing);
			Vector3f along = site(b[0] - a[0], 0, b[1] - a[1]).normalize();
			for (int k = 0; k < count; k++) {
				double t0 = (double) k / count, t1 = (double) (k + 1) / count;
				double r0 = a[0] + (b[0] - a[0]) * t0, d0 = a[1] + (b[1] - a[1]) * t0;
				double r1 = a[0] + (b[0] - a[0]) * t1, d1 = a[1] + (b[1] - a[1]) * t1;
				posts.levelBox(site(r0, 0.6, d0), along, 0.04f, 0.04f, 0.6f);
				// The gate: a gap in the side towards the spectators, opposite the pads.
				boolean gate = side == 2 && Math.abs(0.5 * (r0 + r1) - 20) < 2;
				if (gate) {
					continue;
				}
				for (double h : new double[] { 0.55, 0.95 }) {
					Vector3f from = site(r0, h, d0), to = site(r1, h, d1);
					Vector3f middle = new Vector3f(from).add(to).mul(0.5f);
					wires.levelBox(middle, along, (float) (from.distance(to) / 2), 0.004f, 0.004f);
				}
			}
		}
		pieces.add(toned(posts, FENCE));
		pieces.add(toned(wires, WIRE));
	}

	private static Piece toned(ToneMeshBuilder builder, float[]... variants) {
		Appearance3D appearance = new Appearance3D(new Texture(LaunchSiteImages.ramp(variants)),
				Appearance3D.RenderStyle.TEXTURED);
		appearance.setUnlit(true);
		return new Piece(builder.build(), appearance);
	}
}
