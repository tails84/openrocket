package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.appearance.Appearance;
import info.openrocket.core.motor.MotorConfiguration;
import info.openrocket.core.rocketcomponent.Parachute;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.SymmetricComponent;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.util.BoundingBox;
import info.openrocket.core.util.CoordinateIF;
import info.openrocket.core.util.ORColor;
import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import info.openrocket.swing.gui.figure3d.geometry.Mesh;
import info.openrocket.swing.gui.figure3d.materials.Appearance3D;
import info.openrocket.swing.gui.figure3d.particles.ParticleEmitter;
import info.openrocket.swing.gui.figure3d.particles.flame.FlameEmitter;
import info.openrocket.swing.gui.figure3d.particles.smoke.SmokeEmitter;
import info.openrocket.swing.gui.figure3d.rendering.backgrounds.GradientBackground;
import info.openrocket.swing.gui.figure3d.scene.graph.Camera;
import info.openrocket.swing.gui.figure3d.scene.graph.Light;
import info.openrocket.swing.gui.figure3d.scene.graph.Scene;
import info.openrocket.swing.gui.figure3d.scene.graph.SceneObject;
import info.openrocket.swing.gui.figure3d.scene.orchestration.Scene3DOrchestrator;
import info.openrocket.swing.gui.figure3d.scene.properties.DisplaySettings;
import info.openrocket.swing.gui.figure3d.scene.properties.RenderingConfiguration;
import info.openrocket.swing.gui.figure3d.scene.properties.VisualEffectsSettings;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

/**
 * Poses the rocket and its surroundings for one moment of a replayed flight. Runs on the GL thread.
 *
 * <p>The main rocket stays at the world origin and the ground, launch rail and trails move past it.
 * Keeping it at the origin keeps its shadows, outlines and depth precision exactly as in the other
 * 3D views however high it flies. Stages that separate follow their own flight data branch, and
 * recovery deployments split each body into sections hanging on shock cords (see
 * {@link ReplayBody}).</p>
 */
final class FlightReplayScene {

	/** Where the replay is watched from. */
	enum CameraMode {
		/** Orbits the rocket and follows it; the user can drag and zoom. */
		CHASE,
		/** A spectator at the launch site tracking the rocket with a zoom lens. */
		GROUND,
		/** Mounted on the body tube, looking aft. */
		ONBOARD
	}

	/** Engine units per metre. */
	private static final float S = RenderingConstants.WORLD_SCALE;
	private static final int TRAIL_MAX_SEGMENTS = 500;
	private static final double TRAIL_MIN_STEP = 0.05;
	private static final float TRAIL_WIDTH_PIXELS = 1.6f;
	private static final float CORD_WIDTH_PIXELS = 1.2f;
	private static final float CORD_MIN_RADIUS = 0.005f * S;
	private static final double CANOPY_INFLATION_TIME = 0.8;
	private static final double OBSERVER_EYE_HEIGHT = 1.7;
	private static final float CHASE_FOV = (float) Math.toRadians(40);
	private static final float ONBOARD_FOV = (float) Math.toRadians(75);
	private static final double GROUND_FRAME_FILL = 0.15;
	private static final double EPSILON = 1e-4;

	// sRGB colours. Trail and canopy use the Okabe-Ito palette.
	private static final Vector3f SKY_TOP = new Vector3f(0.34f, 0.55f, 0.84f);
	private static final Vector3f SKY_HAZE = new Vector3f(0.80f, 0.86f, 0.92f);
	private static final Vector3f GRASS_LIGHT = new Vector3f(0.44f, 0.58f, 0.31f);
	private static final Vector3f GRASS_DARK = new Vector3f(0.37f, 0.50f, 0.26f);
	private static final Vector3f CONCRETE = new Vector3f(0.62f, 0.62f, 0.60f);
	private static final Vector3f RAIL_METAL = new Vector3f(0.78f, 0.79f, 0.81f);
	private static final Vector3f TRAIL_ASCENT = new Vector3f(0.902f, 0.624f, 0.0f);	// orange
	private static final Vector3f TRAIL_DESCENT = new Vector3f(0.337f, 0.706f, 0.914f);	// sky blue
	private static final Vector3f CANOPY_DEFAULT = new Vector3f(0.835f, 0.369f, 0.0f);	// vermillion
	private static final Vector3f CANOPY_SECOND = new Vector3f(0.95f, 0.95f, 0.93f);
	private static final Vector3f SHROUD_LINE = new Vector3f(0.25f, 0.25f, 0.25f);

	/** Which body and section a rocket part moves with. */
	record Part(int body, int section, RecoveryDevice device) {
	}

	/** A motor's exhaust: where it sits, what it moves with and when it burns and smokes. */
	private record Exhaust(Vector3f position, Vector3f direction, Part part, List<double[]> burns,
			List<double[]> smoke) {
	}

	private record TrailSegment(SceneObject object, int body, double startTime, double endTime,
			Vector3f start, Vector3f end) {
	}

	private record Canopy(int body, int deployment, double deployTime, SceneObject[] parts) {
	}

	private final List<ReplayBody> bodies = new ArrayList<>();
	private final ReplayBody main;
	private final Map<UUID, Integer> bodyBySource = new HashMap<>();
	/** Motor mounts holding a motor in the configuration being shown. */
	private final Set<RocketComponent> motorMounts = Collections.newSetFromMap(new IdentityHashMap<>());
	private final double length;
	private final double bodyRadius;
	private final double launchCg;
	private final double launchCgHeight;
	private final Vector3f launchAxis;
	private final double railLength;
	private final double launchLatitude;

	private final Map<SceneObject, Matrix4f> baseTransforms = new IdentityHashMap<>();
	private final Map<SceneObject, Part> parts = new IdentityHashMap<>();
	private final Map<ParticleEmitter, Exhaust> exhausts = new IdentityHashMap<>();
	private final List<SceneObject> siteObjects = new ArrayList<>();
	private final List<TrailSegment> trail = new ArrayList<>();
	private final List<Canopy> canopies = new ArrayList<>();
	private final List<List<SceneObject>> cordPools = new ArrayList<>();

	private CameraMode appliedMode;
	private ReplayBody followedBody;
	private float groundFov = (float) Math.toRadians(20);

	/**
	 * @param branches       flight data branches; the first is the main rocket, the others are
	 *                       stages that separated from it
	 * @param rocket         the simulated rocket, in the configuration the 3D view shows
	 * @param railLength     launch rail length, metres
	 * @param launchLatitude launch site latitude, degrees
	 */
	FlightReplayScene(List<FlightReplayData> branches, Rocket rocket, double railLength, double launchLatitude) {
		this.railLength = railLength;
		this.launchLatitude = launchLatitude;
		FlightReplayData mainData = branches.get(0);
		for (int i = 1; i < branches.size(); i++) {
			if (branches.get(i).getSourceComponentId() != null) {
				bodyBySource.put(branches.get(i).getSourceComponentId(), i);
			}
		}

		for (MotorConfiguration motor : rocket.getSelectedConfiguration().getAllMotors()) {
			if (motor.getMount() instanceof RocketComponent mount) {
				motorMounts.add(mount);
			}
		}

		BoundingBox bounds = rocket.getSelectedConfiguration().getBoundingBox();
		this.length = bounds.max.getX() - bounds.min.getX();

		// Extent and radius of each body
		int count = branches.size();
		double[] fronts = new double[count];
		double[] afts = new double[count];
		double[] radii = new double[count];
		Arrays.fill(fronts, Double.MAX_VALUE);
		Arrays.fill(afts, -Double.MAX_VALUE);
		for (RocketComponent component : rocket) {
			if (component instanceof Rocket) {
				continue;
			}
			int body = ownerOf(component);
			fronts[body] = Math.min(fronts[body], frontOf(component));
			afts[body] = Math.max(afts[body], frontOf(component) + component.getLength());
			if (component instanceof SymmetricComponent symmetric) {
				radii[body] = Math.max(radii[body], symmetric.getMaxRadius());
			}
		}
		this.bodyRadius = radii[0] > 0 ? radii[0] : 0.02;

		double cg = mainData.getValue(FlightDataType.TYPE_CG_LOCATION, mainData.getStartTime());
		this.launchCg = Double.isFinite(cg) ? cg : bounds.min.getX() + 0.5 * length;
		Vector3f axis = mainData.getPose().getAxisDirection(mainData.getStartTime());
		this.launchAxis = axis != null ? axis : new Vector3f(0, 1, 0);
		// The aft end rests on the pad: the CG sits this high above the ground at launch.
		this.launchCgHeight = (bounds.max.getX() - launchCg) * launchAxis.y + 0.02;

		for (int i = 0; i < count; i++) {
			FlightReplayData data = branches.get(i);
			double front = fronts[i] < afts[i] ? fronts[i] : bounds.min.getX();
			double aft = fronts[i] < afts[i] ? afts[i] : bounds.max.getX();
			List<ReplayBody.DeviceJoint> devices = new ArrayList<>();
			for (FlightReplayData.Event deployment : data.getDeployments()) {
				RecoveryDevice device = (RecoveryDevice) deployment.source();
				if (ownerOf(device) == i) {
					devices.add(new ReplayBody.DeviceJoint(deployment.time(), device, stationOf(device, front, aft)));
				}
			}
			bodies.add(new ReplayBody(data, front, aft, radii[i] > 0 ? radii[i] : bodyRadius, devices,
					launchCgHeight, launchAxis));
		}
		this.main = bodies.get(0);
	}

	// ---------- rocket geometry ----------

	/** Index of the body a component flies with after any stage separation. */
	int ownerOf(RocketComponent component) {
		for (RocketComponent c = component; c != null; c = c.getParent()) {
			Integer body = bodyBySource.get(c.getID());
			if (body != null) {
				return body;
			}
		}
		return 0;
	}

	private static double frontOf(RocketComponent component) {
		CoordinateIF[] locations = component.getComponentLocations();
		double front = Double.MAX_VALUE;
		for (CoordinateIF location : locations) {
			front = Math.min(front, location.getX());
		}
		return locations.length == 0 ? 0 : front;
	}

	/**
	 * Where a recovery device opens the airframe: the forward end of the tube it is packed in, or the
	 * aft end when that tube is the front of the body, like a nose cone.
	 */
	static double stationOf(RecoveryDevice device, double bodyFront, double bodyAft) {
		RocketComponent tube = device.getParent();
		while (tube != null && !(tube instanceof SymmetricComponent)) {
			tube = tube.getParent();
		}
		if (tube == null) {
			return bodyFront;
		}
		double tubeFront = frontOf(tube);
		double tubeAft = tubeFront + tube.getLength();
		if (tubeFront <= bodyFront + EPSILON && tubeAft < bodyAft - EPSILON) {
			return tubeAft;
		}
		return tubeFront;
	}

	/** Which body and section a rocket component moves with. */
	Part partOf(RocketComponent component) {
		int body = ownerOf(component);
		double middle = frontOf(component) + 0.5 * component.getLength();
		RecoveryDevice device = component instanceof RecoveryDevice recovery ? recovery : null;
		return new Part(body, bodies.get(body).sectionOf(middle), device);
	}

	List<ReplayBody> getBodies() {
		return bodies;
	}

	// ---------- set-up ----------

	/** Configures rendering and adds the surroundings to the scene. Call once, on the GL thread. */
	void install(Scene3DOrchestrator orchestrator, boolean hasMotors) {
		RenderingConfiguration config = orchestrator.getRenderingConfiguration();
		config.getDisplay().setMode(DisplaySettings.RenderMode.FINISHED);
		config.getDisplay().setRenderInternalSurfaces(true);
		config.getQuality().setBackfaceCullingEnabled(true);
		config.getQuality().setShadowsEnabled(true);
		config.getQuality().setAmbientOcclusionEnabled(true);
		VisualEffectsSettings effects = config.getVisualEffects();
		effects.setCaretsVisible(false);
		effects.setRotateRocketOnDrag(false);
		effects.setMotionBlurEnabled(false);
		effects.setAmbientLightFactor(0.35f);
		effects.setParticleEffectsEnabled(hasMotors);
		effects.setFlameParticlesEnabled(hasMotors);
		effects.setSmokeParticlesEnabled(hasMotors);
		effects.setSparkParticlesEnabled(false);
		// Particles must run live rather than as a precomputed snapshot.
		effects.setStaticParticles(false);
		// Upper stages light in flight, so they need exhaust too.
		effects.setExhaustOnAllMotors(true);
		// Same exhaust look as the Photo Studio defaults.
		effects.setFlameColor(new Vector3f(1.0f, 100 / 255f, 50 / 255f));
		effects.setSmokeColor(new Vector3f(230 / 255f, 230 / 255f, 230 / 255f));
		effects.setSmokeOpacity(0.4f);
		effects.setParticleLengthScale(0.82f);
		effects.setSmokeLengthScale(0.5f);
		effects.setFlameExposureScale(0.9f);
		config.notifyListeners();
		orchestrator.rebuildRocketScene(false);

		Scene scene = orchestrator.getScene();
		scene.setBackground(new GradientBackground(SKY_TOP, SKY_HAZE));
		scene.setFogEnabled(true);
		scene.setFogDensity(5.0e-6f);
		configureSun(scene);
		configureCamera(orchestrator);

		addSiteObject(scene, ReplayMeshes.groundTiles(0), GRASS_LIGHT);
		addSiteObject(scene, ReplayMeshes.groundTiles(1), GRASS_DARK);
		addPadAndRail(scene);
		Mesh cylinder = ReplayMeshes.unitCylinder(6);
		for (int i = 0; i < bodies.size(); i++) {
			addTrail(scene, cylinder, i);
			addCanopies(scene, i);
			addCords(scene, cylinder, i);
		}
	}

	private void configureSun(Scene scene) {
		if (scene.getLightController().getLights().isEmpty()) {
			return;
		}
		// Mid-morning sun on the equator side of the launch site, 50 degrees up.
		double elevation = Math.toRadians(50);
		double bearing = Math.toRadians(launchLatitude < 0 ? 30 : 150);
		Vector3f toSun = new Vector3f(
				(float) (Math.cos(elevation) * Math.sin(bearing)),
				(float) Math.sin(elevation),
				(float) (-Math.cos(elevation) * Math.cos(bearing)));
		Light sun = scene.getLightController().getLight(0);
		sun.setDirection(-toSun.x, -toSun.y, -toSun.z);
		sun.setColor(1.0f, 0.97f, 0.92f);
	}

	private void configureCamera(Scene3DOrchestrator orchestrator) {
		Camera camera = orchestrator.getScene().getCamera();
		// Reach the ground from several kilometres up; the near plane still scales with distance.
		camera.setClippingPlanes(1.0e7f, 1.0e6f);
		camera.setZoomLimits(0.01f, 1.0e7f);
		camera.setPitchClampingEnabled(false);
		camera.setForceFixedUp(false);
		orchestrator.getCameraController().setZoomFitting(false);
	}

	private void addPadAndRail(Scene scene) {
		Vector3f tail = new Vector3f(0, (float) (launchCgHeight * S), 0)
				.sub(new Vector3f(launchAxis).mul((float) (length - launchCg) * S));
		SceneObject plate = addSiteObject(scene, ReplayMeshes.disc((float) Math.max(0.25, 3 * bodyRadius) * S, 32),
				CONCRETE);
		plate.getModelMatrix().translate(tail.x, 0.03f * S, tail.z);

		// The rail stands on the far side of the rocket from the default chase view.
		SceneObject rail = addSiteObject(scene, ReplayMeshes.unitCylinder(8), RAIL_METAL);
		Vector3f railBase = new Vector3f(tail.x, 0, tail.z - (float) (bodyRadius + 0.012) * S);
		rail.getModelMatrix()
				.translate(railBase)
				.rotate(new Quaternionf().rotateTo(new Vector3f(0, 1, 0), launchAxis))
				.scale(0.01f * S, (float) Math.max(railLength, 0.1) * S, 0.01f * S);
	}

	private void addTrail(Scene scene, Mesh cylinder, int index) {
		ReplayBody body = bodies.get(index);
		FlightReplayData data = body.getData();
		Appearance3D ascent = unlit(TRAIL_ASCENT);
		Appearance3D descent = unlit(TRAIL_DESCENT);
		double start = index == 0 ? data.getLiftoffTime() : body.getSeparationTime();
		if (Double.isNaN(start)) {
			start = data.getStartTime();
		}
		double end = data.getEndTime();
		if (end <= start) {
			return;
		}
		double step = Math.max(TRAIL_MIN_STEP, (end - start) / TRAIL_MAX_SEGMENTS);
		double apogee = data.getApogeeTime();
		Vector3f previous = sitePivot(body, start);
		for (double t0 = start; t0 < end; t0 += step) {
			double t1 = Math.min(end, t0 + step);
			Vector3f next = sitePivot(body, t1);
			boolean descending = !Double.isNaN(apogee) && t0 >= apogee;
			SceneObject segment = new SceneObject(cylinder, new Vector3f(), descending ? descent : ascent);
			segment.setSelectable(false);
			hide(segment);
			scene.addObject(segment);
			trail.add(new TrailSegment(segment, index, t0, t1, previous, next));
			previous = next;
		}
	}

	private static Vector3f sitePivot(ReplayBody body, double t) {
		body.update(t);
		return new Vector3f(body.getSitePivot());
	}

	private void addCanopies(Scene scene, int index) {
		ReplayBody body = bodies.get(index);
		List<RecoveryDevice> devices = body.getDeployedDevices();
		for (int d = 0; d < devices.size(); d++) {
			if (!(devices.get(d) instanceof Parachute parachute)) {
				continue;	// Streamers are not drawn.
			}
			ReplayMeshes.CanopyShape shape = ReplayMeshes.CanopyShape.of(parachute.getDiameter(),
					parachute.getLineLength(), parachute.getLineCount());
			Vector3f primary = canopyColor(parachute);
			SceneObject goresA = new SceneObject(ReplayMeshes.canopy(shape, 0), new Vector3f(), lit(primary));
			SceneObject goresB = new SceneObject(ReplayMeshes.canopy(shape, 1), new Vector3f(), lit(CANOPY_SECOND));
			SceneObject lines = new SceneObject(ReplayMeshes.shroudLines(shape, 0.004f * S), new Vector3f(),
					unlit(SHROUD_LINE));
			SceneObject[] canopyParts = { goresA, goresB, lines };
			for (SceneObject part : canopyParts) {
				part.setSelectable(false);
				hide(part);
				scene.addObject(part);
			}
			canopies.add(new Canopy(index, d, body.deploymentTime(parachute), canopyParts));
		}
	}

	private void addCords(Scene scene, Mesh cylinder, int index) {
		List<SceneObject> pool = new ArrayList<>();
		Appearance3D appearance = unlit(SHROUD_LINE);
		for (int i = 0; i < bodies.get(index).getSectionCount() + 1; i++) {
			SceneObject cord = new SceneObject(cylinder, new Vector3f(), appearance);
			cord.setSelectable(false);
			hide(cord);
			scene.addObject(cord);
			pool.add(cord);
		}
		cordPools.add(pool);
	}

	private static Vector3f canopyColor(RecoveryDevice device) {
		Appearance appearance = device.getAppearance();
		ORColor paint = appearance != null ? appearance.getPaint() : null;
		if (paint == null) {
			paint = device.getColor();
		}
		if (paint == null) {
			return new Vector3f(CANOPY_DEFAULT);
		}
		return new Vector3f(paint.getRed() / 255f, paint.getGreen() / 255f, paint.getBlue() / 255f);
	}

	private SceneObject addSiteObject(Scene scene, Mesh mesh, Vector3f color) {
		SceneObject object = new SceneObject(mesh, new Vector3f(), unlit(color));
		object.setSelectable(false);
		scene.addObject(object);
		siteObjects.add(object);
		return object;
	}

	private static Appearance3D unlit(Vector3f color) {
		Appearance3D appearance = new Appearance3D(color);
		appearance.setUnlit(true);
		return appearance;
	}

	private static Appearance3D lit(Vector3f color) {
		Appearance3D appearance = new Appearance3D(color);
		appearance.setShine(0.15f);
		return appearance;
	}

	// ---------- per frame ----------

	/** Poses everything for flight time {@code t}. Call on the GL thread before rendering. */
	void apply(Scene3DOrchestrator orchestrator, double t, CameraMode mode, int follow, float frameSeconds) {
		Scene scene = orchestrator.getScene();
		if (scene == null) {
			return;
		}
		// The rail and pad were placed when they were added; capture them before the first move.
		for (SceneObject object : siteObjects) {
			baseTransforms.computeIfAbsent(object, o -> new Matrix4f(o.getModelMatrix()));
		}
		for (ReplayBody body : bodies) {
			body.update(t);
		}
		Vector3f origin = new Vector3f(main.getSitePivot());

		applyRocketParts(scene, t, origin);
		applyEmitters(scene, t, origin);

		Matrix4f site = new Matrix4f().translation(-origin.x, -origin.y, -origin.z);
		for (SceneObject object : siteObjects) {
			site.mul(baseTransforms.get(object), object.getModelMatrix());
		}

		Camera camera = scene.getCamera();
		ReplayBody followed = bodies.get(Math.max(0, Math.min(bodies.size() - 1, follow)));
		// A stage follows the main rocket until it separates.
		if (followed != main && !(t >= followed.getSeparationTime())) {
			followed = main;
		}
		applyCamera(orchestrator, camera, mode, followed, origin, frameSeconds);
		float pixelAngle = (float) (2 * Math.tan(camera.getFieldOfView() / 2)
				/ Math.max(1, orchestrator.getViewport().getWindowHeight()));
		applyTrail(camera, pixelAngle, t, origin);
		applyCanopies(t, origin);
		applyCords(camera, pixelAngle, origin);
	}

	/**
	 * Transform from rocket coordinates to the replay world for a part at time {@code t}. A stage
	 * moves with the main rocket until it separates.
	 */
	Matrix4f partTransform(Part part, double t, Vector3f origin) {
		ReplayBody body = bodies.get(part.body());
		if (part.body() != 0 && !(t >= body.getSeparationTime())) {
			Vector3f offset = new Vector3f(main.getSitePivot()).sub(origin);
			return new Matrix4f().translation(offset).mul(main.rigidMatrix(t));
		}
		Vector3f offset = new Vector3f(body.getSitePivot()).sub(origin);
		return new Matrix4f().translation(offset).mul(body.getSectionMatrix(part.section()));
	}

	private void applyRocketParts(Scene scene, double t, Vector3f origin) {
		Matrix4f sceneRotationInverse = scene.getRocketRotationTransform(new Matrix4f()).invert();
		// Forget objects a rocket rebuild has replaced.
		Set<SceneObject> live = Collections.newSetFromMap(new IdentityHashMap<>());
		live.addAll(scene.getObjects());
		parts.keySet().retainAll(live);
		baseTransforms.keySet().removeIf(o -> o.getRocketComponent() != null && !live.contains(o));
		Map<Part, Matrix4f> transforms = new HashMap<>();
		for (SceneObject object : scene.getObjects()) {
			RocketComponent component = object.getRocketComponent();
			if (component == null) {
				continue;
			}
			Matrix4f base = baseTransforms.computeIfAbsent(object,
					o -> sceneRotationInverse.mul(o.getModelMatrix(), new Matrix4f()));
			Part part = parts.computeIfAbsent(object, o -> partOf(component));
			// A packed recovery device leaves the airframe when it deploys.
			if (part.device() != null && t >= bodies.get(part.body()).deploymentTime(part.device())) {
				hide(object);
				continue;
			}
			Matrix4f transform = transforms.computeIfAbsent(new Part(part.body(), part.section(), null),
					p -> partTransform(p, t, origin));
			transform.mul(base, object.getModelMatrix());
		}
	}

	private void applyEmitters(Scene scene, double t, Vector3f origin) {
		exhausts.keySet().retainAll(scene.getParticleEmitters());
		for (ParticleEmitter emitter : scene.getParticleEmitters()) {
			Exhaust exhaust = exhausts.computeIfAbsent(emitter, e -> exhaustFor(e, scene));
			Matrix4f transform = partTransform(exhaust.part(), t, origin);
			emitter.setEmitterPosition(transform.transformPosition(exhaust.position(), new Vector3f()));
			emitter.setDirection(transform.transformDirection(exhaust.direction(), new Vector3f()).normalize());

			boolean flame = within(exhaust.burns(), t);
			boolean on = emitter instanceof FlameEmitter ? flame
					: emitter instanceof SmokeEmitter && (flame || within(exhaust.smoke(), t));
			if (on && emitter.isStaticMode()) {
				emitter.setStaticMode(false);
			} else if (!on && !emitter.isStaticMode()) {
				emitter.getParticles().clear();
				emitter.setStaticMode(true);
			}
		}
	}

	/** Ties an emitter to the motor mount it sits in, and works out when that motor burns. */
	private Exhaust exhaustFor(ParticleEmitter emitter, Scene scene) {
		Vector3f position = new Vector3f(emitter.getEmitterPosition());
		Vector3f direction = new Vector3f(emitter.getDirection());
		RocketComponent mount = null;
		float nearest = Float.MAX_VALUE;
		for (SceneObject object : scene.getObjects()) {
			if (!motorMounts.contains(object.getRocketComponent()) || !baseTransforms.containsKey(object)) {
				continue;
			}
			float distance = distanceToBounds(object.getMesh(), baseTransforms.get(object), position);
			if (distance < nearest) {
				nearest = distance;
				mount = object.getRocketComponent();
			}
		}
		Part part = mount != null ? partOf(mount) : new Part(0, main.getSectionCount() - 1, null);
		ReplayBody body = bodies.get(part.body());
		FlightReplayData data = body.getData();

		List<double[]> burns = new ArrayList<>();
		List<double[]> smoke = new ArrayList<>();
		double ignition = Double.NaN;
		for (FlightReplayData.Event event : data.getAllEvents()) {
			if (mount == null || event.source() != mount) {
				continue;
			}
			if (event.type() == FlightEvent.Type.IGNITION) {
				ignition = event.time();
			} else if (event.type() == FlightEvent.Type.BURNOUT && !Double.isNaN(ignition)) {
				burns.add(new double[] { ignition, event.time() });
				ignition = Double.NaN;
			}
		}
		if (!Double.isNaN(ignition)) {
			burns.add(new double[] { ignition, Double.MAX_VALUE });
		}
		if (burns.isEmpty()) {
			// No motor events to go on: burn whenever the branch records thrust.
			burns.addAll(thrustSpans(data));
		}
		// A delay grain keeps smoking from burnout until its ejection charge.
		double stopSmoke = Math.min(nanTo(body.getFirstDeploymentTime(), Double.MAX_VALUE),
				nanTo(data.getGroundHitTime(), Double.MAX_VALUE));
		for (double[] burn : burns) {
			for (FlightReplayData.Event event : data.getAllEvents()) {
				if (event.type() == FlightEvent.Type.EJECTION_CHARGE && mount != null
						&& event.source() == mount.getStage() && event.time() > burn[1] + EPSILON) {
					smoke.add(new double[] { burn[1], Math.min(event.time(), stopSmoke) });
					break;
				}
			}
		}
		return new Exhaust(position, direction, part, burns, smoke);
	}

	private static List<double[]> thrustSpans(FlightReplayData data) {
		List<double[]> spans = new ArrayList<>();
		double start = Double.NaN;
		for (double t : data.getTimes()) {
			boolean thrusting = data.isThrusting(t);
			if (thrusting && Double.isNaN(start)) {
				start = t;
			} else if (!thrusting && !Double.isNaN(start)) {
				spans.add(new double[] { start, t });
				start = Double.NaN;
			}
		}
		if (!Double.isNaN(start)) {
			spans.add(new double[] { start, Double.MAX_VALUE });
		}
		return spans;
	}

	private static float distanceToBounds(Mesh mesh, Matrix4f transform, Vector3f point) {
		Vector3f low = mesh.getBoundsMin(new Vector3f());
		Vector3f high = mesh.getBoundsMax(new Vector3f());
		Vector3f min = new Vector3f(Float.MAX_VALUE);
		Vector3f max = new Vector3f(-Float.MAX_VALUE);
		for (int i = 0; i < 8; i++) {
			Vector3f corner = transform.transformPosition(new Vector3f(
					(i & 1) == 0 ? low.x : high.x, (i & 2) == 0 ? low.y : high.y, (i & 4) == 0 ? low.z : high.z));
			min.min(corner);
			max.max(corner);
		}
		float dx = Math.max(0, Math.max(min.x - point.x, point.x - max.x));
		float dy = Math.max(0, Math.max(min.y - point.y, point.y - max.y));
		float dz = Math.max(0, Math.max(min.z - point.z, point.z - max.z));
		return dx * dx + dy * dy + dz * dz;
	}

	private void applyTrail(Camera camera, float pixelAngle, double t, Vector3f origin) {
		Vector3f eye = camera.getPosition();
		for (TrailSegment segment : trail) {
			if (t <= segment.startTime()) {
				hide(segment.object());
				continue;
			}
			Vector3f end = t < segment.endTime() ? bodies.get(segment.body()).getSitePivot() : segment.end();
			placeLine(segment.object(), new Vector3f(segment.start()).sub(origin), new Vector3f(end).sub(origin),
					eye, pixelAngle, TRAIL_WIDTH_PIXELS, 0);
		}
	}

	private void applyCords(Camera camera, float pixelAngle, Vector3f origin) {
		Vector3f eye = camera.getPosition();
		for (int b = 0; b < bodies.size(); b++) {
			ReplayBody body = bodies.get(b);
			Vector3f offset = new Vector3f(body.getSitePivot()).sub(origin);
			List<Vector3f[]> cords = body.getCords();
			List<SceneObject> pool = cordPools.get(b);
			for (int i = 0; i < pool.size(); i++) {
				if (i < cords.size()) {
					placeLine(pool.get(i), new Vector3f(cords.get(i)[0]).add(offset),
							new Vector3f(cords.get(i)[1]).add(offset), eye, pixelAngle, CORD_WIDTH_PIXELS, CORD_MIN_RADIUS);
				} else {
					hide(pool.get(i));
				}
			}
		}
	}

	/** Stretches a unit cylinder between two points, keeping it a few pixels wide on screen. */
	private static void placeLine(SceneObject line, Vector3f from, Vector3f to, Vector3f eye, float pixelAngle,
			float widthPixels, float minRadius) {
		Vector3f span = new Vector3f(to).sub(from);
		float len = span.length();
		if (len < 1e-4f) {
			hide(line);
			return;
		}
		Vector3f middle = new Vector3f(from).add(to).mul(0.5f);
		float radius = Math.max(minRadius, 0.5f * widthPixels * pixelAngle * middle.distance(eye));
		line.getModelMatrix().translation(from)
				.rotate(new Quaternionf().rotateTo(new Vector3f(0, 1, 0), span.div(len)))
				.scale(radius, len, radius);
	}

	private void applyCanopies(double t, Vector3f origin) {
		for (Canopy canopy : canopies) {
			ReplayBody body = bodies.get(canopy.body());
			Vector3f position = t < canopy.deployTime() ? null : body.getCanopyPosition(canopy.deployment());
			if (position == null) {
				for (SceneObject part : canopy.parts()) {
					hide(part);
				}
				continue;
			}
			double inflate = ReplayBody.smoothstep((t - canopy.deployTime()) / CANOPY_INFLATION_TIME);
			double scale = 0.15 + 0.85 * inflate;
			double groundHit = body.getData().getGroundHitTime();
			double collapse = Double.isNaN(groundHit) ? 0 : ReplayBody.smoothstep((t - groundHit) / 1.5);
			Vector3f offset = new Vector3f(body.getSitePivot()).sub(origin).add(position);
			// After touchdown the canopy deflates and falls over beyond the nose, clear of the pieces.
			Vector3f heading = body.heading();
			Vector3f fallAxis = new Vector3f(heading).cross(0, 1, 0).normalize();
			Matrix4f model = new Matrix4f()
					.translation(offset)
					.scale(1, (float) (1 - 0.85 * collapse), 1)	// flatten onto the ground
					.rotate(new Quaternionf().rotateAxis((float) (-collapse * Math.toRadians(85)), fallAxis))
					.rotate(body.getCanopyRotation(canopy.deployment()))
					.scale((float) scale);
			for (SceneObject part : canopy.parts()) {
				part.getModelMatrix().set(model);
			}
		}
	}

	// ---------- cameras ----------

	private void applyCamera(Scene3DOrchestrator orchestrator, Camera camera, CameraMode mode, ReplayBody followed,
			Vector3f origin, float frameSeconds) {
		boolean entered = mode != appliedMode || followed != followedBody;
		appliedMode = mode;
		followedBody = followed;
		Vector3f target = new Vector3f(followed.getSitePivot()).sub(origin);
		switch (mode) {
			case CHASE -> {
				if (entered) {
					camera.resetViewOffset();
					camera.setFieldOfView(CHASE_FOV);
					camera.setAngleX(0.55f);
					camera.setAngleY(0.22f);
					camera.setDistance((float) (Math.max(1.6 * length, 0.6) * S / Math.tan(CHASE_FOV / 2)));
				}
				camera.setCenterOfInterest(target);
			}
			case GROUND -> {
				Vector3f eye = observerSitePosition().sub(origin);
				float distance = eye.distance(target);
				double rocketSpan = Math.max(length, 0.3) * S / GROUND_FRAME_FILL;
				float fov = (float) Math.min(Math.toRadians(50),
						Math.max(Math.toRadians(1.5), 2 * Math.atan(rocketSpan / (2 * distance))));
				groundFov = entered ? fov : groundFov + (fov - groundFov) * Math.min(1f, 3f * frameSeconds);
				camera.setFieldOfView(groundFov);
				lookFrom(camera, eye, target);
			}
			case ONBOARD -> {
				// Mid-body, clear of the nose cone, standing off the tube like a camera pod.
				double station = 0.5 * (followed.getFront() + followed.getAft());
				Matrix4f section = new Matrix4f().translation(target)
						.mul(followed.getSectionMatrix(followed.sectionOf(station)));
				Vector3f eye = section.transformPosition(
						new Vector3f((float) (station * S), 0, (float) ((followed.getRadius() + 0.03) * S)),
						new Vector3f());
				Vector3f look = section.transformDirection(new Vector3f(1, 0, 0.06f), new Vector3f()).normalize();
				camera.setFieldOfView(ONBOARD_FOV);
				lookFrom(camera, eye, new Vector3f(eye).add(look.mul(20)));
			}
		}
		camera.update();
	}

	/** Places the orbit camera at {@code eye} looking at {@code target}. */
	private static void lookFrom(Camera camera, Vector3f eye, Vector3f target) {
		Vector3f offset = new Vector3f(eye).sub(target);
		float distance = offset.length();
		if (distance < 1e-4f) {
			return;
		}
		camera.resetViewOffset();
		camera.setCenterOfInterest(target);
		camera.setAngleX((float) Math.atan2(offset.x, offset.z));
		camera.setAngleY((float) Math.asin(Math.max(-1f, Math.min(1f, offset.y / distance))));
		camera.setDistance(distance);
	}

	/** A spectator standing beside the pad, off to the side of the launch direction. */
	private Vector3f observerSitePosition() {
		double distance = Math.max(30, Math.min(150, 25 * length));
		Vector3f side = new Vector3f(launchAxis.x, 0, launchAxis.z);
		if (side.lengthSquared() < 1e-4f) {
			side.set(0, 0, 1);	// vertical launch: stand south of the pad
		} else {
			side.normalize().set(-side.z, 0, side.x);
		}
		return side.mul((float) (distance * S)).add(0, (float) (OBSERVER_EYE_HEIGHT * S), 0);
	}

	// ---------- helpers ----------

	private static boolean within(List<double[]> spans, double t) {
		for (double[] span : spans) {
			if (t >= span[0] && t < span[1]) {
				return true;
			}
		}
		return false;
	}

	private static void hide(SceneObject object) {
		object.getModelMatrix().scaling(0);
	}

	private static double nanTo(double value, double fallback) {
		return Double.isNaN(value) ? fallback : value;
	}
}
