package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.appearance.Appearance;
import info.openrocket.core.rocketcomponent.Parachute;
import info.openrocket.core.rocketcomponent.RecoveryDevice;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.rocketcomponent.SymmetricComponent;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.util.BoundingBox;
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
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * Poses the rocket and its surroundings for one moment of a replayed flight. Runs on the GL thread.
 *
 * <p>The rocket stays at the world origin, rotated about its launch centre of gravity, and the
 * ground, launch rail and trail move past it. Keeping the rocket at the origin keeps its shadows,
 * outlines and depth precision exactly as in the other 3D views however high it flies.</p>
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
	private static final double CANOPY_INFLATION_TIME = 0.8;
	private static final double HANG_SETTLE_TIME = 2.0;
	private static final double LANDING_SETTLE_HEIGHT = 3.0;
	private static final double OBSERVER_EYE_HEIGHT = 1.7;
	private static final float CHASE_FOV = (float) Math.toRadians(40);
	private static final float ONBOARD_FOV = (float) Math.toRadians(75);
	private static final double GROUND_FRAME_FILL = 0.15;

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

	private final FlightReplayData data;
	private final double cgX;
	private final double length;
	private final double bodyRadius;
	private final double launchCgHeight;
	private final Vector3f launchAxis;
	private final double railLength;
	private final double launchLatitude;

	private final Map<SceneObject, Matrix4f> baseTransforms = new IdentityHashMap<>();
	private final Map<ParticleEmitter, Vector3f[]> baseEmitters = new IdentityHashMap<>();
	private final List<SceneObject> siteObjects = new ArrayList<>();
	private final List<TrailSegment> trail = new ArrayList<>();
	private final List<Canopy> canopies = new ArrayList<>();

	private CameraMode appliedMode;
	private float groundFov = (float) Math.toRadians(20);

	private record TrailSegment(SceneObject object, double startTime, double endTime, Vector3f start, Vector3f end) {
	}

	private record Canopy(double deployTime, int index, SceneObject[] parts) {
	}

	FlightReplayScene(FlightReplayData data, Rocket rocket, double railLength, double launchLatitude) {
		this.data = data;
		this.railLength = railLength;
		this.launchLatitude = launchLatitude;

		BoundingBox bounds = rocket.getSelectedConfiguration().getBoundingBox();
		this.length = bounds.max.getX() - bounds.min.getX();
		double radius = 0;
		for (RocketComponent component : rocket) {
			if (component instanceof SymmetricComponent symmetric) {
				radius = Math.max(radius, symmetric.getMaxRadius());
			}
		}
		this.bodyRadius = radius > 0 ? radius : 0.02;

		double cg = data.getValue(FlightDataType.TYPE_CG_LOCATION, data.getStartTime());
		this.cgX = Double.isFinite(cg) ? cg : bounds.min.getX() + 0.5 * length;

		Vector3f axis = data.getPose().getAxisDirection(data.getStartTime());
		this.launchAxis = axis != null ? axis : new Vector3f(0, 1, 0);
		// The aft end rests on the pad: the CG sits this high above the ground at launch.
		this.launchCgHeight = (bounds.max.getX() - cgX) * launchAxis.y + 0.02;
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

		addSiteObject(scene, ReplayMeshes.groundTiles(0), GRASS_LIGHT, false);
		addSiteObject(scene, ReplayMeshes.groundTiles(1), GRASS_DARK, false);
		addPadAndRail(scene);
		addTrail(scene);
		addCanopies(scene);
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
		Vector3f tail = tailSitePosition();
		SceneObject plate = addSiteObject(scene, ReplayMeshes.disc((float) Math.max(0.25, 3 * bodyRadius) * S, 32),
				CONCRETE, false);
		plate.getModelMatrix().translate(tail.x, 0.03f * S, tail.z);

		// The rail stands on the far side of the rocket from the default chase view.
		SceneObject rail = addSiteObject(scene, ReplayMeshes.unitCylinder(8), RAIL_METAL, false);
		Vector3f railBase = new Vector3f(tail.x, 0, tail.z - (float) (bodyRadius + 0.012) * S);
		rail.getModelMatrix()
				.translate(railBase)
				.rotate(new Quaternionf().rotateTo(new Vector3f(0, 1, 0), launchAxis))
				.scale(0.01f * S, (float) Math.max(railLength, 0.1) * S, 0.01f * S);
	}

	private void addTrail(Scene scene) {
		Mesh cylinder = ReplayMeshes.unitCylinder(6);
		Appearance3D ascent = unlit(TRAIL_ASCENT);
		Appearance3D descent = unlit(TRAIL_DESCENT);
		double start = Double.isNaN(data.getLiftoffTime()) ? data.getStartTime() : data.getLiftoffTime();
		double end = data.getEndTime();
		if (end <= start) {
			return;
		}
		double step = Math.max(TRAIL_MIN_STEP, (end - start) / TRAIL_MAX_SEGMENTS);
		double apogee = data.getApogeeTime();
		for (double t0 = start; t0 < end; t0 += step) {
			double t1 = Math.min(end, t0 + step);
			boolean descending = !Double.isNaN(apogee) && t0 >= apogee;
			SceneObject segment = new SceneObject(cylinder, new Vector3f(), descending ? descent : ascent);
			segment.setSelectable(false);
			scene.addObject(segment);
			trail.add(new TrailSegment(segment, t0, t1, cgSitePosition(t0), cgSitePosition(t1)));
		}
	}

	private void addCanopies(Scene scene) {
		int index = 0;
		for (FlightReplayData.Event deployment : data.getDeployments()) {
			if (!(deployment.source() instanceof Parachute parachute)) {
				continue;	// Streamers are not drawn.
			}
			ReplayMeshes.CanopyShape shape = ReplayMeshes.CanopyShape.of(parachute.getDiameter(),
					parachute.getLineLength(), parachute.getLineCount());
			Vector3f primary = canopyColor(parachute);
			SceneObject goresA = new SceneObject(ReplayMeshes.canopy(shape, 0), new Vector3f(), lit(primary));
			SceneObject goresB = new SceneObject(ReplayMeshes.canopy(shape, 1), new Vector3f(), lit(CANOPY_SECOND));
			SceneObject lines = new SceneObject(ReplayMeshes.shroudLines(shape, 0.004f * S), new Vector3f(),
					unlit(SHROUD_LINE));
			SceneObject[] parts = { goresA, goresB, lines };
			for (SceneObject part : parts) {
				part.setSelectable(false);
				hide(part);
				scene.addObject(part);
			}
			canopies.add(new Canopy(deployment.time(), index++, parts));
		}
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

	private SceneObject addSiteObject(Scene scene, Mesh mesh, Vector3f color, boolean isLit) {
		SceneObject object = new SceneObject(mesh, new Vector3f(), isLit ? lit(color) : unlit(color));
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
	void apply(Scene3DOrchestrator orchestrator, double t, CameraMode mode, float frameSeconds) {
		Scene scene = orchestrator.getScene();
		if (scene == null) {
			return;
		}
		// The rail and pad were placed when they were added; capture them before the first move.
		for (SceneObject object : siteObjects) {
			baseTransforms.computeIfAbsent(object, o -> new Matrix4f(o.getModelMatrix()));
		}

		Matrix4f rocketTransform = rocketTransform(t);
		applyRocketTransform(scene, rocketTransform);
		applyEmitters(scene, rocketTransform, t);

		Vector3f cg = cgSitePosition(t);
		Matrix4f site = new Matrix4f().translation(-cg.x, -cg.y, -cg.z);
		for (SceneObject object : siteObjects) {
			site.mul(baseTransforms.get(object), object.getModelMatrix());
		}

		Camera camera = scene.getCamera();
		applyCamera(orchestrator, camera, mode, t, cg, rocketTransform, frameSeconds);
		applyTrail(camera, orchestrator.getViewport().getWindowHeight(), t, cg);
		applyCanopies(t);
	}

	/** Model transform taking rocket coordinates to the replay world for time {@code t}. */
	Matrix4f rocketTransform(double t) {
		Vector3f axis = attitude(t);
		double roll = data.getRollAngle(rollTime(t));
		// Rocket coordinates run aft along +X from the nose tip; turn the nose (-X) to +Y,
		// then to the flight attitude, pivoting about the launch centre of gravity.
		return new Matrix4f()
				.rotate(new Quaternionf().rotateTo(new Vector3f(0, 1, 0), axis))
				.rotateZ((float) (-Math.PI / 2))
				.rotateX((float) roll)
				.translate((float) (-cgX * S), 0, 0);
	}

	private void applyRocketTransform(Scene scene, Matrix4f rocketTransform) {
		Matrix4f sceneRotation = scene.getRocketRotationTransform(new Matrix4f());
		Matrix4f sceneRotationInverse = new Matrix4f(sceneRotation).invert();
		for (SceneObject object : scene.getObjects()) {
			if (object.getRocketComponent() == null) {
				continue;
			}
			Matrix4f base = baseTransforms.computeIfAbsent(object,
					o -> sceneRotationInverse.mul(o.getModelMatrix(), new Matrix4f()));
			rocketTransform.mul(base, object.getModelMatrix());
		}
		// Forget objects a rocket rebuild has replaced.
		baseTransforms.keySet().removeIf(o -> o.getRocketComponent() != null && !scene.getObjects().contains(o));
	}

	private void applyEmitters(Scene scene, Matrix4f rocketTransform, double t) {
		boolean flame = data.isThrusting(t);
		double liftoff = data.getLiftoffTime();
		double smokeEnd = Math.min(nanTo(data.getFirstDeploymentTime(), Double.MAX_VALUE),
				nanTo(data.getGroundHitTime(), Double.MAX_VALUE));
		// Delay grains keep smoking after burnout until the ejection charge.
		boolean smoke = flame || (!Double.isNaN(liftoff) && t >= liftoff && t < smokeEnd);

		baseEmitters.keySet().retainAll(scene.getParticleEmitters());
		for (ParticleEmitter emitter : scene.getParticleEmitters()) {
			Vector3f[] base = baseEmitters.computeIfAbsent(emitter, e -> new Vector3f[] {
					new Vector3f(e.getEmitterPosition()), new Vector3f(e.getDirection()) });
			emitter.setEmitterPosition(rocketTransform.transformPosition(base[0], new Vector3f()));
			emitter.setDirection(rocketTransform.transformDirection(base[1], new Vector3f()).normalize());

			boolean on = emitter instanceof FlameEmitter ? flame : emitter instanceof SmokeEmitter && smoke;
			if (on && emitter.isStaticMode()) {
				emitter.setStaticMode(false);
			} else if (!on && !emitter.isStaticMode()) {
				emitter.getParticles().clear();
				emitter.setStaticMode(true);
			}
		}
	}

	private void applyTrail(Camera camera, int viewportHeight, double t, Vector3f cg) {
		Vector3f eye = camera.getPosition();
		float pixelAngle = (float) (2 * Math.tan(camera.getFieldOfView() / 2) / Math.max(1, viewportHeight));
		Vector3f now = null;
		for (TrailSegment segment : trail) {
			Matrix4f model = segment.object().getModelMatrix();
			if (t <= segment.startTime()) {
				hide(segment.object());
				continue;
			}
			Vector3f end = segment.end();
			if (t < segment.endTime()) {
				if (now == null) {
					now = cgSitePosition(t);
				}
				end = now;
			}
			Vector3f from = new Vector3f(segment.start()).sub(cg);
			Vector3f to = new Vector3f(end).sub(cg);
			Vector3f span = new Vector3f(to).sub(from);
			float len = span.length();
			if (len < 1e-4f) {
				hide(segment.object());
				continue;
			}
			Vector3f middle = new Vector3f(from).add(to).mul(0.5f);
			float radius = 0.5f * TRAIL_WIDTH_PIXELS * pixelAngle * middle.distance(eye);
			model.translation(from)
					.rotate(new Quaternionf().rotateTo(new Vector3f(0, 1, 0), span.div(len)))
					.scale(radius, len, radius);
		}
	}

	private void applyCanopies(double t) {
		double groundHit = data.getGroundHitTime();
		for (Canopy canopy : canopies) {
			if (t < canopy.deployTime()) {
				for (SceneObject part : canopy.parts()) {
					hide(part);
				}
				continue;
			}
			double inflate = smoothstep((t - canopy.deployTime()) / CANOPY_INFLATION_TIME);
			double scale = 0.15 + 0.85 * inflate;
			double collapse = Double.isNaN(groundHit) ? 0 : smoothstep((t - groundHit) / 1.5);
			// A slow pendulum sway, different for each canopy.
			double phase = canopy.index() * 1.7;
			float swayX = (float) Math.toRadians(4 * Math.sin(2 * Math.PI * t / 3.1 + phase));
			float swayZ = (float) Math.toRadians(3 * Math.sin(2 * Math.PI * t / 4.3 + 1 + phase));
			// Later canopies hang off to the side of earlier ones.
			float lean = (float) Math.toRadians(14 * canopy.index());
			Matrix4f model = new Matrix4f()
					.rotateZ(-lean + swayZ)
					.rotateX(swayX + (float) (collapse * Math.toRadians(80)))
					.scale((float) scale, (float) (scale * (1 - 0.85 * collapse)), (float) scale);
			for (SceneObject part : canopy.parts()) {
				part.getModelMatrix().set(model);
			}
		}
	}

	// ---------- cameras ----------

	private void applyCamera(Scene3DOrchestrator orchestrator, Camera camera, CameraMode mode, double t,
			Vector3f cg, Matrix4f rocketTransform, float frameSeconds) {
		boolean entered = mode != appliedMode;
		appliedMode = mode;
		switch (mode) {
			case CHASE -> {
				if (entered) {
					camera.resetViewOffset();
					camera.setCenterOfInterest(new Vector3f());
					camera.setFieldOfView(CHASE_FOV);
					camera.setAngleX(0.55f);
					camera.setAngleY(0.22f);
					camera.setDistance((float) (Math.max(1.6 * length, 0.6) * S / Math.tan(CHASE_FOV / 2)));
				}
			}
			case GROUND -> {
				Vector3f eye = observerSitePosition().sub(cg);
				float distance = eye.length();
				double rocketSpan = Math.max(length, 0.3) * S / GROUND_FRAME_FILL;
				float target = (float) Math.min(Math.toRadians(50),
						Math.max(Math.toRadians(1.5), 2 * Math.atan(rocketSpan / (2 * distance))));
				groundFov = entered ? target : groundFov + (target - groundFov) * Math.min(1f, 3f * frameSeconds);
				camera.setFieldOfView(groundFov);
				lookFrom(camera, eye, new Vector3f());
			}
			case ONBOARD -> {
				// Mid-body, clear of the nose cone, standing off the tube like a camera pod.
				Vector3f eye = rocketTransform.transformPosition(
						new Vector3f((float) (0.5 * length * S), 0, (float) ((bodyRadius + 0.03) * S)), new Vector3f());
				Vector3f look = rocketTransform.transformDirection(new Vector3f(1, 0, 0.06f), new Vector3f()).normalize();
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

	// ---------- flight geometry ----------

	/** Direction the nose points, with the rocket settling level under a parachute. */
	Vector3f attitude(double t) {
		Vector3f axis = axisAt(t);
		double deploy = data.getFirstDeploymentTime();
		if (Double.isNaN(deploy) || t <= deploy) {
			return axis;
		}
		Vector3f atDeploy = axisAt(deploy);
		Vector3f heading = new Vector3f(atDeploy.x, 0, atDeploy.z);
		if (heading.lengthSquared() < 1e-6f) {
			heading.set(1, 0, 0);
		}
		heading.normalize();
		// Hang slightly nose down, levelling out as it settles onto the ground.
		double noseDown = Math.toRadians(8) * landingSettle(t);
		Vector3f hanging = heading.mul((float) Math.cos(noseDown)).add(0, (float) -Math.sin(noseDown), 0);
		float blend = (float) smoothstep((t - deploy) / HANG_SETTLE_TIME);
		return atDeploy.lerp(hanging, blend).normalize();
	}

	private Vector3f axisAt(double t) {
		Vector3f axis = data.getPose().getAxisDirection(t);
		return axis != null ? axis : new Vector3f(launchAxis);
	}

	/** Roll stops being meaningful once the rocket hangs from a parachute. */
	private double rollTime(double t) {
		double deploy = data.getFirstDeploymentTime();
		return Double.isNaN(deploy) ? t : Math.min(t, deploy);
	}

	/**
	 * Position of the centre of gravity in the launch-site frame, in engine units. The site
	 * origin is on the ground below the CG at launch.
	 */
	Vector3f cgSitePosition(double t) {
		Vector3f position = data.getPose().getPosition(t);
		position.y += (float) (cgHeightOffset(t) * S);
		return position;
	}

	/**
	 * The simulated altitude is zero at launch and at landing. Lift the rocket so the aft end rests
	 * on the pad at launch, and let it settle onto its side for landing.
	 */
	private double cgHeightOffset(double t) {
		double apogee = data.getApogeeTime();
		if (Double.isNaN(apogee) || t < apogee) {
			return launchCgHeight;
		}
		return bodyRadius + (launchCgHeight - bodyRadius) * landingSettle(t);
	}

	/** 1 while the rocket is well above the ground, falling to 0 as it touches down. */
	private double landingSettle(double t) {
		double altitude = data.getValue(FlightDataType.TYPE_ALTITUDE, t);
		return Double.isFinite(altitude) ? Math.max(0, Math.min(1, altitude / LANDING_SETTLE_HEIGHT)) : 1;
	}

	private Vector3f tailSitePosition() {
		Vector3f cg = cgSitePosition(data.getStartTime());
		return cg.sub(new Vector3f(launchAxis).mul((float) (length - cgX) * S));
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

	private static void hide(SceneObject object) {
		object.getModelMatrix().scaling(0);
	}

	private static double smoothstep(double x) {
		double c = Math.max(0, Math.min(1, x));
		return c * c * (3 - 2 * c);
	}

	private static double nanTo(double value, double fallback) {
		return Double.isNaN(value) ? fallback : value;
	}
}
