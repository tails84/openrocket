package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.figure3d.SharedCanvasRenderScheduler;
import info.openrocket.swing.gui.figure3d.animation.PlaybackClock;
import info.openrocket.swing.gui.figure3d.scene.orchestration.Scene3DOrchestrator;
import info.openrocket.swing.gui.figure3d.ui.GLScenePanel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import java.awt.BorderLayout;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * 3D view that plays back a simulated flight. Playback state is changed from the Swing thread
 * and advanced on the render thread, which owns the clock.
 */
final class FlightReplayPanel extends JPanel implements SharedCanvasRenderScheduler.Client {
	private static final long serialVersionUID = 1L;
	private static final Logger log = LoggerFactory.getLogger(FlightReplayPanel.class);
	private static final Translator trans = Application.getTranslator();
	private static final SharedCanvasRenderScheduler RENDER_SCHEDULER = SharedCanvasRenderScheduler.getInstance();
	private static final long RENDER_SHUTDOWN_TIMEOUT_MS = 2_000;
	/** Keep playing a moment past touchdown so the canopy can settle. */
	/** Time to keep playing after touchdown, while the pieces settle and the canopies come down. */
	private static final double LANDING_HOLD_SECONDS = 6.0;
	/** Longest step the clock takes in one frame, so a stalled window does not jump ahead. */
	private static final float MAX_FRAME_SECONDS = 0.5f;

	private final OpenRocketDocument document;
	private final FlightReplayScene replayScene;
	private final boolean hasMotors;

	// Render thread only
	private PlaybackClock clock;
	private long lastFrameNanos;

	private final double startTime;
	/** End of the simulated flight. */
	private final double simulatedEnd;
	private volatile double endTime;

	private final AtomicReference<Double> pendingSeek = new AtomicReference<>();
	private final AtomicBoolean dirty = new AtomicBoolean(true);
	private volatile GLScenePanel glPanel;
	private volatile boolean renderLoopRunning;
	private volatile boolean playing;
	private volatile double rate = 1.0;
	private volatile FlightReplayScene.CameraMode cameraMode = FlightReplayScene.CameraMode.CHASE;
	private volatile int followBody;
	private volatile double currentTime;

	/**
	 * @param branches flight data branches; the first is the main rocket, the others are stages
	 *                 that separated from it
	 */
	FlightReplayPanel(OpenRocketDocument document, List<FlightReplayData> branches, FlightReplayScene replayScene) {
		super(new BorderLayout());
		this.document = document;
		this.replayScene = replayScene;
		this.hasMotors = document.getSelectedConfiguration() != null
				&& document.getSelectedConfiguration().hasMotors();
		FlightReplayData main = branches.get(0);
		this.startTime = main.getStartTime();
		this.simulatedEnd = endOf(branches);
		this.endTime = simulatedEnd;
		this.clock = new PlaybackClock(startTime, endTime);
		this.currentTime = startTime;
	}

	/** When playback stops: the last branch's end, and a moment more if something lands. */
	private static double endOf(List<FlightReplayData> branches) {
		double end = branches.get(0).getEndTime();
		boolean landed = false;
		for (FlightReplayData branch : branches) {
			end = Math.max(end, branch.getEndTime());
			landed |= !Double.isNaN(branch.getGroundHitTime());
		}
		return landed ? end + LANDING_HOLD_SECONDS : end;
	}

	// ---------- playback control (any thread) ----------

	void play() {
		if (currentTime >= endTime) {
			seek(startTime);
		}
		playing = true;
		requestRenderNow();
	}

	void pause() {
		playing = false;
		requestRenderNow();
	}

	boolean isPlaying() {
		return playing;
	}

	void seek(double time) {
		double clamped = Math.max(startTime, Math.min(endTime, time));
		pendingSeek.set(clamped);
		currentTime = clamped;
		requestRenderNow();
	}

	void setRate(double rate) {
		this.rate = rate;
	}

	void setCameraMode(FlightReplayScene.CameraMode mode) {
		this.cameraMode = mode;
		requestRenderNow();
	}

	/** Chooses which flight branch the cameras follow: 0 for the main rocket. */
	void setFollowBody(int body) {
		this.followBody = body;
		requestRenderNow();
	}

	double getTime() {
		return currentTime;
	}

	double getStartTime() {
		return startTime;
	}

	double getEndTime() {
		return endTime;
	}

	/**
	 * Shows a recorded flight beside the simulation, and plays on until it ends too.
	 *
	 * @param measured the measured flight on the simulation's clock
	 */
	void setMeasured(FlightReplayData measured) {
		endTime = Math.max(simulatedEnd, endOf(List.of(measured)));
		currentTime = Math.min(currentTime, endTime);
		replayScene.setMeasured(measured);
		requestRenderNow();
	}

	// ---------- lifecycle ----------

	void start() {
		if (glPanel != null) {
			return;
		}
		GLScenePanel panel;
		try {
			panel = new GLScenePanel(document.getRocket(), null);
		} catch (UnsatisfiedLinkError | ExceptionInInitializerError e) {
			log.warn("Flight replay unavailable: LWJGL native libraries not found for {}/{}.",
					System.getProperty("os.name"), System.getProperty("os.arch"), e);
			showFailure();
			return;
		}
		glPanel = panel;
		// Leave keyboard focus with the window so its playback keys keep working.
		panel.setFocusable(false);
		panel.setInitializationHook(this::initializeOnGlThread);
		panel.setRenderActivityCallback(this::markDirty);
		panel.setRenderRequestCallback(this::requestRenderNow);
		panel.setGlInitFailureCallback(() -> SwingUtilities.invokeLater(() -> {
			stopRenderLoop();
			showFailure();
		}));
		add(panel, BorderLayout.CENTER);
		revalidate();
		startRenderLoop();
	}

	void dispose() {
		stopRenderLoop();
		GLScenePanel panel = glPanel;
		if (panel == null) {
			return;
		}
		RENDER_SCHEDULER.awaitQuiescence(this, RENDER_SHUTDOWN_TIMEOUT_MS);
		panel.setInitializationHook(null);
		panel.setRenderActivityCallback(null);
		panel.setRenderRequestCallback(null);
		panel.setGlInitFailureCallback(null);
		RENDER_SCHEDULER.disposeOnRenderThread(this, panel::cleanup, RENDER_SHUTDOWN_TIMEOUT_MS);
		remove(panel);
		glPanel = null;
	}

	private void initializeOnGlThread(Scene3DOrchestrator orchestrator) {
		replayScene.install(orchestrator, hasMotors);
		orchestrator.setRocketSceneRebuiltCallback(this::markDirty);
		replayScene.apply(orchestrator, currentTime, cameraMode, followBody, 0f);
	}

	private void showFailure() {
		JLabel label = new JLabel(trans.get("FlightReplay.glInitFailed"));
		label.setHorizontalAlignment(SwingConstants.CENTER);
		removeAll();
		add(label, BorderLayout.CENTER);
		revalidate();
		repaint();
	}

	// ---------- render loop ----------

	private void startRenderLoop() {
		if (renderLoopRunning) {
			return;
		}
		renderLoopRunning = true;
		markDirty();
		RENDER_SCHEDULER.register(this);
	}

	private void stopRenderLoop() {
		if (renderLoopRunning) {
			renderLoopRunning = false;
			RENDER_SCHEDULER.unregister(this);
		}
	}

	private void markDirty() {
		dirty.set(true);
	}

	private void requestRenderNow() {
		markDirty();
		if (renderLoopRunning) {
			RENDER_SCHEDULER.requestImmediate(this);
		}
	}

	@Override
	public boolean isRenderActive() {
		return renderLoopRunning && glPanel != null;
	}

	@Override
	public boolean shouldRenderOnTick() {
		// Clear first so activity arriving while this frame renders schedules another.
		boolean wasDirty = dirty.getAndSet(false);
		return playing || wasDirty;
	}

	@Override
	public void renderScheduledFrame() {
		GLScenePanel panel = glPanel;
		if (!renderLoopRunning || panel == null) {
			return;
		}
		if (panel.hasGlInitFailed()) {
			return;
		}
		if (!panel.isDisplayable() || !panel.isShowing() || panel.getWidth() <= 0 || panel.getHeight() <= 0) {
			markDirty();
			return;
		}

		long now = System.nanoTime();
		float frameSeconds = lastFrameNanos == 0 ? 0f
				: Math.min(MAX_FRAME_SECONDS, (now - lastFrameNanos) / 1.0e9f);
		lastFrameNanos = now;

		// A measured flight changes where playback ends.
		if (clock.getEnd() != endTime) {
			PlaybackClock resized = new PlaybackClock(startTime, endTime);
			resized.setTime(clock.getTime());
			resized.setRate(clock.getRate());
			clock = resized;
		}
		Double seek = pendingSeek.getAndSet(null);
		if (seek != null) {
			clock.setTime(seek);
		}
		if (playing && seek == null) {
			clock.setRate(rate);
			clock.update(frameSeconds);
			if (clock.getTime() >= clock.getEnd()) {
				playing = false;
			}
		}
		double t = clock.getTime();
		// A seek that arrived during this frame wins until the next frame applies it.
		if (pendingSeek.get() == null) {
			currentTime = t;
		}

		Scene3DOrchestrator orchestrator = panel.getScene3DOrchestrator();
		if (orchestrator != null) {
			FlightReplayScene.CameraMode mode = cameraMode;
			int follow = followBody;
			orchestrator.enqueueGlTask(() -> replayScene.apply(orchestrator, t, mode, follow, frameSeconds));
		}
		panel.render();
	}

	@Override
	public String getRenderDebugName() {
		return "FlightReplayPanel";
	}
}
