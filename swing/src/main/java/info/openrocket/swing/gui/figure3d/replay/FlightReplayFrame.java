package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.startup.Application;
import info.openrocket.swing.gui.util.GUIUtil;
import net.miginfocom.swing.MigLayout;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.InputMap;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.KeyStroke;
import javax.swing.ScrollPaneConstants;
import javax.swing.Timer;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

/**
 * Window that plays back a simulated flight in 3D, with a timeline and live flight values.
 */
public final class FlightReplayFrame extends JFrame {
	private static final long serialVersionUID = 1L;
	private static final Translator trans = Application.getTranslator();
	private static final double[] SPEEDS = { 0.1, 0.25, 0.5, 1, 2, 5, 10 };
	private static final int DEFAULT_SPEED = 3;
	private static final double STEP_SECONDS = 1.0;

	private final FlightReplayPanel replayPanel;
	private final FlightReplayTimeline timeline;
	private final FlightReplayReadout readout;
	private final JButton playButton;
	private final Timer refreshTimer;

	/**
	 * Opens a replay of the first flight branch of a simulation that has flight data.
	 *
	 * @param owner      window to centre on
	 * @param document   document holding the simulated rocket
	 * @param simulation simulation with flight data
	 */
	public static void open(Window owner, OpenRocketDocument document, Simulation simulation) {
		FlightReplayData data = null;
		if (simulation.hasSimulationData() && simulation.getSimulatedData().getBranchCount() > 0) {
			FlightDataBranch branch = simulation.getSimulatedData().getBranch(0);
			try {
				data = FlightReplayData.fromBranch(branch);
			} catch (IllegalArgumentException e) {
				data = null;
			}
		}
		if (data == null) {
			JOptionPane.showMessageDialog(owner, trans.get("FlightReplay.noData"),
					trans.get("FlightReplay.title"), JOptionPane.INFORMATION_MESSAGE);
			return;
		}
		FlightReplayFrame frame = new FlightReplayFrame(document, simulation, data);
		frame.setLocationRelativeTo(owner);
		frame.setVisible(true);
	}

	private FlightReplayFrame(OpenRocketDocument document, Simulation simulation, FlightReplayData data) {
		super(trans.get("FlightReplay.title") + " - " + simulation.getName());
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		GUIUtil.setWindowIcons(this);

		SimulationOptions options = simulation.getOptions();
		FlightReplayScene scene = new FlightReplayScene(data, document.getRocket(),
				options.getLaunchRodLength(), options.getLaunchLatitude());
		replayPanel = new FlightReplayPanel(document, data, scene);
		readout = new FlightReplayReadout(data);
		timeline = new FlightReplayTimeline(data, replayPanel.getStartTime(), replayPanel.getEndTime(),
				replayPanel::seek);

		playButton = new JButton(trans.get("FlightReplay.but.play"));
		playButton.setFocusable(false);
		playButton.addActionListener(e -> togglePlay());

		JButton restartButton = new JButton(trans.get("FlightReplay.but.restart"));
		restartButton.setFocusable(false);
		restartButton.addActionListener(e -> replayPanel.seek(replayPanel.getStartTime()));

		JComboBox<String> speedBox = new JComboBox<>();
		for (double speed : SPEEDS) {
			speedBox.addItem(formatSpeed(speed));
		}
		speedBox.setSelectedIndex(DEFAULT_SPEED);
		speedBox.setFocusable(false);
		speedBox.addActionListener(e -> replayPanel.setRate(SPEEDS[Math.max(0, speedBox.getSelectedIndex())]));

		JComboBox<String> cameraBox = new JComboBox<>();
		for (FlightReplayScene.CameraMode mode : FlightReplayScene.CameraMode.values()) {
			cameraBox.addItem(trans.get("FlightReplay.camera." + mode.name()));
		}
		cameraBox.setFocusable(false);
		cameraBox.addActionListener(e -> replayPanel.setCameraMode(
				FlightReplayScene.CameraMode.values()[Math.max(0, cameraBox.getSelectedIndex())]));

		JPanel controls = new JPanel(new MigLayout("fillx, insets 6 10 8 10", "[][]20[][]20[][]push[]"));
		controls.add(playButton, "sg playback");
		controls.add(restartButton, "sg playback");
		controls.add(new JLabel(trans.get("FlightReplay.lbl.speed")));
		controls.add(speedBox);
		controls.add(new JLabel(trans.get("FlightReplay.lbl.camera")));
		controls.add(cameraBox);
		JLabel hint = new JLabel(trans.get("FlightReplay.lbl.keys"));
		hint.setEnabled(false);
		controls.add(hint, "wmin 0, wrap");
		controls.add(timeline, "span, growx, h 90!");

		// The playback keys work while the window has focus; controls do not take it.
		JPanel content = new JPanel(new BorderLayout());
		content.setFocusable(true);
		content.add(replayPanel, BorderLayout.CENTER);
		JScrollPane readoutScroll = new JScrollPane(readout, ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED,
				ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		readoutScroll.setBorder(BorderFactory.createEmptyBorder());
		content.add(readoutScroll, BorderLayout.EAST);
		content.add(controls, BorderLayout.SOUTH);
		setContentPane(content);
		installKeyBindings(content, cameraBox);

		setMinimumSize(new Dimension(720, 480));
		setSize(1200, 780);
		GUIUtil.rememberWindowSize(this);

		refreshTimer = new Timer(40, e -> refresh());
		addWindowListener(new WindowAdapter() {
			@Override
			public void windowOpened(WindowEvent e) {
				replayPanel.start();
				content.requestFocusInWindow();
				refreshTimer.start();
				replayPanel.play();
			}

			@Override
			public void windowClosed(WindowEvent e) {
				refreshTimer.stop();
				replayPanel.dispose();
			}
		});
		refresh();
	}

	private void installKeyBindings(JComponent root, JComboBox<String> cameraBox) {
		InputMap keys = root.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
		keys.put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "togglePlay");
		keys.put(KeyStroke.getKeyStroke(KeyEvent.VK_HOME, 0), "restart");
		keys.put(KeyStroke.getKeyStroke(KeyEvent.VK_LEFT, 0), "back");
		keys.put(KeyStroke.getKeyStroke(KeyEvent.VK_RIGHT, 0), "forward");
		keys.put(KeyStroke.getKeyStroke(KeyEvent.VK_C, 0), "nextCamera");
		root.getActionMap().put("togglePlay", action(this::togglePlay));
		root.getActionMap().put("restart", action(() -> replayPanel.seek(replayPanel.getStartTime())));
		root.getActionMap().put("back", action(() -> replayPanel.seek(replayPanel.getTime() - STEP_SECONDS)));
		root.getActionMap().put("forward", action(() -> replayPanel.seek(replayPanel.getTime() + STEP_SECONDS)));
		root.getActionMap().put("nextCamera", action(() ->
				cameraBox.setSelectedIndex((cameraBox.getSelectedIndex() + 1) % cameraBox.getItemCount())));
	}

	private static AbstractAction action(Runnable runnable) {
		return new AbstractAction() {
			private static final long serialVersionUID = 1L;

			@Override
			public void actionPerformed(ActionEvent e) {
				runnable.run();
			}
		};
	}

	private void togglePlay() {
		if (replayPanel.isPlaying()) {
			replayPanel.pause();
		} else {
			replayPanel.play();
		}
		refresh();
	}

	private void refresh() {
		double t = replayPanel.getTime();
		timeline.setTime(t);
		readout.setTime(t);
		playButton.setText(trans.get(replayPanel.isPlaying() ? "FlightReplay.but.pause" : "FlightReplay.but.play"));
	}

	private static String formatSpeed(double speed) {
		String number = speed == Math.rint(speed) ? Integer.toString((int) speed) : Double.toString(speed);
		return number + "×";
	}
}
