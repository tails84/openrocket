package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.document.OpenRocketDocument;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.FlightDataType;
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
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.KeyStroke;
import javax.swing.ScrollPaneConstants;
import javax.swing.Timer;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Window that plays back a simulated flight in 3D, with a timeline and live flight values.
 */
public final class FlightReplayFrame extends JFrame {
	private static final long serialVersionUID = 1L;
	private static final Translator trans = Application.getTranslator();
	private static final double[] SPEEDS = { 0.1, 0.25, 0.5, 1, 2, 5, 10 };
	private static final int DEFAULT_SPEED = 3;
	private static final double STEP_SECONDS = 1.0;

	private final List<FlightReplayData> branches;
	private final FlightReplayPanel replayPanel;
	private final FlightReplayTimeline timeline;
	private final FlightReplayReadout readout;
	private final JButton playButton;
	private final JLabel followLabel;
	private final JComboBox<String> followBox;
	private final Timer refreshTimer;
	private boolean measuredShown;

	/**
	 * Opens a replay of the first flight branch of a simulation that has flight data.
	 *
	 * @param owner      window to centre on
	 * @param document   document holding the simulated rocket
	 * @param simulation simulation with flight data
	 */
	public static void open(Window owner, OpenRocketDocument document, Simulation simulation) {
		List<FlightReplayData> branches = new ArrayList<>();
		if (simulation.hasSimulationData()) {
			FlightData flightData = simulation.getSimulatedData();
			for (int i = 0; i < flightData.getBranchCount(); i++) {
				try {
					branches.add(FlightReplayData.fromBranch(flightData.getBranch(i)));
				} catch (IllegalArgumentException e) {
					// The main branch must replay; a stage branch without position data is left out.
					if (i == 0) {
						break;
					}
				}
			}
		}
		if (branches.isEmpty()) {
			JOptionPane.showMessageDialog(owner, trans.get("FlightReplay.noData"),
					trans.get("FlightReplay.title"), JOptionPane.INFORMATION_MESSAGE);
			return;
		}
		FlightReplayFrame frame = new FlightReplayFrame(document, simulation, branches);
		frame.setLocationRelativeTo(owner);
		frame.setVisible(true);
	}

	private FlightReplayFrame(OpenRocketDocument document, Simulation simulation, List<FlightReplayData> branches) {
		super(trans.get("FlightReplay.title") + " - " + simulation.getName());
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		GUIUtil.setWindowIcons(this);

		this.branches = branches;
		SimulationOptions options = simulation.getOptions();
		FlightReplayData data = branches.get(0);
		FlightReplayScene scene = new FlightReplayScene(branches, document.getRocket(),
				options.getLaunchRodLength(), options.getLaunchLatitude());
		replayPanel = new FlightReplayPanel(document, branches, scene);
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

		followBox = new JComboBox<>();
		for (FlightReplayData branch : branches) {
			followBox.addItem(branch.getName());
		}
		followBox.setFocusable(false);
		followBox.addActionListener(e -> {
			int index = Math.max(0, followBox.getSelectedIndex());
			replayPanel.setFollowBody(index < branches.size() ? index : FlightReplayScene.FOLLOW_MEASURED);
		});
		followLabel = new JLabel(trans.get("FlightReplay.lbl.follow"));

		JButton loadLogButton = new JButton(trans.get("FlightReplay.but.loadLog"));
		loadLogButton.setToolTipText(trans.get("FlightReplay.but.ttip.loadLog"));
		loadLogButton.setFocusable(false);
		loadLogButton.addActionListener(e -> chooseLog());

		JPanel controls = new JPanel(new MigLayout("fillx, hidemode 3, insets 6 10 8 10",
				"[][]20[][]20[][]20[][]20[]push[]"));
		controls.add(playButton, "sg playback");
		controls.add(restartButton, "sg playback");
		controls.add(new JLabel(trans.get("FlightReplay.lbl.speed")));
		controls.add(speedBox);
		controls.add(new JLabel(trans.get("FlightReplay.lbl.camera")));
		controls.add(cameraBox);
		// Only staged flights, or a flight beside a recorded one, have more than one thing to follow.
		controls.add(followLabel);
		controls.add(followBox);
		setFollowVisible(branches.size() > 1);
		controls.add(loadLogButton);
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

	private void setFollowVisible(boolean visible) {
		followLabel.setVisible(visible);
		followBox.setVisible(visible);
	}

	private void chooseLog() {
		JFileChooser chooser = new JFileChooser(Application.getPreferences().getDefaultDirectory());
		chooser.setDialogTitle(trans.get("FlightReplay.loadLog.title"));
		chooser.setFileFilter(new FileNameExtensionFilter(trans.get("FlightReplay.loadLog.filter"), "csv", "txt", "tsv"));
		if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) {
			return;
		}
		Application.getPreferences().setDefaultDirectory(chooser.getCurrentDirectory());
		loadLog(chooser.getSelectedFile().toPath());
	}

	/** Reads an altimeter or flight computer log and shows it beside the simulated flight. */
	void loadLog(Path file) {
		FlightReplayData simulated = branches.get(0);
		double expectedApogee = Double.isNaN(simulated.getApogeeTime()) ? Double.NaN
				: simulated.getValue(FlightDataType.TYPE_ALTITUDE, simulated.getApogeeTime());
		FlightLog log;
		try {
			log = FlightLogReader.read(file, expectedApogee);
		} catch (IOException | FlightLogReader.LogException e) {
			JOptionPane.showMessageDialog(this, trans.get("FlightReplay.loadLog.failed") + "\n" + e.getMessage(),
					trans.get("FlightReplay.loadLog.title"), JOptionPane.ERROR_MESSAGE);
			return;
		}
		FlightReplayData measured = MeasuredFlight.build(log, simulated, trans.get("FlightReplay.measured"));
		replayPanel.setMeasured(measured);
		timeline.setMeasured(measured, replayPanel.getEndTime());
		readout.setMeasured(log, measured);
		if (!measuredShown) {
			followBox.addItem(trans.get("FlightReplay.measured"));
			measuredShown = true;
		}
		setFollowVisible(true);
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
