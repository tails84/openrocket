package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.startup.Application;
import net.miginfocom.swing.MigLayout;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.UIManager;
import java.awt.Color;
import java.awt.Font;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Side panel with the flight values at the replay's current time.
 */
final class FlightReplayReadout extends JPanel {
	private static final long serialVersionUID = 1L;
	private static final Translator trans = Application.getTranslator();
	private static final String MISSING = "–";

	private static final FlightDataType[] VALUES = {
			FlightDataType.TYPE_ALTITUDE,
			FlightDataType.TYPE_VELOCITY_Z,
			FlightDataType.TYPE_VELOCITY_TOTAL,
			FlightDataType.TYPE_ACCELERATION_TOTAL,
			FlightDataType.TYPE_MACH_NUMBER,
			FlightDataType.TYPE_THRUST_FORCE,
	};

	private final FlightReplayData data;
	private final JLabel timeValue = valueLabel();
	private final JLabel phaseValue = valueLabel();
	private final Map<FlightDataType, JLabel> valueLabels = new LinkedHashMap<>();
	private double shownTime = Double.NaN;

	FlightReplayReadout(FlightReplayData data) {
		super(new MigLayout("wrap 1, fillx, insets 12 14 12 14, gapy 0", "[200!]"));
		this.data = data;

		addRow(FlightDataType.TYPE_TIME.getName(), timeValue);
		addRow(trans.get("FlightReplay.readout.phase"), phaseValue);
		add(new JSeparator(), "growx, gaptop 6, gapbottom 6");
		for (FlightDataType type : VALUES) {
			if (!data.hasChannel(type)) {
				continue;
			}
			JLabel label = valueLabel();
			valueLabels.put(type, label);
			addRow(type.getName(), label);
		}

		double apogee = data.getApogeeTime();
		if (!Double.isNaN(apogee)) {
			add(new JSeparator(), "growx, gaptop 6, gapbottom 6");
			JLabel summary = new JLabel(trans.get("FlightReplay.readout.apogee") + " "
					+ format(FlightDataType.TYPE_ALTITUDE, data.getValue(FlightDataType.TYPE_ALTITUDE, apogee))
					+ ", T+ " + format(FlightDataType.TYPE_TIME, apogee));
			summary.setForeground(captionColor());
			add(summary);
		}
	}

	void setTime(double t) {
		if (t == shownTime) {
			return;
		}
		shownTime = t;
		timeValue.setText("T+ " + format(FlightDataType.TYPE_TIME, t));
		phaseValue.setText(trans.get("FlightReplay.phase." + data.getPhase(t).name()));
		for (Map.Entry<FlightDataType, JLabel> entry : valueLabels.entrySet()) {
			entry.getValue().setText(format(entry.getKey(), data.getValue(entry.getKey(), t)));
		}
	}

	private void addRow(String caption, JLabel value) {
		JLabel captionLabel = new JLabel(caption);
		captionLabel.setForeground(captionColor());
		add(captionLabel, "gaptop 4");
		add(value);
	}

	private static JLabel valueLabel() {
		JLabel label = new JLabel(MISSING);
		label.setFont(label.getFont().deriveFont(Font.BOLD, label.getFont().getSize2D() + 4f));
		return label;
	}

	private static Color captionColor() {
		Color color = UIManager.getColor("Label.disabledForeground");
		return color != null ? color : Color.GRAY;
	}

	private static String format(FlightDataType type, double value) {
		if (!Double.isFinite(value)) {
			return MISSING;
		}
		return type.getUnitGroup().getDefaultUnit().toStringUnit(value);
	}
}
