package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.startup.Application;
import net.miginfocom.swing.MigLayout;

import javax.swing.Icon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.UIManager;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Side panel with the flight values at the replay's current time.
 */
final class FlightReplayReadout extends JPanel {
	private static final long serialVersionUID = 1L;
	private static final Translator trans = Application.getTranslator();
	private static final String MISSING = "–";
	/** Width of the panel's column, pixels. */
	private static final int WIDTH = 200;

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

	private final JPanel measuredPanel = new JPanel(new MigLayout("wrap 1, fillx, insets 0, gapy 0", "[" + WIDTH + "!]"));
	private FlightReplayData measured;
	private final JLabel measuredAltitude = valueLabel();
	private final JLabel measuredVelocity = valueLabel();
	private final JLabel altitudeDifference = valueLabel();

	FlightReplayReadout(FlightReplayData data) {
		super(new MigLayout("wrap 1, fillx, insets 12 14 12 14, gapy 0", "[" + WIDTH + "!]"));
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
		measuredPanel.setOpaque(false);
		measuredPanel.setVisible(false);
		add(measuredPanel, "hidemode 3");
	}

	/** Adds the recorded flight's values, with how its log was read. */
	void setMeasured(FlightLog log, FlightReplayData measured) {
		this.measured = measured;
		measuredPanel.removeAll();
		measuredPanel.add(new JSeparator(), "growx, gaptop 6, gapbottom 6");
		JLabel heading = new JLabel(trans.get("FlightReplay.readout.measured"), new Swatch(), JLabel.LEADING);
		heading.setFont(heading.getFont().deriveFont(Font.BOLD));
		measuredPanel.add(heading);
		addRow(measuredPanel, FlightDataType.TYPE_ALTITUDE.getName(), measuredAltitude);
		addRow(measuredPanel, FlightDataType.TYPE_VELOCITY_Z.getName(), measuredVelocity);
		addRow(measuredPanel, trans.get("FlightReplay.readout.difference"), altitudeDifference);

		double measuredApogee = log.apogeeHeight();
		double simulatedApogee = data.getValue(FlightDataType.TYPE_ALTITUDE, data.getApogeeTime());
		StringBuilder summary = new StringBuilder(trans.get("FlightReplay.readout.apogee") + " "
				+ format(FlightDataType.TYPE_ALTITUDE, measuredApogee) + ", T+ "
				+ format(FlightDataType.TYPE_TIME, measured.getApogeeTime()));
		if (Double.isFinite(simulatedApogee) && simulatedApogee > 0 && Double.isFinite(measuredApogee)) {
			long percent = Math.round(100 * (measuredApogee - simulatedApogee) / simulatedApogee);
			summary.append(" (").append(percent > 0 ? "+" : "").append(percent).append(" % ")
					.append(trans.get("FlightReplay.readout.vsSimulation")).append(")");
		}
		measuredPanel.add(caption(summary.toString()), "gaptop 6");

		StringBuilder notes = new StringBuilder(log.source());
		for (String note : log.notes()) {
			notes.append('\n').append(note);
		}
		measuredPanel.add(caption(notes.toString()), "gaptop 6");
		measuredPanel.setVisible(true);
		shownTime = Double.NaN;
		revalidate();
		repaint();
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
		if (measured != null) {
			double height = measured.getValue(FlightDataType.TYPE_ALTITUDE, t);
			measuredAltitude.setText(format(FlightDataType.TYPE_ALTITUDE, height));
			measuredVelocity.setText(format(FlightDataType.TYPE_VELOCITY_Z,
					measured.getValue(FlightDataType.TYPE_VELOCITY_Z, t)));
			double difference = height - data.getValue(FlightDataType.TYPE_ALTITUDE, t);
			altitudeDifference.setText((difference > 0 ? "+" : "") + format(FlightDataType.TYPE_ALTITUDE, difference));
		}
	}

	private void addRow(String caption, JLabel value) {
		addRow(this, caption, value);
	}

	private static void addRow(JPanel panel, String caption, JLabel value) {
		JLabel captionLabel = new JLabel(caption);
		captionLabel.setForeground(captionColor());
		panel.add(captionLabel, "gaptop 4");
		panel.add(value);
	}

	private static String escape(String text) {
		return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
	}

	/** Grey text wrapped to the panel's width; each line of {@code text} starts a new paragraph. */
	private static JLabel caption(String text) {
		JLabel label = new JLabel();
		label.setForeground(captionColor());
		label.setFont(label.getFont().deriveFont(label.getFont().getSize2D() - 1f));
		FontMetrics metrics = label.getFontMetrics(label.getFont());
		StringBuilder html = new StringBuilder("<html>");
		for (String paragraph : text.split("\n")) {
			StringBuilder line = new StringBuilder();
			for (String word : paragraph.split(" ")) {
				if (line.length() > 0 && metrics.stringWidth(line + " " + word) > WIDTH) {
					html.append(escape(line.toString())).append("<br>");
					line.setLength(0);
				}
				if (line.length() > 0) {
					line.append(' ');
				}
				line.append(word);
			}
			html.append(escape(line.toString())).append("<br>");
		}
		label.setText(html.append("</html>").toString());
		return label;
	}

	/** A dashed line in the measured flight's colour, as the timeline draws it. */
	private static final class Swatch implements Icon {
		@Override
		public void paintIcon(Component c, Graphics graphics, int x, int y) {
			Graphics2D g = (Graphics2D) graphics.create();
			g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
			g.setColor(FlightReplayTimeline.MEASURED);
			g.setStroke(new BasicStroke(2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND, 1f, new float[] { 6f, 4f }, 0f));
			g.drawLine(x, y + getIconHeight() / 2, x + getIconWidth(), y + getIconHeight() / 2);
			g.dispose();
		}

		@Override
		public int getIconWidth() {
			return 18;
		}

		@Override
		public int getIconHeight() {
			return 10;
		}
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
