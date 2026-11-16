package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;

import javax.swing.JComponent;
import javax.swing.ToolTipManager;
import javax.swing.UIManager;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * Scrub bar for the flight replay: the altitude profile with lettered event markers and a
 * draggable playhead.
 */
final class FlightReplayTimeline extends JComponent {
	private static final long serialVersionUID = 1L;
	private static final int MARKER_SIZE = 15;
	private static final int MARKER_ROWS = 2;
	private static final int PAD = 8;

	// Okabe-Ito palette; each marker also carries a letter.
	private static final Color PROFILE = new Color(0x00, 0x72, 0xB2);

	private final FlightReplayData data;
	private final double start;
	private final double end;
	private final double[] times;
	private final double[] altitudes;
	private final double maxAltitude;
	private final DoubleConsumer seekListener;
	private double time;
	private final List<PlacedMarker> placedMarkers = new ArrayList<>();

	private record PlacedMarker(int x, int y, FlightReplayData.Event event) {
	}

	FlightReplayTimeline(FlightReplayData data, double start, double end, DoubleConsumer seekListener) {
		this.data = data;
		this.start = start;
		this.end = end;
		this.seekListener = seekListener;
		this.times = data.getTimes();
		this.altitudes = new double[times.length];
		double max = 0;
		for (int i = 0; i < times.length; i++) {
			altitudes[i] = data.getValue(FlightDataType.TYPE_ALTITUDE, times[i]);
			if (Double.isFinite(altitudes[i])) {
				max = Math.max(max, altitudes[i]);
			}
		}
		this.maxAltitude = max > 0 ? max : 1;
		this.time = start;
		setPreferredSize(new Dimension(600, 30 + MARKER_ROWS * (MARKER_SIZE + 2) + 40));
		setMinimumSize(new Dimension(200, 80));
		setFocusable(false);
		ToolTipManager.sharedInstance().registerComponent(this);

		MouseAdapter scrub = new MouseAdapter() {
			@Override
			public void mousePressed(MouseEvent e) {
				seekTo(e.getX());
			}

			@Override
			public void mouseDragged(MouseEvent e) {
				seekTo(e.getX());
			}
		};
		addMouseListener(scrub);
		addMouseMotionListener(scrub);
	}

	void setTime(double time) {
		if (time != this.time) {
			this.time = time;
			repaint();
		}
	}

	private void seekTo(int x) {
		double fraction = (x - PAD) / (double) Math.max(1, getWidth() - 2 * PAD);
		seekListener.accept(start + Math.max(0, Math.min(1, fraction)) * (end - start));
	}

	private int xOf(double t) {
		double fraction = (t - start) / Math.max(1e-9, end - start);
		return PAD + (int) Math.round(fraction * (getWidth() - 2 * PAD));
	}

	@Override
	public String getToolTipText(MouseEvent e) {
		for (PlacedMarker marker : placedMarkers) {
			if (Math.abs(e.getX() - marker.x()) <= MARKER_SIZE / 2 + 1
					&& Math.abs(e.getY() - marker.y()) <= MARKER_SIZE / 2 + 1) {
				return describe(marker.event());
			}
		}
		return null;
	}

	static String describe(FlightReplayData.Event event) {
		String time = FlightDataType.TYPE_TIME.getUnitGroup().getDefaultUnit().toStringUnit(event.time());
		String source = event.source() != null && event.type() == FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT
				? " (" + event.source().getName() + ")" : "";
		return event.type().toString() + source + ", T+ " + time;
	}

	@Override
	protected void paintComponent(Graphics graphics) {
		Graphics2D g = (Graphics2D) graphics.create();
		try {
			g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
			g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
			Color foreground = uiColor("Label.foreground", Color.DARK_GRAY);
			Color background = uiColor("TextField.background", Color.WHITE);
			Color border = uiColor("Component.borderColor", Color.GRAY);

			int markerBand = MARKER_ROWS * (MARKER_SIZE + 2);
			int top = PAD + markerBand;
			int bottom = getHeight() - PAD;
			int left = PAD, right = getWidth() - PAD;
			if (right <= left || bottom <= top) {
				return;
			}

			g.setColor(background);
			g.fillRoundRect(left, top, right - left, bottom - top, 6, 6);

			// Altitude profile: the flown part solid, the rest faint.
			Path2D profile = new Path2D.Double();
			profile.moveTo(xOf(times[0]), bottom);
			for (int i = 0; i < times.length; i++) {
				double alt = Double.isFinite(altitudes[i]) ? altitudes[i] : 0;
				profile.lineTo(xOf(times[i]), bottom - (alt / maxAltitude) * (bottom - top - 4));
			}
			profile.lineTo(xOf(times[times.length - 1]), bottom);
			profile.closePath();
			int playX = xOf(time);
			g.setColor(withAlpha(PROFILE, 60));
			g.fill(profile);
			Graphics2D flown = (Graphics2D) g.create();
			flown.clipRect(left, top, playX - left, bottom - top);
			flown.setColor(withAlpha(PROFILE, 150));
			flown.fill(profile);
			flown.dispose();
			g.setColor(PROFILE);
			g.setStroke(new BasicStroke(1.2f));
			g.draw(profile);

			g.setColor(border);
			g.setStroke(new BasicStroke(1f));
			g.drawRoundRect(left, top, right - left, bottom - top, 6, 6);

			paintMarkers(g, foreground, top);

			// Playhead
			g.setColor(foreground);
			g.setStroke(new BasicStroke(2f));
			g.drawLine(playX, top - 2, playX, bottom);
			int[] xs = { playX - 5, playX + 5, playX };
			int[] ys = { top - 8, top - 8, top - 1 };
			g.fillPolygon(xs, ys, 3);
		} finally {
			g.dispose();
		}
	}

	private void paintMarkers(Graphics2D g, Color foreground, int top) {
		placedMarkers.clear();
		Font font = getFont().deriveFont(Font.BOLD, 10f);
		g.setFont(font);
		FontMetrics metrics = g.getFontMetrics();
		int[] lastXInRow = new int[MARKER_ROWS];
		java.util.Arrays.fill(lastXInRow, Integer.MIN_VALUE / 2);
		for (FlightReplayData.Event event : data.getEvents()) {
			int x = xOf(event.time());
			int row = 0;
			while (row < MARKER_ROWS - 1 && x - lastXInRow[row] < MARKER_SIZE + 2) {
				row++;
			}
			lastXInRow[row] = x;
			int y = PAD + row * (MARKER_SIZE + 2) + MARKER_SIZE / 2;

			Color color = markerColor(event.type());
			g.setColor(withAlpha(color, 140));
			g.setStroke(new BasicStroke(1f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1f, new float[] { 2f, 2f }, 0f));
			g.drawLine(x, y + MARKER_SIZE / 2, x, getHeight() - PAD);

			g.setColor(color);
			g.fillOval(x - MARKER_SIZE / 2, y - MARKER_SIZE / 2, MARKER_SIZE, MARKER_SIZE);
			g.setColor(foreground);
			g.setStroke(new BasicStroke(1f));
			g.drawOval(x - MARKER_SIZE / 2, y - MARKER_SIZE / 2, MARKER_SIZE, MARKER_SIZE);
			String letter = markerLetter(event.type());
			g.setColor(isLight(color) ? Color.BLACK : Color.WHITE);
			g.drawString(letter, x - metrics.stringWidth(letter) / 2, y + metrics.getAscent() / 2 - 1);
			placedMarkers.add(new PlacedMarker(x, y, event));
		}
	}

	static String markerLetter(FlightEvent.Type type) {
		return switch (type) {
			case LIFTOFF -> "L";
			case LAUNCHROD -> "R";
			case IGNITION -> "I";
			case BURNOUT -> "B";
			case STAGE_SEPARATION -> "S";
			case APOGEE -> "A";
			case RECOVERY_DEVICE_DEPLOYMENT -> "D";
			case TUMBLE -> "T";
			case GROUND_HIT -> "G";
			default -> "?";
		};
	}

	static Color markerColor(FlightEvent.Type type) {
		return switch (type) {
			case LIFTOFF -> new Color(0x00, 0x9E, 0x73);
			case LAUNCHROD -> new Color(0x56, 0xB4, 0xE9);
			case IGNITION, STAGE_SEPARATION -> new Color(0xCC, 0x79, 0xA7);
			case BURNOUT -> new Color(0xE6, 0x9F, 0x00);
			case APOGEE -> new Color(0x00, 0x72, 0xB2);
			case RECOVERY_DEVICE_DEPLOYMENT -> new Color(0xD5, 0x5E, 0x00);
			case TUMBLE -> new Color(0xF0, 0xE4, 0x42);
			default -> new Color(0x66, 0x66, 0x66);
		};
	}

	private static boolean isLight(Color color) {
		return 0.299 * color.getRed() + 0.587 * color.getGreen() + 0.114 * color.getBlue() > 150;
	}

	private static Color withAlpha(Color color, int alpha) {
		return new Color(color.getRed(), color.getGreen(), color.getBlue(), alpha);
	}

	private static Color uiColor(String key, Color fallback) {
		Color color = UIManager.getColor(key);
		return color != null ? color : fallback;
	}
}
