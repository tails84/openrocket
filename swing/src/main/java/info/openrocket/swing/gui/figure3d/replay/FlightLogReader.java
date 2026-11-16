package info.openrocket.swing.gui.figure3d.replay;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.function.Predicate;

/**
 * Reads altimeter and flight computer logs saved as text tables.
 *
 * <p>Known layouts are recognised by their header row: Altus Metrum's AltosUI CSV, the Featherweight
 * Blue Raven's low-rate CSV, Eggtimer CSV, Missile Works RRC3 (mDACS) CSV and PerfectFlite CSV.
 * Any other table with a time column and an altitude column is read by its header names.</p>
 *
 * <p>Units come from the header where it states them, otherwise from the maker's documented
 * unit, otherwise from the pressure column where there is one, and as a last resort from the
 * simulated apogee. Each decision is noted on the log so the user can see it.</p>
 */
final class FlightLogReader {
	private static final double FOOT = 0.3048;
	private static final double MPH = 0.44704;
	private static final double KMH = 1 / 3.6;
	/** Height above the pad that counts as off the ground, metres. */
	private static final double OFF_PAD = 0.5;
	/** Height above the pad that surely means the rocket has launched, metres. */
	private static final double LAUNCHED = 10;
	private static final double LANDED = 3;
	private static final double SMOOTHING_SECONDS = 0.25;
	private static final int HEADER_SEARCH_LINES = 100;
	/**
	 * Fastest climb or fall a reading is allowed to imply, m/s (about Mach 2.6). Barometric heights
	 * jump by kilometres in a fraction of a second near Mach 1 and when a port is disturbed.
	 */
	private static final double MAX_SPEED = 900;
	/** Noise allowed on top of {@link #MAX_SPEED} between two readings, metres. */
	private static final double JUMP_MARGIN = 3;

	/** A log that could not be read, with a message for the user. */
	static final class LogException extends Exception {
		private static final long serialVersionUID = 1L;

		LogException(String message) {
			super(message);
		}
	}

	private FlightLogReader() {
	}

	/**
	 * Reads a log file.
	 *
	 * @param expectedApogee the simulated apogee in metres, used only when nothing else settles the
	 *                       altitude unit; NaN for none
	 */
	static FlightLog read(Path file, double expectedApogee) throws IOException, LogException {
		byte[] bytes = Files.readAllBytes(file);
		return read(file.getFileName().toString(), new String(bytes, StandardCharsets.UTF_8), expectedApogee);
	}

	static FlightLog read(String source, String text, double expectedApogee) throws LogException {
		Table table = Table.parse(text);
		List<String> notes = new ArrayList<>();
		Layout layout = Layout.detect(table);
		notes.add("Read as " + layout.name + ".");

		double[] time = table.numbers(layout.time);
		double timeScale = timeScale(table.names.get(layout.time), time, notes);
		double[] altitudeRaw = table.numbers(layout.altitude);

		// Keep rows with a time and a height, in time order.
		boolean[] keep = new boolean[time.length];
		double last = Double.NEGATIVE_INFINITY;
		int count = 0;
		for (int i = 0; i < time.length; i++) {
			if (Double.isFinite(time[i]) && Double.isFinite(altitudeRaw[i]) && time[i] > last) {
				keep[i] = true;
				last = time[i];
				count++;
			}
		}
		if (count < 10) {
			throw new LogException("The log has fewer than 10 rows with a time and an altitude.");
		}
		time = scale(select(time, keep), timeScale);
		double[] pressure = layout.pressure >= 0 ? select(table.numbers(layout.pressure), keep) : null;
		double altitudeScale = altitudeScale(table.names.get(layout.altitude), select(altitudeRaw, keep),
				pressure, layout, expectedApogee, notes);
		// A five-sample median drops the odd corrupt row before anything is worked out from it.
		double[] altitude = medianFilter(scale(select(altitudeRaw, keep), altitudeScale));
		int dropped = dropJumps(time, altitude);
		if (dropped > 0) {
			notes.add("Ignored " + dropped + " altitude readings that jump further than the rocket could travel.");
		}

		// Heights above the pad: take off the reading while it sat there.
		double[] smooth = smooth(time, altitude);
		int launched = firstIndex(smooth, h -> h > smooth[0] + LAUNCHED);
		if (launched < 0) {
			throw new LogException("The altitude never rises more than " + (int) LAUNCHED + " m, so there is "
					+ "no flight in the log.");
		}
		double pad = median(Arrays.copyOfRange(altitude, 0, Math.max(1, padEnd(smooth, launched))));
		for (int i = 0; i < altitude.length; i++) {
			altitude[i] -= pad;
			smooth[i] -= pad;
		}

		// Events the log states
		List<Double> deployments = new ArrayList<>();
		double liftoff = Double.NaN;
		double landing = Double.NaN;
		if (layout.state >= 0) {
			String[] states = select(layout.stateAnywhere ? table.words() : table.texts(layout.state), keep);
			String previous = "";
			for (int i = 0; i < states.length; i++) {
				String state = states[i].trim().toLowerCase(Locale.ROOT);
				if (!state.equals(previous)) {
					if (state.contains("boost") && Double.isNaN(liftoff)) {
						liftoff = time[i];
					} else if (state.contains("drogue") || state.contains("main") || state.contains("aux")) {
						deployments.add(time[i]);
					} else if (state.contains("land") && Double.isNaN(landing)) {
						landing = time[i];
					}
				}
				previous = state;
			}
		}
		for (int column : layout.liftoffFlags) {
			double rise = firstRise(select(table.numbers(column), keep), time);
			if (!Double.isNaN(rise) && Double.isNaN(liftoff)) {
				liftoff = rise;
			}
		}
		for (int column : layout.deploymentFlags) {
			double rise = firstRise(select(table.numbers(column), keep), time);
			if (!Double.isNaN(rise)) {
				deployments.add(rise);
			}
		}
		deployments.sort(Double::compare);

		// Events worked out from the height where the log does not state them.
		if (Double.isNaN(liftoff)) {
			int index = launched;
			while (index > 0 && smooth[index - 1] > OFF_PAD) {
				index--;
			}
			liftoff = time[index];
		}
		double launchTime = liftoff;
		// The highest reading once the odd bad ones are gone; the middle one where several read the same.
		int apogeeIndex = argmax(altitude, Math.max(0, firstIndex(time, t -> t >= launchTime)));
		int lastEqual = apogeeIndex;
		while (lastEqual + 1 < altitude.length && altitude[lastEqual + 1] == altitude[apogeeIndex]) {
			lastEqual++;
		}
		apogeeIndex = (apogeeIndex + lastEqual) / 2;
		double apogee = time[apogeeIndex];
		boolean endsAloft = smooth[time.length - 1] > LANDED;
		if (Double.isNaN(landing) && !endsAloft) {
			landing = time[time.length - 1];
			for (int i = time.length - 1; i > apogeeIndex; i--) {
				if (smooth[i] > LANDED) {
					landing = time[Math.min(time.length - 1, i + 1)];
					break;
				}
			}
		}

		if (apogeeIndex >= time.length - 2) {
			notes.add("The log ends while the rocket is still climbing.");
		} else if (Double.isNaN(landing)) {
			notes.add("The log ends before the rocket lands.");
		}

		double[] velocity = velocity(table, layout, keep, time, altitude, altitudeScale, liftoff, apogee, notes);
		double[] tilt = layout.tilt >= 0 ? fill(select(table.numbers(layout.tilt), keep)) : null;
		double[] roll = layout.roll >= 0 ? fill(select(table.numbers(layout.roll), keep)) : null;
		double[] latitude = null;
		double[] longitude = null;
		if (layout.latitude >= 0 && layout.longitude >= 0) {
			latitude = gps(select(table.numbers(layout.latitude), keep), 90);
			longitude = gps(select(table.numbers(layout.longitude), keep), 180);
			if (latitude == null || longitude == null) {
				latitude = longitude = null;
			} else {
				notes.add("Horizontal track from GPS.");
			}
		}
		if (deployments.isEmpty()) {
			notes.add("The log records no deployments; they are placed where the simulation's happen.");
		}
		return new FlightLog(source, layout.name, time, altitude, velocity, tilt, roll, latitude, longitude,
				liftoff, apogee, landing, deployments, notes);
	}

	// ---------- units ----------

	private static double timeScale(String name, double[] time, List<String> notes) {
		String unit = unitOf(name);
		if (unit.equals("ms")) {
			return 0.001;
		}
		if (unit.equals("s")) {
			return 1;
		}
		// Eggtimer's Classic and TRS write milliseconds under the same name as the seconds of
		// later models; a step far longer than any logger's in seconds means milliseconds.
		double step = medianStep(time);
		if (step > 5) {
			notes.add("Time read in milliseconds from its step of " + Math.round(step) + ".");
			return 0.001;
		}
		return 1;
	}

	private static double altitudeScale(String name, double[] altitude, double[] pressure, Layout layout,
			double expectedApogee, List<String> notes) {
		double stated = lengthUnit(unitOf(name));
		if (!Double.isNaN(stated)) {
			return stated;
		}
		if (!Double.isNaN(layout.altitudeScale)) {
			notes.add("Altitude in " + (layout.altitudeScale == FOOT ? "feet" : "metres") + ", as "
					+ layout.name + " records it.");
			return layout.altitudeScale;
		}
		double rise = max(altitude) - altitude[0];
		if (pressure != null) {
			double climb = pressureClimb(pressure);
			if (climb > 30) {
				double scale = Math.abs(Math.log(rise * FOOT / climb)) < Math.abs(Math.log(rise / climb)) ? FOOT : 1;
				notes.add("Altitude in " + (scale == FOOT ? "feet" : "metres") + ", matched to the pressure.");
				return scale;
			}
		}
		if (Double.isFinite(expectedApogee) && expectedApogee > 0 && rise > 0) {
			double scale = Math.abs(Math.log(rise * FOOT / expectedApogee)) < Math.abs(Math.log(rise / expectedApogee))
					? FOOT : 1;
			notes.add("Altitude unit not stated: read in " + (scale == FOOT ? "feet" : "metres")
					+ ", whichever is closer to the simulated apogee.");
			return scale;
		}
		notes.add("Altitude unit not stated: read in metres.");
		return 1;
	}

	/** Highest climb the pressure shows in the standard atmosphere, metres. */
	private static double pressureClimb(double[] pressure) {
		double pad = Double.NaN;
		double lowest = Double.POSITIVE_INFINITY;
		for (double p : pressure) {
			if (Double.isFinite(p) && p > 0) {
				if (Double.isNaN(pad)) {
					pad = p;
				}
				lowest = Math.min(lowest, p);
			}
		}
		if (Double.isNaN(pad)) {
			return 0;
		}
		return 44330.77 * (1 - Math.pow(lowest / pad, 0.190263));
	}

	/**
	 * Vertical speed: the log's own column where its unit can be settled against the height,
	 * otherwise the height's rate of change.
	 */
	private static double[] velocity(Table table, Layout layout, boolean[] keep, double[] time, double[] altitude,
			double altitudeScale, double liftoff, double apogee, List<String> notes) {
		double[] derived = derivative(time, altitude);
		if (layout.velocity < 0) {
			notes.add("Vertical speed worked out from the altitude.");
			return derived;
		}
		double[] raw = medianFilter(fill(select(table.numbers(layout.velocity), keep)));
		double stated = speedUnit(unitOf(table.names.get(layout.velocity)));
		double[] candidates = Double.isNaN(stated)
				? new double[] { altitudeScale, 1, FOOT, MPH, KMH }
				: new double[] { stated };
		for (double factor : candidates) {
			if (agrees(raw, factor, derived, time, liftoff, apogee)) {
				return scale(raw, factor);
			}
		}
		notes.add("The log's speed column does not match its altitude; vertical speed worked out from the "
				+ "altitude instead.");
		return derived;
	}

	/** Whether a speed column, scaled, follows the climb rate on most of the ascent. */
	private static boolean agrees(double[] speed, double factor, double[] derived, double[] time,
			double liftoff, double apogee) {
		List<Double> ratios = new ArrayList<>();
		for (int i = 0; i < time.length; i++) {
			if (time[i] > liftoff && time[i] < apogee && Math.abs(derived[i]) > 10 && Double.isFinite(speed[i])) {
				ratios.add(speed[i] * factor / derived[i]);
			}
		}
		if (ratios.size() < 5) {
			return false;
		}
		double ratio = median(ratios.stream().mapToDouble(Double::doubleValue).toArray());
		return ratio > 0.8 && ratio < 1.25;
	}

	private static String unitOf(String name) {
		String n = name.toLowerCase(Locale.ROOT).trim();
		int open = Math.max(n.lastIndexOf('('), n.lastIndexOf('['));
		if (open >= 0 && (n.endsWith(")") || n.endsWith("]"))) {
			return n.substring(open + 1, n.length() - 1).trim();
		}
		int underscore = n.lastIndexOf('_');
		if (underscore >= 0) {
			return n.substring(underscore + 1);
		}
		return "";
	}

	private static double lengthUnit(String unit) {
		return switch (unit) {
			case "ft", "feet", "foot" -> FOOT;
			case "m", "meters", "metres", "meter", "metre" -> 1;
			default -> Double.NaN;
		};
	}

	private static double speedUnit(String unit) {
		return switch (unit) {
			case "ft/s", "fps", "ft/sec", "feet/s" -> FOOT;
			case "m/s", "mps", "m/sec" -> 1;
			case "mph" -> MPH;
			case "km/h", "kph", "kmh" -> KMH;
			default -> Double.NaN;
		};
	}

	// ---------- events and numbers ----------

	/** The first time a flag column turns non-zero, or NaN when it never does. */
	private static double firstRise(double[] flag, double[] time) {
		for (int i = 0; i < flag.length; i++) {
			if (Double.isFinite(flag[i]) && flag[i] != 0) {
				return time[i];
			}
		}
		return Double.NaN;
	}

	/** Index just past the samples on the pad before the climb that reaches {@code launched}. */
	private static int padEnd(double[] smooth, int launched) {
		int index = launched;
		while (index > 0 && smooth[index - 1] - smooth[0] > OFF_PAD) {
			index--;
		}
		return index;
	}

	/**
	 * Replaces readings the rocket could not have reached from the readings around them with a
	 * straight line between the readings kept. A pass forward and a pass back each keep a reading
	 * when it lies within reach of the last one they kept; a reading stays only when both keep it,
	 * so a run of bad readings is dropped whichever end it starts from.
	 *
	 * @return how many readings were replaced
	 */
	static int dropJumps(double[] time, double[] values) {
		boolean[] forward = withinReach(time, values, true);
		boolean[] backward = withinReach(time, values, false);
		int dropped = 0;
		int previous = -1;
		for (int i = 0; i < values.length; i++) {
			if (!forward[i] || !backward[i]) {
				dropped++;
				continue;
			}
			for (int j = previous + 1; j < i; j++) {
				values[j] = previous < 0 ? values[i]
						: values[previous] + (values[i] - values[previous])
								* (time[j] - time[previous]) / (time[i] - time[previous]);
			}
			previous = i;
		}
		for (int j = previous + 1; j < values.length && previous >= 0; j++) {
			values[j] = values[previous];
		}
		return dropped;
	}

	private static boolean[] withinReach(double[] time, double[] values, boolean forward) {
		boolean[] kept = new boolean[values.length];
		int last = -1;
		for (int k = 0; k < values.length; k++) {
			int i = forward ? k : values.length - 1 - k;
			if (last < 0 || Math.abs(values[i] - values[last])
					<= JUMP_MARGIN + MAX_SPEED * Math.abs(time[i] - time[last])) {
				kept[i] = true;
				last = i;
			}
		}
		return kept;
	}

	static double[] medianFilter(double[] values) {
		double[] out = new double[values.length];
		double[] window = new double[5];
		for (int i = 0; i < values.length; i++) {
			int n = 0;
			for (int j = Math.max(0, i - 2); j <= Math.min(values.length - 1, i + 2); j++) {
				window[n++] = values[j];
			}
			double[] sorted = Arrays.copyOf(window, n);
			Arrays.sort(sorted);
			out[i] = sorted[n / 2];
		}
		return out;
	}

	private static double[] smooth(double[] time, double[] values) {
		double[] out = new double[values.length];
		int lo = 0, hi = 0;
		double sum = 0;
		for (int i = 0; i < values.length; i++) {
			while (hi < values.length && time[hi] <= time[i] + SMOOTHING_SECONDS) {
				sum += values[hi++];
			}
			while (time[lo] < time[i] - SMOOTHING_SECONDS) {
				sum -= values[lo++];
			}
			out[i] = sum / (hi - lo);
		}
		return out;
	}

	private static double[] derivative(double[] time, double[] values) {
		double[] smooth = smooth(time, values);
		double[] out = new double[values.length];
		for (int i = 0; i < values.length; i++) {
			int a = i, b = i;
			while (a > 0 && time[i] - time[a] < SMOOTHING_SECONDS) {
				a--;
			}
			while (b < values.length - 1 && time[b] - time[i] < SMOOTHING_SECONDS) {
				b++;
			}
			out[i] = b > a ? (smooth[b] - smooth[a]) / (time[b] - time[a]) : 0;
		}
		return out;
	}

	/** GPS column with gaps filled, or null when it holds no fix. */
	private static double[] gps(double[] values, double limit) {
		double[] cleaned = values.clone();
		for (int i = 0; i < cleaned.length; i++) {
			if (!Double.isFinite(cleaned[i]) || cleaned[i] == 0 || Math.abs(cleaned[i]) > limit) {
				cleaned[i] = Double.NaN;
			}
		}
		return FlightReplayData.fillGaps(cleaned, cleaned.length);
	}

	private static double[] fill(double[] values) {
		double[] filled = FlightReplayData.fillGaps(values, values.length);
		return filled != null ? filled : new double[values.length];
	}

	private static double[] select(double[] values, boolean[] keep) {
		double[] out = new double[count(keep)];
		for (int i = 0, j = 0; i < keep.length; i++) {
			if (keep[i]) {
				out[j++] = values[i];
			}
		}
		return out;
	}

	private static String[] select(String[] values, boolean[] keep) {
		String[] out = new String[count(keep)];
		for (int i = 0, j = 0; i < keep.length; i++) {
			if (keep[i]) {
				out[j++] = values[i];
			}
		}
		return out;
	}

	private static int count(boolean[] keep) {
		int n = 0;
		for (boolean k : keep) {
			n += k ? 1 : 0;
		}
		return n;
	}

	private static double[] scale(double[] values, double factor) {
		double[] out = new double[values.length];
		for (int i = 0; i < values.length; i++) {
			out[i] = values[i] * factor;
		}
		return out;
	}

	private static int firstIndex(double[] values, Predicate<Double> test) {
		for (int i = 0; i < values.length; i++) {
			if (test.test(values[i])) {
				return i;
			}
		}
		return -1;
	}

	private static int argmax(double[] values, int from) {
		int best = Math.max(0, from);
		for (int i = best; i < values.length; i++) {
			if (values[i] > values[best]) {
				best = i;
			}
		}
		return best;
	}

	private static double max(double[] values) {
		double max = Double.NEGATIVE_INFINITY;
		for (double v : values) {
			max = Math.max(max, v);
		}
		return max;
	}

	static double median(double[] values) {
		double[] sorted = Arrays.stream(values).filter(Double::isFinite).sorted().toArray();
		if (sorted.length == 0) {
			return 0;
		}
		int mid = sorted.length / 2;
		return sorted.length % 2 == 1 ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
	}

	private static double medianStep(double[] time) {
		List<Double> steps = new ArrayList<>();
		double previous = Double.NaN;
		for (double t : time) {
			if (Double.isFinite(t)) {
				if (Double.isFinite(previous) && t > previous) {
					steps.add(t - previous);
				}
				previous = t;
			}
		}
		return median(steps.stream().mapToDouble(Double::doubleValue).toArray());
	}

	// ---------- table and layouts ----------

	/** Header names and cells of a delimited text table. */
	private static final class Table {
		final List<String> names;
		final List<String[]> rows;

		private Table(List<String> names, List<String[]> rows) {
			this.names = names;
			this.rows = rows;
		}

		static Table parse(String text) throws LogException {
			String[] lines = text.replace("﻿", "").split("\\R");
			int header = -1;
			for (int i = 0; i < Math.min(lines.length, HEADER_SEARCH_LINES); i++) {
				String line = lines[i].trim();
				if (line.isEmpty() || line.startsWith("sep=")) {
					continue;
				}
				// AltosUI writes its header as a comment line; other comment lines are skipped.
				if (line.startsWith("#") && !line.toLowerCase(Locale.ROOT).startsWith("#version")) {
					continue;
				}
				header = i;
				break;
			}
			if (header < 0) {
				throw new LogException("No header row was found.");
			}
			String headerLine = lines[header].trim();
			if (headerLine.startsWith("#")) {
				headerLine = headerLine.substring(1);
			}
			String separator = separatorOf(headerLine);
			List<String> names = new ArrayList<>();
			for (String name : headerLine.split(separator, -1)) {
				names.add(name.trim().replace("\"", ""));
			}
			if (names.size() < 2) {
				throw new LogException("The header row names fewer than two columns.");
			}
			List<String[]> rows = new ArrayList<>();
			for (int i = header + 1; i < lines.length; i++) {
				String line = lines[i].trim();
				if (line.isEmpty() || line.startsWith("#")) {
					continue;
				}
				rows.add(line.split(separator, -1));
			}
			return new Table(names, rows);
		}

		private static String separatorOf(String header) {
			String best = ",";
			int most = 0;
			for (String separator : new String[] { ",", ";", "\t", "\\|" }) {
				int cells = header.split(separator, -1).length;
				if (cells > most) {
					most = cells;
					best = separator;
				}
			}
			return best;
		}

		int column(Predicate<String> test) {
			for (int i = 0; i < names.size(); i++) {
				if (test.test(names.get(i).toLowerCase(Locale.ROOT))) {
					return i;
				}
			}
			return -1;
		}

		double[] numbers(int column) {
			double[] out = new double[rows.size()];
			for (int i = 0; i < out.length; i++) {
				String[] row = rows.get(i);
				out[i] = column < row.length ? number(row[column]) : Double.NaN;
			}
			return out;
		}

		/** Each row's cells that are not numbers, joined; for loggers that drop columns from some rows. */
		String[] words() {
			String[] out = new String[rows.size()];
			for (int i = 0; i < out.length; i++) {
				StringBuilder words = new StringBuilder();
				for (String cell : rows.get(i)) {
					String text = cell.replace("\"", "").trim();
					if (!text.isEmpty() && Double.isNaN(number(text))) {
						words.append(text).append(' ');
					}
				}
				out[i] = words.toString().trim();
			}
			return out;
		}

		String[] texts(int column) {
			String[] out = new String[rows.size()];
			for (int i = 0; i < out.length; i++) {
				String[] row = rows.get(i);
				out[i] = column < row.length ? row[column].replace("\"", "").trim() : "";
			}
			return out;
		}

		private static double number(String cell) {
			String text = cell.replace("\"", "").trim();
			if (text.isEmpty()) {
				return Double.NaN;
			}
			// A decimal comma, as a European locale's spreadsheet writes beside semicolons.
			if (text.indexOf('.') < 0 && text.indexOf(',') >= 0) {
				text = text.replace(',', '.');
			}
			try {
				return Double.parseDouble(text);
			} catch (NumberFormatException e) {
				return Double.NaN;
			}
		}
	}

	/** Which columns hold what, for one logger layout. */
	private static final class Layout {
		String name;
		int time = -1;
		int altitude = -1;
		int velocity = -1;
		int pressure = -1;
		int tilt = -1;
		int roll = -1;
		int latitude = -1;
		int longitude = -1;
		int state = -1;
		/** The state is whichever cell of a row holds a word, not a fixed column. */
		boolean stateAnywhere;
		final List<Integer> liftoffFlags = new ArrayList<>();
		final List<Integer> deploymentFlags = new ArrayList<>();
		/** The maker's altitude unit in metres, or NaN when the layout does not fix one. */
		double altitudeScale = Double.NaN;

		static Layout detect(Table table) throws LogException {
			List<String> keys = table.names.stream().map(n -> n.toLowerCase(Locale.ROOT)).toList();
			Layout layout = new Layout();
			if (keys.contains("state_name") && keys.contains("height") && keys.contains("time")) {
				layout.name = "Altus Metrum (AltosUI)";
				layout.time = keys.indexOf("time");
				layout.altitude = keys.indexOf("height");
				layout.altitudeScale = 1;
				layout.velocity = firstOf(keys, "speed", "baro_speed", "accel_speed");
				layout.state = keys.indexOf("state_name");
				layout.latitude = keys.indexOf("latitude");
				layout.longitude = keys.indexOf("longitude");
			} else if (keys.contains("flight_time_(s)") && keys.contains("baro_altitude_agl_(feet)")) {
				layout.name = "Featherweight Blue Raven";
				layout.time = keys.indexOf("flight_time_(s)");
				layout.altitude = keys.indexOf("baro_altitude_agl_(feet)");
				layout.velocity = keys.indexOf("velocity_up");
				layout.tilt = keys.indexOf("tilt_angle_(deg)");
				layout.roll = keys.indexOf("roll_angle_(deg)");
				addIfPresent(layout.liftoffFlags, keys.indexOf("liftoff"));
				addIfPresent(layout.deploymentFlags, keys.indexOf("apo_fired"));
				addIfPresent(layout.deploymentFlags, keys.indexOf("main_fired"));
			} else if (keys.size() > 1 && keys.get(0).equals("t") && keys.get(1).equals("alt")) {
				layout.name = "Eggtimer";
				layout.time = 0;
				int filtered = firstOf(keys, "falt", "filt_alt", "kalt");
				layout.altitude = filtered >= 0 ? filtered : 1;
				layout.altitudeScale = FOOT;
				layout.velocity = firstOf(keys, "fveloc", "kveloc", "vfilt", "veloc", "vraw");
				addIfPresent(layout.deploymentFlags, keys.indexOf("drogue"));
				addIfPresent(layout.deploymentFlags, keys.indexOf("main"));
			} else if (startsWith(keys, "time") && keys.contains("pressure") && keys.contains("events")
					&& startsWith(keys, "altitude")) {
				layout.name = "Missile Works RRC3 (mDACS)";
				layout.time = indexStarting(keys, "time");
				layout.altitude = indexStarting(keys, "altitude");
				layout.pressure = indexStarting(keys, "pressure");
				layout.velocity = indexStarting(keys, "velocity");
				layout.state = keys.indexOf("events");
				// mDACS leaves out the temperature and voltage on some rows, which moves the event left.
				layout.stateAnywhere = true;
			} else if (keys.size() >= 3 && keys.get(0).equals("time") && keys.get(1).equals("altitude")
					&& keys.get(2).equals("velocity") && startsWith(keys, "temperature")) {
				layout.name = "PerfectFlite";
				layout.time = 0;
				layout.altitude = 1;
				layout.altitudeScale = FOOT;
				layout.velocity = 2;
			} else {
				layout.name = "a table read by its column names";
				layout.time = table.column(k -> k.equals("t") || k.startsWith("t ") || k.startsWith("t(")
						|| k.startsWith("t_") || k.contains("time"));
				layout.altitude = table.column(k -> (k.contains("alt") || k.contains("height")) && k.contains("agl"));
				if (layout.altitude < 0) {
					layout.altitude = table.column(k -> k.contains("height") || k.contains("alt"));
				}
				layout.velocity = table.column(k -> (k.contains("vel") || k.contains("speed"))
						&& !k.contains("x") && !k.contains("horiz"));
				layout.pressure = table.column(k -> k.contains("press"));
				layout.tilt = table.column(k -> k.contains("tilt"));
				layout.latitude = table.column(k -> k.startsWith("lat"));
				layout.longitude = table.column(k -> k.startsWith("lon") || k.startsWith("lng"));
				layout.state = table.column(k -> k.contains("state") || k.contains("event") || k.contains("phase"));
			}
			if (layout.time < 0 || layout.altitude < 0) {
				throw new LogException("The log needs a time column and an altitude column; the header names "
						+ String.join(", ", table.names) + ".");
			}
			return layout;
		}

		private static int firstOf(List<String> keys, String... names) {
			for (String name : names) {
				int index = keys.indexOf(name);
				if (index >= 0) {
					return index;
				}
			}
			return -1;
		}

		private static boolean startsWith(List<String> keys, String prefix) {
			return indexStarting(keys, prefix) >= 0;
		}

		private static int indexStarting(List<String> keys, String prefix) {
			for (int i = 0; i < keys.size(); i++) {
				if (keys.get(i).startsWith(prefix)) {
					return i;
				}
			}
			return -1;
		}

		private static void addIfPresent(List<Integer> list, int index) {
			if (index >= 0) {
				list.add(index);
			}
		}
	}
}
