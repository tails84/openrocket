package info.openrocket.swing.gui.figure3d.animation;

import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

/**
 * Samples a {@link FlightDataBranch} as an engine-space position and orientation.
 * Position uses east, north, and altitude data; orientation uses theta and phi
 * when available and otherwise follows the sampled velocity.
 *
 * <p>The simulation stores theta as the elevation of the rocket axis above the
 * horizon and phi, like the lateral position direction, as a bearing measured
 * clockwise from north (see {@code SimulationStatus.storeData()}).</p>
 */
public final class FlightPoseProvider implements PoseProvider {
	private final double[] t;
	private final double[] east, north, alt;
	// Engine-space unit vectors of the rocket axis, or null to follow the velocity.
	// Interpolating vectors avoids the jump where the bearing wraps through north.
	private final float[] axisX, axisY, axisZ;

	private FlightPoseProvider(double[] t, double[] east, double[] north, double[] alt,
							   double[] elevation, double[] bearing) {
		this.t = t; this.east = east; this.north = north; this.alt = alt;
		if (elevation != null && bearing != null) {
			axisX = new float[t.length];
			axisY = new float[t.length];
			axisZ = new float[t.length];
			fillAxis(elevation, bearing);
		} else {
			axisX = axisY = axisZ = null;
		}
	}

	// ---------- Factory ----------
	public static FlightPoseProvider fromFlightDataBranch(FlightDataBranch branch) {
		// Required channels
		List<Double> tL     = branch.get(FlightDataType.TYPE_TIME);
		if (tL == null || tL.isEmpty()) throw new IllegalArgumentException("FlightDataBranch has no TIME channel");

		// Position: prefer explicit east/north; fallback to XY + direction
		List<Double> eastL  = branch.get(FlightDataType.TYPE_POSITION_X);
		List<Double> northL = branch.get(FlightDataType.TYPE_POSITION_Y);

		if (eastL == null || northL == null) {
			List<Double> rXY  = branch.get(FlightDataType.TYPE_POSITION_XY);
			List<Double> dirL = branch.get(FlightDataType.TYPE_POSITION_DIRECTION); // bearing from north, clockwise
			if (rXY != null && dirL != null) {
				eastL  = new ArrayList<>(rXY.size());
				northL = new ArrayList<>(rXY.size());
				for (int i = 0; i < Math.min(rXY.size(), dirL.size()); i++) {
					double r = nz(rXY.get(i));
					double bearing = nz(dirL.get(i));
					eastL.add(r * Math.sin(bearing));
					northL.add(r * Math.cos(bearing));
				}
			}
		}

		if (eastL == null || northL == null)
			throw new IllegalArgumentException("FlightDataBranch lacks lateral position (Px/Py or XY+dir)");

		List<Double> altL = nvl(branch.get(FlightDataType.TYPE_ALTITUDE),
				branch.get(FlightDataType.TYPE_ALTITUDE_ABOVE_SEA));
		if (altL == null) throw new IllegalArgumentException("FlightDataBranch lacks altitude channel");

		// Optional orientation
		List<Double> thetaL = branch.get(FlightDataType.TYPE_ORIENTATION_THETA); // elevation, pi/2 = up
		List<Double> phiL   = branch.get(FlightDataType.TYPE_ORIENTATION_PHI);   // bearing from north, clockwise

		// Length align (defensive)
		int n = minLen(tL, eastL, northL, altL);
		if (thetaL != null) n = Math.min(n, thetaL.size());
		if (phiL   != null) n = Math.min(n, phiL.size());

		return new FlightPoseProvider(
				toPrimitive(tL, n),
				toPrimitive(eastL, n),
				toPrimitive(northL, n),
				toPrimitive(altL, n),
				(thetaL != null ? toPrimitive(thetaL, n) : null),
				(phiL   != null ? toPrimitive(phiL,   n) : null)
		);
	}

	// ---------- PoseProvider ----------
	@Override
	public Vector3f getPosition(double time) {
		float ex = sample(t, east,  time);
		float no = sample(t, north, time);
		float al = sample(t, alt,   time);
		return SimToWorld.toEngine(ex, no, al);
	}

	@Override
	public Quaternionf getOrientation(double time) {
		Vector3f dir = getAxisDirection(time);
		if (dir == null) return new Quaternionf(); // no rotation
		return new Quaternionf().rotateTo(new Vector3f(0,1,0), dir);
	}

	/**
	 * Returns the engine-space unit vector the rocket nose points along at the given time,
	 * or {@code null} when neither orientation data nor a usable velocity is available.
	 */
	public Vector3f getAxisDirection(double time) {
		if (axisX != null) {
			Vector3f dir = new Vector3f(sample(t, axisX, time), sample(t, axisY, time), sample(t, axisZ, time));
			if (dir.lengthSquared() > 1e-12f) {
				return dir.normalize();
			}
		}

		// Fallback: face the velocity (central difference of position)
		final double eps = 0.02; // 20 ms
		Vector3f p0 = getPosition(Math.max(getStartTime(), time - eps));
		Vector3f p1 = getPosition(Math.min(getEndTime(),   time + eps));
		Vector3f v = p1.sub(p0, new Vector3f());
		if (v.lengthSquared() < 1e-12f) return null;
		return v.normalize();
	}

	@Override public double getStartTime() { return t[0]; }
	@Override public double getEndTime()   { return t[t.length - 1]; }

	// ---------- helpers ----------
	private void fillAxis(double[] elevation, double[] bearing) {
		float lastX = Float.NaN, lastY = Float.NaN, lastZ = Float.NaN;
		for (int i = 0; i < t.length; i++) {
			double el = elevation[i], br = bearing[i];
			if (Double.isFinite(el) && Double.isFinite(br)) {
				// Engine axes: +X east, +Y up, -Z north
				lastX = (float) (Math.cos(el) * Math.sin(br));
				lastY = (float) Math.sin(el);
				lastZ = (float) (-Math.cos(el) * Math.cos(br));
			}
			axisX[i] = lastX; axisY[i] = lastY; axisZ[i] = lastZ;
		}
		// Samples before the first finite reading take that reading.
		int first = 0;
		while (first < t.length && Float.isNaN(axisX[first])) first++;
		for (int i = 0; i < first && first < t.length; i++) {
			axisX[i] = axisX[first]; axisY[i] = axisY[first]; axisZ[i] = axisZ[first];
		}
	}

	private static int minLen(List<?>... lists) {
		int n = Integer.MAX_VALUE;
		for (List<?> l : lists) n = Math.min(n, l.size());
		return n;
	}

	private static List<Double> nvl(List<Double> a, List<Double> b) { return (a != null ? a : b); }
	private static double nz(Double d) { return (d != null ? d : Double.NaN); }

	private static double[] toPrimitive(List<Double> src, int n) {
		double[] out = new double[n];
		for (int i = 0; i < n; i++) out[i] = nz(src.get(i));
		return out;
	}

	private static float sample(double[] ts, double[] vs, double t) {
		if (t <= ts[0]) return (float)vs[0];
		int hi = upperBound(ts, t);
		if (hi <= 0) return (float)vs[0];
		if (hi >= ts.length) return (float)vs[ts.length - 1];
		int lo = hi - 1;
		double a = ts[lo], b = ts[hi];
		double u = (t - a) / Math.max(1e-9, (b - a));
		return (float)((1.0 - u) * vs[lo] + u * vs[hi]);
	}

	private static float sample(double[] ts, float[] vs, double t) {
		if (t <= ts[0]) return vs[0];
		int hi = upperBound(ts, t);
		if (hi <= 0) return vs[0];
		if (hi >= ts.length) return vs[ts.length - 1];
		int lo = hi - 1;
		double a = ts[lo], b = ts[hi];
		double u = (t - a) / Math.max(1e-9, (b - a));
		return (float)((1.0 - u) * vs[lo] + u * vs[hi]);
	}

	private static int upperBound(double[] ts, double t) {
		int lo = 0, hi = ts.length;
		while (lo < hi) {
			int mid = (lo + hi) >>> 1;
			if (t >= ts[mid]) lo = mid + 1; else hi = mid;
		}
		return lo;
	}
}
