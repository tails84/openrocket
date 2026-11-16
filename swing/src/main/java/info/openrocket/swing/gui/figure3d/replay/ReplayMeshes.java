package info.openrocket.swing.gui.figure3d.replay;

import info.openrocket.swing.gui.figure3d.constants.RenderingConstants;
import info.openrocket.swing.gui.figure3d.geometry.IntList;
import info.openrocket.swing.gui.figure3d.geometry.Mesh;
import info.openrocket.swing.gui.figure3d.geometry.Vertex;
import org.joml.Vector2f;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

/**
 * Geometry for the flight replay surroundings. All meshes are in engine units with +Y up.
 */
final class ReplayMeshes {
	private static final int OUTSIDE = RenderingConstants.SURFACE_ID_OUTSIDE;

	private ReplayMeshes() {
	}

	/** Open cylinder of radius 1 from y = 0 to y = 1, to be scaled into lines and rods. */
	static Mesh unitCylinder(int sides) {
		List<Vertex> vertices = new ArrayList<>();
		IntList indices = new IntList();
		for (int i = 0; i <= sides; i++) {
			double a = 2 * Math.PI * i / sides;
			float x = (float) Math.cos(a), z = (float) Math.sin(a);
			Vector3f normal = new Vector3f(x, 0, z);
			vertices.add(vertex(x, 0, z, normal));
			vertices.add(vertex(x, 1, z, normal));
		}
		for (int i = 0; i < sides; i++) {
			int b0 = 2 * i, t0 = b0 + 1, b1 = b0 + 2, t1 = b0 + 3;
			indices.addTriangle(b0, t0, t1);
			indices.addTriangle(b0, t1, b1);
		}
		return new Mesh(vertices, indices);
	}

	/**
	 * Shape of an inflated round canopy. The attachment point is the origin and the canopy
	 * opens upwards along +Y.
	 *
	 * @param diameter   nominal (flat) canopy diameter, in metres
	 * @param lineLength shroud line length, in metres
	 * @param lineCount  number of shroud lines, also the number of gores
	 */
	record CanopyShape(float rimRadius, float rimHeight, float domeHeight, int gores) {
		static CanopyShape of(double diameter, double lineLength, int lineCount) {
			float s = RenderingConstants.WORLD_SCALE;
			// An inflated round canopy spans roughly 70 % of its flat diameter.
			float rim = (float) (0.35 * diameter) * s;
			float lines = (float) Math.max(lineLength, 0.9 * diameter) * s;
			float rimHeight = (float) Math.sqrt(Math.max(lines * lines - rim * rim, 0.25f * lines * lines));
			int gores = Math.max(6, Math.min(lineCount > 0 ? lineCount : 8, 24));
			if (gores % 2 == 1) {
				gores++;
			}
			return new CanopyShape(rim, rimHeight, 0.55f * rim, gores);
		}
	}

	/**
	 * Builds alternate gores of a canopy, both sides, so it reads correctly from below.
	 *
	 * @param parity 0 or 1, selecting the gores of one colour
	 */
	static Mesh canopy(CanopyShape shape, int parity) {
		final int rings = 8;
		final int segmentsPerGore = 3;
		final float ventFraction = 0.08f;
		List<Vertex> vertices = new ArrayList<>();
		IntList indices = new IntList();
		float rx = shape.rimRadius(), hy = shape.domeHeight();
		for (int g = parity; g < shape.gores(); g += 2) {
			int start = vertices.size();
			for (int r = 0; r <= rings; r++) {
				float s = ventFraction + (1 - ventFraction) * r / rings;
				double polar = s * Math.PI / 2;
				float radius = (float) (rx * Math.sin(polar));
				float y = shape.rimHeight() + (float) (hy * Math.cos(polar));
				for (int k = 0; k <= segmentsPerGore; k++) {
					double a = 2 * Math.PI * (g + (double) k / segmentsPerGore) / shape.gores();
					float x = (float) (radius * Math.cos(a)), z = (float) (radius * Math.sin(a));
					// Outward normal of the spheroid with semi-axes (rx, hy, rx)
					Vector3f normal = new Vector3f(x / (rx * rx), (y - shape.rimHeight()) / (hy * hy), z / (rx * rx))
							.normalize();
					vertices.add(vertex(x, y, z, normal));
				}
			}
			int columns = segmentsPerGore + 1;
			for (int r = 0; r < rings; r++) {
				for (int k = 0; k < segmentsPerGore; k++) {
					int a = start + r * columns + k, b = a + 1, c = a + columns, d = c + 1;
					indices.addTriangle(a, b, d);
					indices.addTriangle(a, d, c);
				}
			}
		}
		addBackFaces(vertices, indices);
		return new Mesh(vertices, indices);
	}

	/** Shroud lines from the attachment point at the origin to the canopy rim. */
	static Mesh shroudLines(CanopyShape shape, float lineRadius) {
		List<Vertex> vertices = new ArrayList<>();
		IntList indices = new IntList();
		final int sides = 4;
		for (int g = 0; g < shape.gores(); g++) {
			double a = 2 * Math.PI * g / shape.gores();
			Vector3f end = new Vector3f((float) (shape.rimRadius() * Math.cos(a)), shape.rimHeight(),
					(float) (shape.rimRadius() * Math.sin(a)));
			addTube(vertices, indices, new Vector3f(), end, lineRadius, sides);
		}
		return new Mesh(vertices, indices);
	}

	private static void addTube(List<Vertex> vertices, IntList indices, Vector3f from, Vector3f to,
			float radius, int sides) {
		Vector3f axis = new Vector3f(to).sub(from).normalize();
		Vector3f side = Math.abs(axis.y) < 0.9f ? new Vector3f(0, 1, 0) : new Vector3f(1, 0, 0);
		Vector3f u = side.cross(axis, new Vector3f()).normalize();
		Vector3f v = axis.cross(u, new Vector3f()).normalize();
		int base = vertices.size();
		for (int i = 0; i <= sides; i++) {
			double a = 2 * Math.PI * i / sides;
			Vector3f normal = new Vector3f(u).mul((float) Math.cos(a)).add(new Vector3f(v).mul((float) Math.sin(a)));
			Vector3f offset = new Vector3f(normal).mul(radius);
			vertices.add(vertex(from.x + offset.x, from.y + offset.y, from.z + offset.z, normal));
			vertices.add(vertex(to.x + offset.x, to.y + offset.y, to.z + offset.z, normal));
		}
		for (int i = 0; i < sides; i++) {
			int b0 = base + 2 * i, t0 = b0 + 1, b1 = b0 + 2, t1 = b0 + 3;
			indices.addTriangle(b0, b1, t1);
			indices.addTriangle(b0, t1, t0);
		}
	}

	private static void addBackFaces(List<Vertex> vertices, IntList indices) {
		int count = vertices.size();
		int triangles = indices.size();
		for (int i = 0; i < count; i++) {
			Vertex front = vertices.get(i);
			vertices.add(new Vertex(new Vector3f(front.position), new Vector3f(front.normal).negate(),
					new Vector2f(front.texCoords), front.surfaceID));
		}
		for (int i = 0; i < triangles; i += 3) {
			indices.addTriangle(indices.get(i) + count, indices.get(i + 2) + count, indices.get(i + 1) + count);
		}
	}

	private static Vertex vertex(float x, float y, float z, Vector3f normal) {
		return new Vertex(new Vector3f(x, y, z), new Vector3f(normal), new Vector2f(0, 0), OUTSIDE);
	}
}
