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
 * Builds scenery meshes that are drawn unlit and shaded by a colour ramp instead. Each vertex's
 * texture u is how brightly the fixed sun lights it, and v picks a colour variant, so a whole
 * material in many shades draws in one call and casts nothing into the rocket's shadow map.
 *
 * <p>Positions are given in metres and stored in engine units.</p>
 */
final class ToneMeshBuilder {
	private static final float S = RenderingConstants.WORLD_SCALE;
	private static final int OUTSIDE = RenderingConstants.SURFACE_ID_OUTSIDE;

	private final List<Vertex> vertices = new ArrayList<>();
	private final IntList indices = new IntList();
	private final Vector3f toSun;
	private final int variants;
	private float v;

	/**
	 * @param toSun    unit vector towards the sun
	 * @param variants colour variants in the material's ramp
	 */
	ToneMeshBuilder(Vector3f toSun, int variants) {
		this.toSun = new Vector3f(toSun).normalize();
		this.variants = variants;
		variant(0);
	}

	/** Uses colour variant {@code index} for what is added next. */
	ToneMeshBuilder variant(int index) {
		v = (Math.floorMod(index, variants) + 0.5f) / variants;
		return this;
	}

	boolean isEmpty() {
		return indices.isEmpty();
	}

	Mesh build() {
		return new Mesh(vertices, indices);
	}

	/** Brightness of a surface facing {@code normal}: sunlight, a little skylight from above, and shade. */
	private float shade(Vector3f normal) {
		float sun = Math.max(0, normal.dot(toSun));
		float light = 0.40f + 0.62f * sun + 0.10f * normal.y;
		return Math.max(0.02f, Math.min(0.98f, light / LaunchSiteImages.RAMP_TOP));
	}

	int vertex(Vector3f position, Vector3f normal) {
		Vector3f n = new Vector3f(normal).normalize();
		vertices.add(new Vertex(new Vector3f(position).mul(S), n, new Vector2f(shade(n), v), OUTSIDE));
		return vertices.size() - 1;
	}

	void triangle(int a, int b, int c) {
		indices.addTriangle(a, b, c);
	}

	/** A flat quad, corners counter-clockwise seen from the side it faces. */
	void quad(Vector3f a, Vector3f b, Vector3f c, Vector3f d) {
		Vector3f normal = new Vector3f(b).sub(a).cross(new Vector3f(c).sub(a)).normalize();
		int i = vertex(a, normal), j = vertex(b, normal), k = vertex(c, normal), l = vertex(d, normal);
		triangle(i, j, k);
		triangle(i, k, l);
	}

	/** A quad seen from both sides. */
	void twoSidedQuad(Vector3f a, Vector3f b, Vector3f c, Vector3f d) {
		quad(a, b, c, d);
		quad(a, d, c, b);
	}

	/** A box from its centre and three half-extent vectors, which should be square to each other. */
	void box(Vector3f centre, Vector3f x, Vector3f y, Vector3f z) {
		Vector3f[] corners = new Vector3f[8];
		for (int i = 0; i < 8; i++) {
			corners[i] = new Vector3f(centre)
					.add(new Vector3f(x).mul((i & 1) == 0 ? -1 : 1))
					.add(new Vector3f(y).mul((i & 2) == 0 ? -1 : 1))
					.add(new Vector3f(z).mul((i & 4) == 0 ? -1 : 1));
		}
		// Each face as a loop around its edge; turned to face away from the centre below.
		int[][] faces = { { 0, 2, 6, 4 }, { 1, 3, 7, 5 }, { 0, 1, 5, 4 }, { 2, 3, 7, 6 }, { 0, 1, 3, 2 }, { 4, 5, 7, 6 } };
		for (int[] f : faces) {
			Vector3f a = corners[f[0]], b = corners[f[1]], c = corners[f[2]], d = corners[f[3]];
			Vector3f outward = new Vector3f(a).add(b).add(c).add(d).mul(0.25f).sub(centre);
			Vector3f normal = new Vector3f(b).sub(a).cross(new Vector3f(c).sub(a));
			if (normal.dot(outward) >= 0) {
				quad(a, b, c, d);
			} else {
				quad(d, c, b, a);
			}
		}
	}

	/** A box with level faces: centre, half widths along a horizontal direction and across it, half height. */
	void levelBox(Vector3f centre, Vector3f along, float halfLength, float halfWidth, float halfHeight) {
		Vector3f a = new Vector3f(along.x, 0, along.z).normalize();
		Vector3f across = new Vector3f(-a.z, 0, a.x);
		box(centre, a.mul(halfLength), new Vector3f(0, halfHeight, 0), across.mul(halfWidth));
	}

	/** A tapered tube between two points, open at both ends unless {@code caps}. */
	void cylinder(Vector3f from, Vector3f to, float fromRadius, float toRadius, int sides, boolean caps) {
		surface(from, to, fromRadius, toRadius, sides, false);
		if (caps) {
			Vector3f axis = new Vector3f(to).sub(from).normalize();
			Vector3f[] basis = basis(axis);
			disc(to, axis, basis[0], basis[1], toRadius, sides);
			disc(from, new Vector3f(axis).negate(), basis[1], basis[0], fromRadius, sides);
		}
	}

	/** Two unit vectors square to {@code axis} and each other, right-handed with it. */
	private static Vector3f[] basis(Vector3f axis) {
		Vector3f helper = Math.abs(axis.y) < 0.9f ? new Vector3f(0, 1, 0) : new Vector3f(1, 0, 0);
		Vector3f u = new Vector3f(helper).cross(axis).normalize();
		return new Vector3f[] { u, new Vector3f(axis).cross(u).normalize() };
	}

	/** An open tapered tube with its inside showing too, as a sleeve of cloth. */
	void sleeve(Vector3f from, Vector3f to, float fromRadius, float toRadius, int sides) {
		surface(from, to, fromRadius, toRadius, sides, false);
		surface(from, to, fromRadius, toRadius, sides, true);
	}

	private void surface(Vector3f from, Vector3f to, float fromRadius, float toRadius, int sides, boolean inside) {
		Vector3f axis = new Vector3f(to).sub(from).normalize();
		Vector3f[] basis = basis(axis);
		Vector3f u = basis[0], w = basis[1];
		float slope = (fromRadius - toRadius) / Math.max(1e-6f, new Vector3f(to).sub(from).length());
		int base = -1;
		for (int i = 0; i <= sides; i++) {
			double angle = 2 * Math.PI * i / sides;
			Vector3f radial = new Vector3f(u).mul((float) Math.cos(angle)).add(new Vector3f(w).mul((float) Math.sin(angle)));
			Vector3f normal = new Vector3f(radial).add(new Vector3f(axis).mul(slope)).normalize();
			if (inside) {
				normal.negate();
			}
			int a = vertex(new Vector3f(from).add(new Vector3f(radial).mul(fromRadius)), normal);
			vertex(new Vector3f(to).add(new Vector3f(radial).mul(toRadius)), normal);
			if (i == 0) {
				base = a;
			}
		}
		for (int i = 0; i < sides; i++) {
			int b0 = base + 2 * i, t0 = b0 + 1, b1 = b0 + 2, t1 = b0 + 3;
			if (inside) {
				triangle(b0, t1, b1);
				triangle(b0, t0, t1);
			} else {
				triangle(b0, b1, t1);
				triangle(b0, t1, t0);
			}
		}
	}

	private void disc(Vector3f centre, Vector3f normal, Vector3f u, Vector3f w, float radius, int sides) {
		int middle = vertex(centre, normal);
		int first = -1;
		for (int i = 0; i <= sides; i++) {
			double angle = 2 * Math.PI * i / sides;
			Vector3f p = new Vector3f(centre).add(new Vector3f(u).mul((float) (radius * Math.cos(angle))))
					.add(new Vector3f(w).mul((float) (radius * Math.sin(angle))));
			int index = vertex(p, normal);
			if (i == 0) {
				first = index;
			}
		}
		for (int i = 0; i < sides; i++) {
			triangle(middle, first + i, first + i + 1);
		}
	}

	/** An ellipsoid with level axes and smooth shading. */
	void blob(Vector3f centre, float rx, float ry, float rz, int slices, int stacks) {
		int base = vertices.size();
		for (int j = 0; j <= stacks; j++) {
			double polar = Math.PI * j / stacks;
			for (int i = 0; i <= slices; i++) {
				double azimuth = 2 * Math.PI * i / slices;
				float x = (float) (Math.sin(polar) * Math.cos(azimuth));
				float y = (float) Math.cos(polar);
				float z = (float) (Math.sin(polar) * Math.sin(azimuth));
				Vector3f normal = new Vector3f(x / rx, y / ry, z / rz);
				vertex(new Vector3f(centre).add(x * rx, y * ry, z * rz), normal);
			}
		}
		int columns = slices + 1;
		for (int j = 0; j < stacks; j++) {
			for (int i = 0; i < slices; i++) {
				int a = base + j * columns + i, b = a + 1, c = a + columns, d = c + 1;
				triangle(a, b, d);
				triangle(a, d, c);
			}
		}
	}
}
