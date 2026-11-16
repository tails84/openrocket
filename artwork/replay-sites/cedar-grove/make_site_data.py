"""
Builds the Cedar Grove (QRS launch site LA2) survey data the flight replay draws its
surroundings from, as rasters on a local grid centred near the launch area:

  landcover_near.png  class per 4 m cell, +-3200 m  (from OpenStreetMap tiles at zoom 15)
  landcover_far.png   class per 32 m cell, +-16384 m (from OpenStreetMap tiles at zoom 12)
  heights.png         16-bit height per 64 m cell, +-16384 m, metres + 1000 (from AWS terrain tiles, zoom 12)

Rows run north to south and columns west to east. Map data (c) OpenStreetMap contributors,
ODbL. Elevation from the Terrain Tiles on AWS Open Data (SRTM, GMTED and others).

Usage: python3 make_site_data.py <tile dir> <output dir>
The tile dir holds osm_<x>_<y>.png (zoom 15), osm12_<x>_<y>.png (zoom 12) and
terr_<x>_<y>.png (Terrarium encoding, zoom 12).
"""
import math
import sys

import numpy as np
from PIL import Image
from scipy import ndimage
from skimage.morphology import skeletonize

PAD_LAT, PAD_LON = -27.84968, 152.96084  # centre of the grids, near QRS LA2; SiteSurvey places the pads

# Classes, in the order the Java side reads them. Roads are kept as centre lines one cell wide,
# and narrow water (creeks and gullies) is told apart from the river and dams.
PADDOCK, PARK, PASTURE, WOOD, WATER, RESIDENTIAL, ROAD, WORKS, FARMLAND, GULLY = range(10)
PALETTE = [
    ((242, 239, 233), PADDOCK), ((243, 241, 236), PADDOCK), ((237, 234, 229), PADDOCK),
    ((200, 250, 204), PARK), ((200, 249, 204), PARK),
    ((205, 235, 176), PASTURE),
    ((173, 209, 158), WOOD), ((147, 182, 133), WOOD), ((157, 192, 143), WOOD), ((147, 197, 134), WOOD),
    ((178, 211, 164), WOOD), ((179, 199, 153), WOOD), ((200, 215, 171), WOOD),
    ((170, 211, 223), WATER),
    ((224, 223, 223), RESIDENTIAL), ((221, 221, 221), RESIDENTIAL), ((217, 208, 201), RESIDENTIAL),
    ((255, 255, 255), ROAD), ((254, 254, 254), ROAD), ((252, 214, 164), ROAD), ((249, 178, 156), ROAD),
    ((246, 250, 187), ROAD), ((247, 250, 191), ROAD),
    ((199, 199, 180), WORKS),
    ((238, 240, 213), FARMLAND),
]
UNKNOWN = 255


def mosaic(directory, prefix, xs, ys):
    image = np.zeros((256 * len(ys), 256 * len(xs), 3), np.uint8)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            tile = Image.open(f"{directory}/{prefix}_{x}_{y}.png").convert("RGB")
            image[256 * j:256 * j + 256, 256 * i:256 * i + 256] = np.asarray(tile)
    return image


def classify(rgb):
    colours = np.array([c for c, _ in PALETTE], float)
    classes = np.array([k for _, k in PALETTE], np.uint8)
    flat = rgb.reshape(-1, 3).astype(float)
    best = np.full(len(flat), 1e9)
    out = np.full(len(flat), UNKNOWN, np.uint8)
    for colour, cls in zip(colours, classes):
        d = ((flat - colour) ** 2).sum(1)
        closer = (d < best) & (d < 40)
        best = np.where(closer, d, best)
        out = np.where(closer, cls, out)
    out = out.reshape(rgb.shape[:2])
    # Labels, icons and lines take the class around them.
    for _ in range(6):
        unknown = out == UNKNOWN
        if not unknown.any():
            break
        filled = out.copy()
        for cls in range(9):
            near = ndimage.binary_dilation(out == cls, iterations=1) & unknown
            filled[near & (filled == UNKNOWN)] = cls
        out = filled
    out[out == UNKNOWN] = PADDOCK
    return out


def tile_xy(lat, lon, zoom):
    n = 2 ** zoom
    return (lon + 180) / 360 * n, (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n


def local_grid(half, cell):
    """Latitudes and longitudes of the centres of a north-up grid of cells round the pad."""
    count = int(round(2 * half / cell))
    offsets = (np.arange(count) + 0.5) * cell - half
    east, north = np.meshgrid(offsets, -offsets)
    lat = PAD_LAT + north / 110_574.0
    lon = PAD_LON + east / (111_320.0 * math.cos(math.radians(PAD_LAT)))
    return lat, lon


def refine(grid):
    """Thins roads to centre lines over what lies beside them, and marks narrow water as gullies."""
    out = grid.copy()
    # Close the gaps labels and casing leave in a road before finding its centre line.
    road = ndimage.binary_closing(grid == ROAD, structure=np.ones((3, 3)), iterations=2)
    road = ndimage.binary_opening(road, structure=np.ones((2, 2))) | (grid == ROAD)
    centre = skeletonize(road)
    # Drop the short spurs and specks that labels leave behind.
    labels, count = ndimage.label(centre, structure=np.ones((3, 3)))
    sizes = ndimage.sum(centre, labels, range(1, count + 1))
    centre &= np.isin(labels, np.flatnonzero(sizes >= 12) + 1)
    # Road cells off the centre line take the most common class around them.
    others = np.where(road, UNKNOWN, grid)
    for _ in range(8):
        unknown = others == UNKNOWN
        if not unknown.any():
            break
        for cls in range(10):
            near = ndimage.binary_dilation(others == cls, iterations=1) & (others == UNKNOWN)
            others[near] = cls
    others[others == UNKNOWN] = PADDOCK
    out = np.where(centre, ROAD, others)
    water = out == WATER
    wide = ndimage.binary_opening(water, structure=np.ones((3, 3)))
    wide = ndimage.binary_dilation(wide, iterations=1) & water
    # Creeks are dotted lines on the map: join the dots into one line.
    narrow = water & ~wide
    out[narrow] = PADDOCK
    joined = skeletonize(ndimage.binary_closing(narrow, structure=np.ones((3, 3)), iterations=3))
    out[joined & (out != WATER) & (out != ROAD)] = GULLY
    return out.astype(np.uint8)


def sample_classes(classes, x0, y0, zoom, half, cell):
    lat, lon = local_grid(half, cell)
    n = 2 ** zoom
    px = ((lon + 180) / 360 * n - x0) * 256
    py = ((1 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2 * n - y0) * 256
    px = np.clip(px.astype(int), 0, classes.shape[1] - 1)
    py = np.clip(py.astype(int), 0, classes.shape[0] - 1)
    return classes[py, px]


def main(tiles, out):
    lx, ly = tile_xy(PAD_LAT, PAD_LON, 15)
    xs15 = range(int(lx) - 3, int(lx) + 4)
    ys15 = range(int(ly) - 3, int(ly) + 4)
    near = classify(mosaic(tiles, "osm", xs15, ys15))
    Image.fromarray(refine(sample_classes(near, xs15[0], ys15[0], 15, 3200, 4))).save(f"{out}/landcover_near.png")

    gx, gy = tile_xy(PAD_LAT, PAD_LON, 12)
    xs12 = range(int(gx) - 2, int(gx) + 3)
    ys12 = range(int(gy) - 2, int(gy) + 3)
    far = classify(mosaic(tiles, "osm12", xs12, ys12))
    Image.fromarray(refine(sample_classes(far, xs12[0], ys12[0], 12, 16384, 32))).save(f"{out}/landcover_far.png")

    terrain = mosaic(tiles, "terr", xs12, ys12).astype(float)
    height = terrain[:, :, 0] * 256 + terrain[:, :, 1] + terrain[:, :, 2] / 256 - 32768
    lat, lon = local_grid(16384, 64)
    n = 2 ** 12
    px = ((lon + 180) / 360 * n - xs12[0]) * 256 - 0.5
    py = ((1 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2 * n - ys12[0]) * 256 - 0.5
    sampled = ndimage.map_coordinates(height, [py, px], order=1, mode="nearest")
    Image.fromarray(np.clip(sampled + 1000, 0, 65535).astype(np.uint16)).save(f"{out}/heights.png")
    print("near", np.bincount(np.asarray(Image.open(f"{out}/landcover_near.png")).ravel(), minlength=10))
    print("height at pad", float(ndimage.map_coordinates(height, [[(tile_xy(PAD_LAT, PAD_LON, 12)[1] - ys12[0]) * 256 - 0.5],
          [(gx - xs12[0]) * 256 - 0.5]], order=1)[0]), "range", sampled.min(), sampled.max())


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
