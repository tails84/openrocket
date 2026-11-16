# Flight replay launch sites

The flight replay draws the real surroundings of a launch site it knows: land cover near the pads
and out to the hills, and the height of the ground. Each site has a folder here with the script
that builds its data, and the data itself sits in
`swing/src/main/resources/datafiles/replay/<site>/`.

## Cedar Grove (QRS launch site LA2)

`cedar-grove/make_site_data.py` turns map tiles into three rasters on a north-up grid centred on
the launch area (-27.84968, 152.96084):

- `landcover_near.png`: one land cover class per 4 m cell out to 3.2 km, from OpenStreetMap tiles at zoom 15.
- `landcover_far.png`: one class per 32 m cell out to 16.4 km, from OpenStreetMap tiles at zoom 12.
- `heights.png`: 16-bit ground height per 64 m cell out to 16.4 km, in metres plus 1000, from the
  Terrain Tiles on AWS Open Data (Terrarium encoding, zoom 12).

Map data © OpenStreetMap contributors, available under the Open Database License
(https://www.openstreetmap.org/copyright). Elevation from the Terrain Tiles dataset on the Registry of
Open Data on AWS, which combines SRTM, GMTED2010, ETOPO1 and other public sources.

The crops south of Cedar Grove Road and the track into the launch paddock are placed by eye in
`SiteSurvey.java`, as the map does not show them.

To rebuild, fetch the tiles into one folder as `osm_<x>_<y>.png` (zoom 15, 7 x 7 round the site),
`osm12_<x>_<y>.png` and `terr_<x>_<y>.png` (zoom 12, 5 x 5 round the site), then run
`python3 make_site_data.py <tile folder> swing/src/main/resources/datafiles/replay/cedar-grove`.
It needs numpy, scipy, scikit-image and Pillow. Fetch OpenStreetMap tiles sparingly and with a
descriptive User-Agent, as its tile usage policy asks.
