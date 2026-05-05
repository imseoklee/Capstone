import omni
from pxr import Usd, UsdGeom


OUTPUT_PATH = "/home/nsl/Capston_workspace/occupancy_map/maps/Map01_prim_structure.txt"
KEYWORD = "PalletBin"
EXTRA_TOP_LEVEL_PRIMS = ("/Root/PackingStation")
TOP_LEVEL_PREFIX = "/Root/"
TOP_LEVEL_DEPTH = 2


stage = omni.usd.get_context().get_stage()
cache = UsdGeom.XformCache(Usd.TimeCode.Default())

with open(OUTPUT_PATH, "w", encoding="utf-8") as output_file:
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if KEYWORD not in path and path not in EXTRA_TOP_LEVEL_PRIMS:
            continue
        if not path.startswith(TOP_LEVEL_PREFIX):
            continue
        if path.count("/") != TOP_LEVEL_DEPTH:
            continue

        transform = cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        x = float(translation[0])
        y = float(translation[1])
        z = float(translation[2])
        output_file.write(f"{path} | position=({x:.6f}, {y:.6f}, {z:.6f})\n")

print(f"Saved {KEYWORD} prim paths and extra top-level prims to: {OUTPUT_PATH}")
