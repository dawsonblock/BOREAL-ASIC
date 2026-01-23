
import mujoco
import os

model_path = "panda_table_cube.xml"
if not os.path.exists(model_path):
    print(f"Model not found at {model_path}")
    exit(1)

model = mujoco.MjModel.from_xml_path(model_path)
cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
print(f"Cube Geom ID: {cube_geom_id}")

# Count total geoms
print(f"Total Geoms: {model.ngeom}")
