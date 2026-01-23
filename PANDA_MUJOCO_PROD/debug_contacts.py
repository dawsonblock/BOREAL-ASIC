
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("panda_table_cube.xml")
data = mujoco.MjData(model)

mujoco.mj_step(model, data)

# Run simulation to let cube settle
for _ in range(1000):
   mujoco.mj_step(model, data)

cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
print(f"Cube geom ID: {cube_geom_id}")

print(f"Number of contacts: {data.ncon}")
for i in range(data.ncon):
    con = data.contact[i]
    print(f"Contact {i}: geom1={con.geom1}, geom2={con.geom2}")
    if con.geom1 == cube_geom_id or con.geom2 == cube_geom_id:
        print(" -> Contact with CUBE detected!")
