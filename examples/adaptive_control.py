import mujoco
import mujoco_viewer

# model = mujoco.MjModel.from_xml_string(MODEL_XML)
model = mujoco.MjModel.from_xml_path( '../models/double_pendulum.xml')
data  = mujoco.MjData( model )

# create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True)


# simulate and render
for _ in range(100000):
    mujoco.mj_step(model, data)
    viewer.render()

# close
viewer.close()