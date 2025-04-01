import mujoco
from mujoco import viewer  # Uses native viewer
# import rtde_control
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time

model = mujoco.MjModel.from_xml_path("/app/scene.xml")
data = mujoco.MjData(model)
viewer.launch(model, data)  # Full OpenGL viewer

robot_rtde = RTDEControlInterface("192.168.20.35")  # Robot IP
pc_rtde = RTDEReceiveInterface("192.168.20.1")

# Mirroring loop
try:
    while True:
        # Get real robot joints (radians)
        actual_q = pc_rtde.getActualQ()
        
        # Update simulation
        data.qpos[:6] = actual_q
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time.sleep(0.008)  # ~125Hz (UR default)
        
except KeyboardInterrupt:
    robot_rtde.disconnect()