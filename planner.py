import mujoco
from mujoco import viewer
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time
import numpy as np

# Initialize MuJoCo
model = mujoco.MjModel.from_xml_path("/app/scene.xml")
data = mujoco.MjData(model)

# Connect to UR5e
rtde_control = RTDEControlInterface("192.168.20.35")
rtde_receive = RTDEReceiveInterface("192.168.20.35")

initial_q = rtde_receive.getActualQ() # initial joint positions
data.qpos[:6] = initial_q

sim_to_robot = False  # Set to True to control robot from GUI

previous_ctrl = np.zeros(model.nu)

velocity = 0.1
acceleration = 0.1
time_step = 0.008  # 125Hz
lookahead_time = 0.1  # seconds
gain = 300  # Gain for servo control

# Launch viewer with modern API
with viewer.launch_passive(model, data) as v:
    v.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

    try:
        while True:
            if sim_to_robot:
                if not np.array_equal(data.ctrl, previous_ctrl):
                    # Control values changed - prepare to send to robot
                    print("Control panel change detected!")
                    
                    # Get the control values for the 6 joints
                    control_values = data.ctrl[:6].copy()
                    
                    # Map control values to actual joint positions
                    # (You might need to adjust this mapping based on your model)
                    target_joint_positions = initial_q + control_values
                    
                    # Send to real robot
                    rtde_control.servoJ(target_joint_positions, velocity, acceleration, time_step, lookahead_time, gain)
                    
                    # Update previous control values
                    previous_ctrl = data.ctrl.copy()
            
            # Always update the simulation with the real robot's current position
            actual_q = rtde_receive.getActualQ()
            data.qpos[:6] = actual_q
            
            # Step physics simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            v.sync()
            
            time.sleep(0.008)  # ~125Hz
            
    except KeyboardInterrupt:
        rtde_control.servoStop()
        rtde_control.disconnect()