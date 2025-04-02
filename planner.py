import mujoco
from mujoco import viewer
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time
import numpy as np
import ompl.base as ob
import ompl.geometric as og
import sys

HOME_JOINT_POSITIONS = [42.31, -109.92, 71.49, -51.47, -89.67, -47.7] # degrees
HOME_JOINT_POSITIONS = np.deg2rad(HOME_JOINT_POSITIONS) # radians
TEST_POSITIONS = [60.35, -95.93, 95.34, -89.31, -89.57, -29.40] # degrees
TEST_POSITIONS = np.deg2rad(TEST_POSITIONS) # radians
class MotionPlanner:
    def __init__(self):
        print("Initializing Motion Planner...")
        self.sim_to_robot = False  # Set to True to control robot from GUI

        # Initialize MuJoCo
        self.model = mujoco.MjModel.from_xml_path("/app/scene.xml")
        self.data = mujoco.MjData(self.model)

        # Connect to UR5e
        self.rtde_control = RTDEControlInterface("192.168.20.35")
        self.rtde_receive = RTDEReceiveInterface("192.168.20.35")

        initial_q = self.rtde_receive.getActualQ() # initial joint positions
        self.data.qpos[:6] = initial_q

        self.previous_ctrl = np.zeros(self.model.nu)
        self.velocity = 0.01
        self.acceleration = 0.01
        self.time_step = 0.008  # 125Hz
        self.lookahead_time = 0.1  # seconds
        self.gain = 300  # Gain for servo control

        self.setup_motion_planner()

    def setup_motion_planner(self):
        """Set up OMPL motion planner"""
        # Create state space for 6 joints
        self.space = ob.RealVectorStateSpace(6)
        
        # Set joint limits - adjust these to match your robot's actual limits
        bounds = ob.RealVectorBounds(6)
        for i in range(6):
            bounds.setLow(i, -np.pi)  # Lower joint limit
            bounds.setHigh(i, np.pi)  # Upper joint limit
        self.space.setBounds(bounds)
        
        # Create simple setup
        self.ss = og.SimpleSetup(self.space)
        
        # Set state validity checker (collision detection)
        def is_state_valid(state):
            # Store current state
            original_qpos = self.data.qpos.copy()
            
            # Set state for collision checking
            for i in range(6):
                self.data.qpos[i] = state[i]
            
            # Forward kinematics to update positions
            mujoco.mj_forward(self.model, self.data)
            
            # Check for collisions - this depends on how your model is set up
            # This is a simplified check and might need to be adapted to your specific model
            has_collision = False
            for i in range(self.data.ncon):
                if self.data.contact[i].dist < 0:
                    has_collision = True
                    break
            
            # Restore original state
            self.data.qpos[:] = original_qpos
            mujoco.mj_forward(self.model, self.data)
            
            return not has_collision
        
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
        
        # Set planner
        planner = og.RRTConnect(self.ss.getSpaceInformation())
        self.ss.setPlanner(planner)

    def plan_motion(self, start, goal, planning_time=1.0):
        """Plan motion from start to goal joint positions"""
        # Convert to OMPL states
        start_state = ob.State(self.ss.getStateSpace())
        for i in range(6):
            start_state()[i] = start[i]
        
        goal_state = ob.State(self.ss.getStateSpace())
        for i in range(6):
            goal_state()[i] = goal[i]
        
        # Set start and goal states
        self.ss.setStartAndGoalStates(start_state, goal_state)
        
        # Solve
        solved = self.ss.solve(planning_time)
        
        if solved:
            # Get solution path
            path = self.ss.getSolutionPath()
            path.interpolate(50)  # Generate smoother path with more waypoints
            
            # Extract waypoints
            states = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                joint_values = [state[j] for j in range(6)]
                states.append(joint_values)
            
            return states
        else:
            return None

    def move_to_home(self):
        # Move to home position
        with viewer.launch_passive(self.model, self.data) as v:

            try:
                print("Moving to home position...")
                # Move the robot to the home position using servo control
                self.rtde_control.servoJ(HOME_JOINT_POSITIONS, self.velocity, self.acceleration, self.time_step, self.lookahead_time, self.gain)
                print("Moved to home position")
                
                while True:
                    actual_q = self.rtde_receive.getActualQ()
                    self.data.qpos[:6] = actual_q
                    mujoco.mj_step(self.model, self.data)
                    
                    v.sync()
                            
                    time.sleep(0.008)  # ~125Hz

            except KeyboardInterrupt:
                self.rtde_control.servoStop()
                self.rtde_control.disconnect()
                print("Interrupted while moving to home position")

    def move_robot_joint(self, simulate_only=False):
        print("adasasasdasdads")
        """Move to home position with motion planning"""
        with viewer.launch_passive(self.model, self.data) as v:
            try:
                print("Planning motion...")
                current_q = self.rtde_receive.getActualQ()
                
                # Plan motion from current position to home
                path = self.plan_motion(current_q, TEST_POSITIONS)
                # path = self.plan_motion(current_q, HOME_JOINT_POSITIONS)
                print(f"Planned path: {path}")
                print("faasdasdasdsadsd")
                
                if path:
                    print(f"Visualizing planned path with {len(path)} waypoints")
                    original_qpos = self.data.qpos.copy()
                    end_effector_positions = []


                    # Visualize the path in MuJoCo
                    # for waypoint in path:
                    #     # Set the joint positions to the waypoint
                    #     self.data.qpos[:6] = waypoint
                        
                    #     # Forward kinematics to update positions
                    #     mujoco.mj_forward(self.model, self.data)
                        
                    #     # Get the end effector position (modify the index as needed)
                    #     ee_idx = 6  # This is often the last body, adjust if needed
                    #     ee_pos = self.data.xpos[ee_idx].copy()
                        
                    #     end_effector_positions.append(ee_pos)
                    
                    # # Restore original state
                    # self.data.qpos[:] = original_qpos
                    # mujoco.mj_forward(self.model, self.data)
                    
 
                    if simulate_only:
                        print(f"Simulating planned path with {len(path)} waypoints")
                        for waypoint in path:
                            self.data.qpos[:6] = waypoint
                            mujoco.mj_step(self.model, self.data)
                            v.sync()
                            time.sleep(self.time_step)

                    if not simulate_only: 
                        print(f"Executing planned path with {len(path)} waypoints")
                        # Execute each waypoint
                        for waypoint in path:
                            self.rtde_control.servoJ(
                                waypoint, 
                                self.velocity, 
                                self.acceleration, 
                                self.time_step, 
                                self.lookahead_time, 
                                self.gain
                            )
                            
                            # Update simulation
                            actual_q = self.rtde_receive.getActualQ()
                            self.data.qpos[:6] = actual_q
                            mujoco.mj_step(self.model, self.data)
                            v.sync()
                            
                            time.sleep(self.time_step)
                        
                        print("Moved")
                else:
                    print("Motion planning failed, using direct movement")
                    # Fallback to direct movement
                    self.rtde_control.servoJ(
                        HOME_JOINT_POSITIONS,
                        self.velocity, 
                        self.acceleration, 
                        self.time_step, 
                        self.lookahead_time, 
                        self.gain
                    )
                
                # Keep updating the simulation
                while True:
                    actual_q = self.rtde_receive.getActualQ()
                    self.data.qpos[:6] = actual_q
                    mujoco.mj_step(self.model, self.data)
                    v.sync()
                    time.sleep(self.time_step)
                    sys.stdout.flush()

            except KeyboardInterrupt:
                self.rtde_control.servoStop()
                self.rtde_control.disconnect()
                print("Interrupted while moving to home position")

    def add_marker(self, position, size=0.02, color=(1, 0, 0, 1)):
        """
        Add a marker to the MuJoCo simulation.
        
        Args:
            position (list or tuple): The (x, y, z) position of the marker.
            size (float): The size of the marker (default is 0.02).
            color (tuple): The RGBA color of the marker (default is red).
        """
        # marker = mujoco.MjvGeom()
        # marker.type = mujoco.mjtGeom.mjGEOM_SPHERE  # Sphere marker
        # marker.size = [size, size, size]  # Marker size
        # marker.rgba = color  # Marker color
        # marker.pos = position  # Marker position
        # Create a scene if it doesn't exist
        # if not hasattr(self, "scene"):
        #     self.scene = mujoco.MjvScene(self.model, maxgeom=1000)

        # # Create a camera if it doesn't exist
        # if not hasattr(self, "camera"):
        #     self.camera = mujoco.MjvCamera()

        # # Update the scene with the marker
        # mujoco.mjv_updateScene(
        #     self.model,
        #     self.data,
        #     mujoco.MjvOption(),
        #     None,  # No perturbation
        #     self.camera,  # Camera object
        #     mujoco.mjtCatBit.mjCAT_ALL,
        #     self.scene,
        # )

        # # Add the marker to the scene's geoms
        # self.scene.add_marker(marker)
        if not hasattr(self, 'markers'):
            self.markers = []
        
        # Store the marker information
        self.markers.append({
            'pos': position,
            'size': [size, size, size],
            'rgba': color,
            'type': mujoco.mjtGeom.mjGEOM_SPHERE
        })

    # Launch viewer with modern API
    def main(self):
        with viewer.launch_passive(self.model, self.data) as v:
            try:
                while True:
                    # print("Initial joint positions:", initial_q)
                    if self.sim_to_robot:
                        if not np.array_equal(self.data.ctrl, previous_ctrl):
                            # Control values changed - prepare to send to robot
                            print("Control panel change detected!")
                            
                            # Get the control values for the 6 joints
                            control_values = self.data.ctrl[:6].copy()
                            
                            # Map control values to actual joint positions
                            target_joint_positions = self.initial_q + control_values
                            
                            # Send to real robot
                            self.rtde_control.servoJ(target_joint_positions, self.velocity, self.acceleration, self.time_step, self.lookahead_time, self.gain)
                            
                            # Update previous control values
                            previous_ctrl = self.data.ctrl.copy()
                    
                    # Always update the simulation with the real robot's current position
                    actual_q = self.rtde_receive.getActualQ()
                    self.data.qpos[:6] = actual_q
                    
                    # Step physics simulation
                    mujoco.mj_step(self.model, self.data)
                    
                    # Sync viewer
                    v.sync()
                    
                    time.sleep(0.008)  # ~125Hz
                    
            except KeyboardInterrupt:
                self.rtde_control.servoStop()
                self.rtde_control.disconnect()


    

if __name__ == "__main__":
    # main()
    motion_planner = MotionPlanner()
    # motion_planner.move_to_home()
    motion_planner.move_robot_joint()
    sys.stdout.flush()
    # rtde_control.disconnect()