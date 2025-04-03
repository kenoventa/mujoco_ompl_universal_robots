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
        self.sim_to_robot = True  # Set to True to control robot from GUI

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
        print("Prepare to move robot joint")
        """Move to home position with motion planning"""
        mujoco.glfw.glfw.init()  # Initialize GLFW
        # mujoco.mjv.makeScene(self.model, maxgeom=1000)  # Ensure scene capacity

        with viewer.launch_passive(self.model, self.data) as v:
            print(f"Scene geometry capacity: {v.user_scn.maxgeom}")  # Should be >100
            time.sleep(2)
            try:
                print("Planning motion...")
                current_q = self.rtde_receive.getActualQ()
                
                # Plan motion from current position to home
                # path = self.plan_motion(current_q, TEST_POSITIONS)
                path = self.plan_motion(current_q, HOME_JOINT_POSITIONS)
                print(f"Planned path: {path[1]}")
 
                if path:
                    print(f"Visualizing planned path with {len(path)} waypoints")
                    # Store path as instance variable for the callback
                    # Pre-compute Cartesian waypoints once
                    cartesian_waypoints = []
                    for wp in path:
                        self.data.qpos[:6] = wp
                        mujoco.mj_forward(self.model, self.data)
                        cartesian_waypoints.append(self.data.site_xpos[0].copy())

                    print('cartesian, ', cartesian_waypoints[1])    
                    print(f"Computed {len(cartesian_waypoints)} Cartesian waypoints")

                    # Store for the callback
                    self.visualization_waypoints = cartesian_waypoints

                    # Register the callback
                    v.user_scene_callback = lambda scn: self.modify_scene(self)
                    
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

    def modify_scene(self):
        def scene_callback(scn):
            print("executing callback")
            """Scene modification callback"""
            geom = mujoco.MjvGeom()
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.1, 0, 0]),  # radius = 1 cm
                np.array([0, 0, 0.1]),   # visible above ground
                np.eye(3).flatten(),
                np.array([1, 0, 0, 1])   # red
            )
            if scn.ngeom < scn.geoms.shape[0]:
                scn.geoms[scn.ngeom] = geom
                scn.ngeom += 1
        return scene_callback
        # if hasattr(self, 'visualization_waypoints'):
        #     # Clear previous visualizations
        #     scn.ngeom = 0
        #     print('aasdasdasdas  ', self.visualization_waypoints[1])
        #     # Visualize waypoints and connections
        #     self.visualize_waypoints(scn, self.model, self.data, 
        #                         self.visualization_waypoints)

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

    def visualize_waypoints(self, scn, model, data, waypoints):
        """Visualize waypoints as spheres and connecting lines."""
        # Visualize waypoints (spheres)
        for i, waypoint in enumerate(waypoints):
            # Create sphere geometry
            geom = mujoco.MjvGeom()
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.01, 0, 0]),  # Size (radius for sphere)
                waypoint,                # Position (already Cartesian)
                np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]),             # Rotation matrix (identity)
                np.array([0,1,0,1]) if i == 0 else  # Start = green
                np.array([1,0,0,1]) if i == len(waypoints)-1 else  # Goal = red
                np.array([1,1,0,1])      # Intermediate = yellow
            )
            
            # Add to scene if there's space
            if scn.ngeom < scn.geoms.shape[0]:
                scn.geoms[scn.ngeom] = geom
                scn.ngeom += 1

        # Add connecting lines between waypoints

        for i in range(len(waypoints)-1):
        # for i, waypoint in enumerate(waypoints):    
            start_pos = np.array(waypoints[i], dtype=np.float64)
            end_pos = np.array(waypoints[i+1], dtype=np.float64)
            from_array = np.array([[start_pos[0]], [start_pos[1]], [start_pos[2]]], dtype=np.float64)
            to_array = np.array([[end_pos[0]], [end_pos[1]], [end_pos[2]]], dtype=np.float64)
            print('wp1, ', waypoints[1])
            
            # Create line geometry (capsule/cylinder)
            geom = mujoco.MjvGeom()
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                np.zeros(3),             # Size (will be set by connector)
                np.zeros(3),             # Position (will be set by connector)
                np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]),             # Rotation matrix
                np.array([0.7, 0.7, 0.7, 0.8])  # Gray with transparency
            )
            
            print ('start_pos, ', start_pos)
            print ('end_pos, ', end_pos)
            # Make it a connector between points
            mujoco.mjv_connector(
                geom,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                100.0,                   # Width of the line
                from_array,
                to_array
            )
            # Add to scene if there's space
            if scn.ngeom < scn.geoms.shape[0]:
                scn.geoms[scn.ngeom] = geom
                scn.ngeom += 1


if __name__ == "__main__":
    # main()
    motion_planner = MotionPlanner()
    # motion_planner.move_to_home()
    motion_planner.move_robot_joint()
    # motion_planner.main()
    sys.stdout.flush()
    # rtde_control.disconnect()