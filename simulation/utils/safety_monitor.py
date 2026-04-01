"""
Safety Monitor for Unitree Go2 Simulation
=========================================
This module monitors the robot's state and prevents dangerous movements.
It checks joint limits, velocities, body orientation, and contact forces.

IMPORTANT: Always test behaviors in simulation before running on real hardware!
"""

# Import numpy - a library for mathematical operations and arrays
import numpy as np
# Import pybullet - the physics simulation library (abbreviated as 'p' for convenience)
import pybullet as p


class SafetyMonitor:
    """
    Monitors robot safety constraints during simulation.

    This helps ensure that simulated behaviors won't damage the real robot.
    """

    def __init__(self, robot_id, max_body_tilt=45.0):
        """
        Initialize safety monitor.

        Args:
            robot_id: PyBullet body ID of the robot
            max_body_tilt: Maximum allowed body tilt in degrees (default 45°)
        """
        # Store the robot's ID number (assigned by PyBullet when robot is loaded)
        self.robot_id = robot_id

        # Convert the maximum tilt from degrees to radians (computers use radians for angles)
        # np.radians() does the conversion: radians = degrees × π/180
        self.max_body_tilt_rad = np.radians(max_body_tilt)

        # Get all joint information from PyBullet
        # Ask PyBullet "how many joints does this robot have?"
        self.num_joints = p.getNumJoints(robot_id)

        # Create an empty dictionary to store information about each joint
        self.joint_info = {}

        # Loop through each joint (i goes from 0 to num_joints-1)
        for i in range(self.num_joints):
            # Ask PyBullet for detailed information about joint number i
            # This returns a tuple (list) with lots of data
            info = p.getJointInfo(robot_id, i)

            # Extract the joint's name from position [1] in the info tuple
            # It's stored as bytes, so we decode it to a regular string
            joint_name = info[1].decode('utf-8')

            # Store the important limits and properties for this joint
            # We create a dictionary for each joint with its key information
            self.joint_info[i] = {
                'name': joint_name,           # The joint's name (e.g., "FR_hip_joint")
                'lower_limit': info[8],       # Minimum angle this joint can reach (radians)
                'upper_limit': info[9],       # Maximum angle this joint can reach (radians)
                'max_velocity': info[11],     # Fastest this joint can rotate (rad/s)
                'max_force': info[10]         # Maximum torque/force the motor can apply (N⋅m)
            }

        # Safety flags - lists to store warnings and errors
        # These are empty lists [] that we'll add messages to when problems are detected
        self.warnings = []           # Non-critical issues (yellow flags)
        self.critical_errors = []    # Serious problems (red flags - stop immediately!)

    def check_joint_limits(self, safety_margin=0.1):
        """
        Check if any joints are approaching their limits.

        Args:
            safety_margin: Stop within this fraction of the limit (0.1 = 10% margin)

        Returns:
            bool: True if all joints are safe, False if any are near limits
        """
        # Start by assuming everything is safe
        safe = True

        # Loop through each joint and its information
        # joint_idx is the joint number (0, 1, 2, ...)
        # info is the dictionary we stored earlier with limits, name, etc.
        for joint_idx, info in self.joint_info.items():
            # Get current state of this joint from PyBullet
            # This tells us where the joint is right now and how fast it's moving
            joint_state = p.getJointState(self.robot_id, joint_idx)
            # Extract position (angle) - this is at index [0] in the returned tuple
            position = joint_state[0]
            # Extract velocity (speed) - this is at index [1]
            velocity = joint_state[1]

            # Get the minimum and maximum angles for this joint
            lower = info['lower_limit']
            upper = info['upper_limit']

            # Skip fixed joints (they don't move)
            # Fixed joints have the same lower and upper limit (no range of motion)
            if lower == upper:
                continue  # Skip to the next joint

            # Calculate safety margin in radians
            # First, find the total range of motion for this joint
            range_size = upper - lower
            # Then, calculate the safety margin (10% of the range by default)
            # This creates a "buffer zone" near the limits
            margin = range_size * safety_margin

            # Check if the joint is approaching the lower limit
            # If position is less than (lower limit + safety margin), that's too close!
            if position < (lower + margin):
                # Add a warning message to our warnings list
                self.warnings.append(
                    f"⚠️  Joint {info['name']} near lower limit: "
                    # Convert radians to degrees for the message (easier to read)
                    f"{np.degrees(position):.1f}° (limit: {np.degrees(lower):.1f}°)"
                )
                # Mark as not safe
                safe = False
            # Check if the joint is approaching the upper limit
            elif position > (upper - margin):
                self.warnings.append(
                    f"⚠️  Joint {info['name']} near upper limit: "
                    f"{np.degrees(position):.1f}° (limit: {np.degrees(upper):.1f}°)"
                )
                safe = False

            # Check velocity (how fast the joint is rotating)
            # abs() gets the absolute value (removes negative sign, we only care about speed)
            # If moving faster than 90% of max velocity, that's too fast!
            if abs(velocity) > info['max_velocity'] * 0.9:
                self.warnings.append(
                    f"⚠️  Joint {info['name']} velocity high: {velocity:.2f} rad/s"
                )
                safe = False

        # Return True if all joints are safe, False if any problems were found
        return safe

    def check_body_orientation(self):
        """
        Check if the robot body is tilted beyond safe limits.

        Returns:
            tuple: (is_safe, roll, pitch, yaw) in degrees
        """
        # Get body position and orientation from PyBullet
        # position is [x, y, z] coordinates of the body center
        # orientation is a quaternion [x, y, z, w] - a 4-number way to represent rotation
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)

        # Convert quaternion to Euler angles (easier to understand)
        # Euler angles are: roll (tilt left/right), pitch (tilt forward/back), yaw (rotation)
        euler = p.getEulerFromQuaternion(orientation)
        # Unpack the three angles from the tuple
        roll, pitch, yaw = euler

        # Convert to degrees for easier checking and messages
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)

        # Start by assuming the robot is safe
        is_safe = True

        # Check if roll (left/right tilt) is too extreme
        # abs() removes the sign - we don't care if it's left or right, just how much
        if abs(roll) > self.max_body_tilt_rad:
            # Robot is tilted too far to the side - it might be falling over!
            self.critical_errors.append(
                f"🛑 CRITICAL: Body roll too extreme: {roll_deg:.1f}°"
            )
            is_safe = False

        # Check if pitch (forward/backward tilt) is too extreme
        if abs(pitch) > self.max_body_tilt_rad:
            # Robot is tilted too far forward or backward - danger!
            self.critical_errors.append(
                f"🛑 CRITICAL: Body pitch too extreme: {pitch_deg:.1f}°"
            )
            is_safe = False

        # Return four values as a tuple:
        # 1. is_safe (True/False)
        # 2. roll in degrees
        # 3. pitch in degrees
        # 4. yaw in degrees
        return is_safe, roll_deg, pitch_deg, np.degrees(yaw)

    def check_body_height(self, min_height=0.05):
        """
        Check if robot body is too low (might indicate a fall).

        Args:
            min_height: Minimum safe height in meters

        Returns:
            tuple: (is_safe, current_height)
        """
        # Get the position and orientation (we only need position here)
        # The underscore _ means "I don't care about this value" (orientation)
        position, _ = p.getBasePositionAndOrientation(self.robot_id)

        # Extract the Z coordinate (height above ground)
        # position is [x, y, z], so position[2] is the z value
        height = position[2]

        # Check if the robot's body is too close to the ground
        # If height is less than min_height (5cm by default), something's wrong!
        if height < min_height:
            # The robot has probably fallen or collapsed
            self.critical_errors.append(
                f"🛑 CRITICAL: Robot too low (possible fall): {height:.3f}m"
            )
            # Return False (not safe) and the current height
            return False, height

        # If we get here, the height is okay
        # Return True (safe) and the current height
        return True, height

    def check_foot_contacts(self, foot_link_names):
        """
        Check which feet are in contact with the ground.

        Args:
            foot_link_names: List of foot link names to check

        Returns:
            dict: {foot_name: is_in_contact}
        """
        contacts = {}

        for foot_name in foot_link_names:
            # Find the link index for this foot
            link_idx = None
            for i in range(self.num_joints):
                info = p.getJointInfo(self.robot_id, i)
                if info[12].decode('utf-8') == foot_name:  # link name
                    link_idx = i
                    break

            if link_idx is not None:
                # Check for contact points
                contact_points = p.getContactPoints(bodyA=self.robot_id, linkIndexA=link_idx)
                contacts[foot_name] = len(contact_points) > 0
            else:
                contacts[foot_name] = False

        return contacts

    def get_base_velocity(self):
        """
        Get the velocity of the robot's base.

        Returns:
            tuple: (linear_velocity, angular_velocity) as numpy arrays
        """
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        return np.array(linear_vel), np.array(angular_vel)

    def check_all(self):
        """
        Run all safety checks.

        Returns:
            bool: True if all checks pass, False otherwise
        """
        # Clear previous warnings and errors (start fresh each time)
        # This prevents old messages from accumulating
        self.warnings = []
        self.critical_errors = []

        # Run all three main checks and store their results
        # Check 1: Are all joints within safe limits and moving at safe speeds?
        joints_safe = self.check_joint_limits()
        # Check 2: Is the body tilted too much? Also get roll, pitch, yaw angles
        orientation_safe, roll, pitch, yaw = self.check_body_orientation()
        # Check 3: Is the robot too low (might have fallen)?
        height_safe, height = self.check_body_height()

        # Overall safety - ALL checks must pass for robot to be considered safe
        # The 'and' operator means: True only if ALL three are True
        is_safe = joints_safe and orientation_safe and height_safe

        # Return the overall safety status
        return is_safe

    def print_status(self, verbose=False):
        """Print current safety status."""
        # If there are any critical errors, print them prominently
        if self.critical_errors:
            # Print a line of equals signs for visual separation
            print("\n" + "="*60)
            print("🛑 CRITICAL SAFETY ERRORS:")
            # Loop through each error message and print it
            for error in self.critical_errors:
                print(error)
            print("="*60 + "\n")

        # If verbose mode is on AND there are warnings, print them
        # verbose=True means "give me all the details"
        if verbose and self.warnings:
            print("\n⚠️  WARNINGS:")
            for warning in self.warnings:
                print(warning)
            print()  # Empty line for spacing

    def emergency_stop(self):
        """
        Apply emergency stop - set all joint velocities to zero.
        """
        # Print a prominent emergency stop message
        print("\n🛑 EMERGENCY STOP ACTIVATED 🛑\n")

        # Loop through every joint on the robot
        for joint_idx in range(self.num_joints):
            # Tell PyBullet to control this joint's velocity
            p.setJointMotorControl2(
                self.robot_id,           # Which robot
                joint_idx,               # Which joint
                p.VELOCITY_CONTROL,      # Control mode: set velocity
                targetVelocity=0,        # Target velocity = 0 (stop moving)
                force=0                  # Apply no force (let it coast to a stop)
            )
