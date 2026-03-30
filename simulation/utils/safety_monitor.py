"""
Safety Monitor for Unitree Go2 Simulation
=========================================
This module monitors the robot's state and prevents dangerous movements.
It checks joint limits, velocities, body orientation, and contact forces.

IMPORTANT: Always test behaviors in simulation before running on real hardware!
"""

import numpy as np
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
        self.robot_id = robot_id
        self.max_body_tilt_rad = np.radians(max_body_tilt)

        # Get all joint information
        self.num_joints = p.getNumJoints(robot_id)
        self.joint_info = {}

        for i in range(self.num_joints):
            info = p.getJointInfo(robot_id, i)
            joint_name = info[1].decode('utf-8')

            # Store joint limits and max velocity
            self.joint_info[i] = {
                'name': joint_name,
                'lower_limit': info[8],   # Minimum angle
                'upper_limit': info[9],   # Maximum angle
                'max_velocity': info[11], # Maximum velocity
                'max_force': info[10]     # Maximum torque/force
            }

        # Safety flags
        self.warnings = []
        self.critical_errors = []

    def check_joint_limits(self, safety_margin=0.1):
        """
        Check if any joints are approaching their limits.

        Args:
            safety_margin: Stop within this fraction of the limit (0.1 = 10% margin)

        Returns:
            bool: True if all joints are safe, False if any are near limits
        """
        safe = True

        for joint_idx, info in self.joint_info.items():
            # Get current joint state
            joint_state = p.getJointState(self.robot_id, joint_idx)
            position = joint_state[0]
            velocity = joint_state[1]

            lower = info['lower_limit']
            upper = info['upper_limit']

            # Skip fixed joints (they have limit = 0)
            if lower == upper:
                continue

            # Calculate safety margin in radians
            range_size = upper - lower
            margin = range_size * safety_margin

            # Check if approaching limits
            if position < (lower + margin):
                self.warnings.append(
                    f"⚠️  Joint {info['name']} near lower limit: "
                    f"{np.degrees(position):.1f}° (limit: {np.degrees(lower):.1f}°)"
                )
                safe = False
            elif position > (upper - margin):
                self.warnings.append(
                    f"⚠️  Joint {info['name']} near upper limit: "
                    f"{np.degrees(position):.1f}° (limit: {np.degrees(upper):.1f}°)"
                )
                safe = False

            # Check velocity
            if abs(velocity) > info['max_velocity'] * 0.9:
                self.warnings.append(
                    f"⚠️  Joint {info['name']} velocity high: {velocity:.2f} rad/s"
                )
                safe = False

        return safe

    def check_body_orientation(self):
        """
        Check if the robot body is tilted beyond safe limits.

        Returns:
            tuple: (is_safe, roll, pitch, yaw) in degrees
        """
        # Get body orientation (as quaternion)
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = p.getEulerFromQuaternion(orientation)
        roll, pitch, yaw = euler

        # Check if tilt is too extreme
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)

        is_safe = True

        if abs(roll) > self.max_body_tilt_rad:
            self.critical_errors.append(
                f"🛑 CRITICAL: Body roll too extreme: {roll_deg:.1f}°"
            )
            is_safe = False

        if abs(pitch) > self.max_body_tilt_rad:
            self.critical_errors.append(
                f"🛑 CRITICAL: Body pitch too extreme: {pitch_deg:.1f}°"
            )
            is_safe = False

        return is_safe, roll_deg, pitch_deg, np.degrees(yaw)

    def check_body_height(self, min_height=0.05):
        """
        Check if robot body is too low (might indicate a fall).

        Args:
            min_height: Minimum safe height in meters

        Returns:
            tuple: (is_safe, current_height)
        """
        position, _ = p.getBasePositionAndOrientation(self.robot_id)
        height = position[2]

        if height < min_height:
            self.critical_errors.append(
                f"🛑 CRITICAL: Robot too low (possible fall): {height:.3f}m"
            )
            return False, height

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
        # Clear previous warnings
        self.warnings = []
        self.critical_errors = []

        # Run checks
        joints_safe = self.check_joint_limits()
        orientation_safe, roll, pitch, yaw = self.check_body_orientation()
        height_safe, height = self.check_body_height()

        # Overall safety
        is_safe = joints_safe and orientation_safe and height_safe

        return is_safe

    def print_status(self, verbose=False):
        """Print current safety status."""
        if self.critical_errors:
            print("\n" + "="*60)
            print("🛑 CRITICAL SAFETY ERRORS:")
            for error in self.critical_errors:
                print(error)
            print("="*60 + "\n")

        if verbose and self.warnings:
            print("\n⚠️  WARNINGS:")
            for warning in self.warnings:
                print(warning)
            print()

    def emergency_stop(self):
        """
        Apply emergency stop - set all joint velocities to zero.
        """
        print("\n🛑 EMERGENCY STOP ACTIVATED 🛑\n")

        for joint_idx in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
