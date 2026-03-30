"""
Inverse Kinematics for Unitree Go2
===================================
Calculates joint angles needed to place the foot at a desired position.

This is essential for controlling where the robot's feet go, which is needed for:
- Walking
- Standing on hind legs
- Jumping
- Backflips
"""

import numpy as np


class LegIK:
    """
    Inverse Kinematics solver for a single leg.

    Given a desired foot position (x, y, z), this calculates the joint angles
    (hip, thigh, calf) needed to reach that position.
    """

    def __init__(self, hip_length=0.08, thigh_length=0.213, calf_length=0.213):
        """
        Initialize with leg segment lengths.

        Args:
            hip_length: Length of hip link (meters)
            thigh_length: Length of thigh link (meters)
            calf_length: Length of calf link (meters)
        """
        self.hip_length = hip_length
        self.thigh_length = thigh_length
        self.calf_length = calf_length

    def solve(self, target_pos, hip_sign=1):
        """
        Solve inverse kinematics for the leg.

        Args:
            target_pos: Desired foot position [x, y, z] relative to body
            hip_sign: 1 for right legs, -1 for left legs

        Returns:
            tuple: (hip_angle, thigh_angle, calf_angle) in radians
                   Returns None if target is unreachable
        """
        x, y, z = target_pos

        # ===== HIP JOINT (abduction/adduction) =====
        # This moves the leg sideways
        # We need to account for the hip length offset
        y_adjusted = y - (hip_sign * self.hip_length)

        # Calculate hip angle using arctangent
        # This determines how far the leg swings outward
        hip_angle = np.arctan2(z, y_adjusted)

        # ===== THIGH and CALF JOINTS (2D IK in sagittal plane) =====
        # Now we solve for thigh and calf in the plane defined by the leg

        # Distance from hip joint to target in the y-z plane
        L = np.sqrt(y_adjusted**2 + z**2)

        # Distance from thigh joint to target in the x-z plane
        D = np.sqrt(x**2 + L**2)

        # Check if target is reachable
        max_reach = self.thigh_length + self.calf_length
        min_reach = abs(self.thigh_length - self.calf_length)

        if D > max_reach or D < min_reach:
            # Target is too far or too close - unreachable
            return None

        # Use law of cosines to find calf angle
        # cos(angle) = (a² + b² - c²) / (2ab)
        cos_calf = (self.thigh_length**2 + self.calf_length**2 - D**2) / \
                   (2 * self.thigh_length * self.calf_length)

        # Clamp to [-1, 1] to avoid numerical errors
        cos_calf = np.clip(cos_calf, -1.0, 1.0)

        # Calf angle (negative because it bends backwards)
        calf_angle = -(np.pi - np.arccos(cos_calf))

        # Calculate thigh angle
        alpha = np.arctan2(-x, L)  # Angle to target
        beta = np.arccos(
            (self.thigh_length**2 + D**2 - self.calf_length**2) /
            (2 * self.thigh_length * D)
        )
        thigh_angle = alpha + beta

        return hip_angle, thigh_angle, calf_angle

    def forward_kinematics(self, hip_angle, thigh_angle, calf_angle, hip_sign=1):
        """
        Calculate foot position from joint angles (forward kinematics).

        This is useful for verifying IK solutions and understanding current pose.

        Args:
            hip_angle, thigh_angle, calf_angle: Joint angles in radians
            hip_sign: 1 for right legs, -1 for left legs

        Returns:
            np.array: Foot position [x, y, z]
        """
        # Hip offset
        y_hip = hip_sign * self.hip_length

        # After hip rotation
        y1 = y_hip + self.hip_length * np.cos(hip_angle)
        z1 = self.hip_length * np.sin(hip_angle)

        # After thigh
        x2 = -self.thigh_length * np.sin(thigh_angle)
        z2 = z1 - self.thigh_length * np.cos(thigh_angle)

        # After calf
        calf_world_angle = thigh_angle + calf_angle
        x3 = x2 - self.calf_length * np.sin(calf_world_angle)
        z3 = z2 - self.calf_length * np.cos(calf_world_angle)

        return np.array([x3, y1, z3])


class QuadrupedIK:
    """
    Full body inverse kinematics for the quadruped robot.
    Manages all 4 legs together.
    """

    def __init__(self):
        """Initialize IK solvers for all legs."""
        self.leg_ik = LegIK()

        # Define leg positions relative to body center
        # FR = Front Right, FL = Front Left, RR = Rear Right, RL = Rear Left
        self.leg_offsets = {
            'FR': np.array([0.18, -0.13, 0]),
            'FL': np.array([0.18, 0.13, 0]),
            'RR': np.array([-0.18, -0.13, 0]),
            'RL': np.array([-0.18, 0.13, 0])
        }

        self.leg_signs = {
            'FR': 1,
            'FL': -1,
            'RR': 1,
            'RL': -1
        }

    def solve_leg(self, leg_name, foot_pos_body_frame):
        """
        Solve IK for a single leg.

        Args:
            leg_name: 'FR', 'FL', 'RR', or 'RL'
            foot_pos_body_frame: Desired foot position in body frame [x, y, z]

        Returns:
            tuple: (hip_angle, thigh_angle, calf_angle) or None if unreachable
        """
        # Get position relative to leg's hip joint
        offset = self.leg_offsets[leg_name]
        relative_pos = foot_pos_body_frame - offset

        # Solve IK
        sign = self.leg_signs[leg_name]
        return self.leg_ik.solve(relative_pos, sign)

    def solve_body_pose(self, body_height, body_pitch=0, body_roll=0):
        """
        Calculate joint angles for a specific body pose.

        Args:
            body_height: Height of body above ground (meters)
            body_pitch: Body pitch angle (radians, positive = nose up)
            body_roll: Body roll angle (radians, positive = right side down)

        Returns:
            dict: Joint angles for all legs, e.g. {'FR': [hip, thigh, calf], ...}
        """
        joint_angles = {}

        for leg_name, offset in self.leg_offsets.items():
            # Calculate foot position for this body pose
            # Assume feet stay flat on ground at corners of a rectangle

            # Nominal stance (feet under body)
            foot_x = offset[0]
            foot_y = offset[1]
            foot_z = -body_height

            # Apply body rotations
            # This is simplified - in reality would use rotation matrices
            foot_z += offset[0] * np.sin(body_pitch)  # Pitch effect
            foot_z += offset[1] * np.sin(body_roll)   # Roll effect

            foot_pos = np.array([foot_x, foot_y, foot_z])

            # Solve IK
            angles = self.solve_leg(leg_name, foot_pos)
            if angles is None:
                print(f"⚠️  Warning: Cannot reach position for {leg_name}")
                # Use default safe pose
                angles = (0, 0.9, -1.8)

            joint_angles[leg_name] = angles

        return joint_angles

    def standing_pose(self, height=0.35):
        """
        Get joint angles for normal standing pose.

        Args:
            height: Desired body height (meters)

        Returns:
            dict: Joint angles for standing
        """
        return self.solve_body_pose(height, body_pitch=0, body_roll=0)

    def bipedal_pose(self, height=0.50, lean_back=0.3):
        """
        Get joint angles for standing on hind legs.

        Args:
            height: Desired body height (meters)
            lean_back: How far to lean back (radians)

        Returns:
            dict: Joint angles for bipedal stance
        """
        joint_angles = {}

        # Front legs - lifted and tucked
        for leg_name in ['FR', 'FL']:
            # Pull feet up and back toward body
            offset = self.leg_offsets[leg_name]
            foot_pos = np.array([
                offset[0] - 0.15,  # Pull back
                offset[1],
                -0.1  # Lift up significantly
            ])
            angles = self.solve_leg(leg_name, foot_pos)
            if angles is None:
                angles = (0, 2.0, -2.5)  # Tucked position
            joint_angles[leg_name] = angles

        # Rear legs - supporting full weight
        for leg_name in ['RR', 'RL']:
            offset = self.leg_offsets[leg_name]
            foot_pos = np.array([
                offset[0] + 0.05,  # Slightly forward for balance
                offset[1],
                -height  # Lower to ground
            ])
            angles = self.solve_leg(leg_name, foot_pos)
            if angles is None:
                angles = (0, 1.2, -2.0)  # Extended standing position
            joint_angles[leg_name] = angles

        return joint_angles
