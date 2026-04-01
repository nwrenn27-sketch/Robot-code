"""
Inverse Kinematics for Unitree Go2
===================================
Calculates joint angles needed to place the foot at a desired position.

This is essential for controlling where the robot's feet go, which is needed for:
- Walking
- Standing on hind legs
- Jumping
- Backflips

WHAT IS INVERSE KINEMATICS (IK)?
---------------------------------
Forward Kinematics: "If I set these joint angles, where will the foot end up?"
Inverse Kinematics: "Where do I set the joints to put the foot at this position?"

IK is the reverse problem - much harder to solve!
"""

# Import numpy - needed for math operations like sin, cos, arctan, sqrt, etc.
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
        # Store the lengths of each leg segment
        # These are the "bones" of the robot leg
        self.hip_length = hip_length        # 8 cm - distance from body to thigh joint
        self.thigh_length = thigh_length    # 21.3 cm - upper leg bone
        self.calf_length = calf_length      # 21.3 cm - lower leg bone

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
        # Unpack the target position into x, y, z coordinates
        # x = forward/backward, y = left/right, z = up/down
        x, y, z = target_pos

        # ===== HIP JOINT (abduction/adduction) =====
        # The hip moves the leg sideways (away from or toward the body)

        # We need to account for the hip length offset
        # The hip joint is not at the body center - it's offset to the side
        # Subtract (or add for left side) the hip length to get the actual position
        y_adjusted = y - (hip_sign * self.hip_length)

        # Calculate hip angle using arctangent
        # arctan2(z, y) gives us the angle in the y-z plane
        # This determines how far the leg swings outward from the body
        hip_angle = np.arctan2(z, y_adjusted)

        # ===== THIGH and CALF JOINTS (2D IK in sagittal plane) =====
        # Now we solve for thigh and calf in a 2D plane
        # Think of looking at the leg from the side - it's now a 2D problem!

        # Distance from hip joint to target in the y-z plane
        # This is how far "sideways" the foot needs to be from the hip
        # sqrt(a² + b²) is the Pythagorean theorem - finding the hypotenuse
        L = np.sqrt(y_adjusted**2 + z**2)

        # Distance from thigh joint to target
        # Now we're in a 2D plane with x (forward) and L (sideways)
        # D is the straight-line distance from thigh joint to foot
        D = np.sqrt(x**2 + L**2)

        # Check if target is reachable - can the leg actually reach there?
        # Maximum reach: if we fully extend both thigh and calf in a straight line
        max_reach = self.thigh_length + self.calf_length
        # Minimum reach: if we fold the leg completely (one bone minus the other)
        min_reach = abs(self.thigh_length - self.calf_length)

        # If the target is too far away OR too close, we can't reach it
        if D > max_reach or D < min_reach:
            # Target is unreachable - return None to indicate failure
            return None

        # ===== LAW OF COSINES =====
        # We have a triangle: thigh -> calf -> target
        # We know all three side lengths, so we can find the angles!
        # Law of cosines: cos(C) = (a² + b² - c²) / (2ab)

        # Find the calf angle (knee angle)
        # This is the angle between the thigh and calf bones
        cos_calf = (self.thigh_length**2 + self.calf_length**2 - D**2) / \
                   (2 * self.thigh_length * self.calf_length)

        # Clamp to [-1, 1] to avoid numerical errors
        # Sometimes floating point math gives us -1.0000001, which would crash arccos
        # np.clip ensures the value stays in the valid range for arccos
        cos_calf = np.clip(cos_calf, -1.0, 1.0)

        # Calculate the calf angle
        # arccos gives us the angle, then subtract from π because knee bends backward
        # The negative sign is because the knee bends in the negative direction
        calf_angle = -(np.pi - np.arccos(cos_calf))

        # Calculate thigh angle (hip-to-knee angle)
        # This has two parts: direction to target + angle adjustment

        # alpha: angle from thigh joint pointing toward the target
        # We use -x because x-axis points forward, but we measure from vertical
        alpha = np.arctan2(-x, L)

        # beta: angle adjustment based on the triangle geometry
        # This accounts for the calf pushing back on the thigh
        beta = np.arccos(
            (self.thigh_length**2 + D**2 - self.calf_length**2) /
            (2 * self.thigh_length * D)
        )

        # Final thigh angle is the sum of these two components
        thigh_angle = alpha + beta

        # Return all three angles as a tuple
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
        # Create one IK solver that we'll use for all four legs
        # (They all have the same dimensions)
        self.leg_ik = LegIK()

        # Define leg positions relative to body center
        # These are where each hip joint is located on the body
        # FR = Front Right, FL = Front Left, RR = Rear Right, RL = Rear Left
        self.leg_offsets = {
            'FR': np.array([0.18, -0.13, 0]),   # Front right: 18cm forward, 13cm to right
            'FL': np.array([0.18, 0.13, 0]),    # Front left: 18cm forward, 13cm to left
            'RR': np.array([-0.18, -0.13, 0]),  # Rear right: 18cm back, 13cm to right
            'RL': np.array([-0.18, 0.13, 0])    # Rear left: 18cm back, 13cm to left
        }

        # Signs for mirroring left and right legs
        # Right legs use +1, left legs use -1
        self.leg_signs = {
            'FR': 1,   # Right side
            'FL': -1,  # Left side
            'RR': 1,   # Right side
            'RL': -1   # Left side
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
        # Create an empty dictionary to store angles for all legs
        joint_angles = {}

        # Front legs - lifted and tucked
        # When standing on hind legs, the front legs don't touch the ground
        for leg_name in ['FR', 'FL']:
            # Pull feet up and back toward body (like a person standing upright)
            offset = self.leg_offsets[leg_name]  # Where this leg attaches to body
            foot_pos = np.array([
                offset[0] - 0.15,  # Pull back 15cm from normal position
                offset[1],         # Keep same left/right position
                -0.1  # Lift up significantly - only 10cm below body (very high!)
            ])
            # Try to solve IK for this foot position
            angles = self.solve_leg(leg_name, foot_pos)
            # If IK fails (unreachable position), use a safe tucked position
            if angles is None:
                angles = (0, 2.0, -2.5)  # Tucked position (hip, thigh, calf)
            joint_angles[leg_name] = angles

        # Rear legs - supporting full weight
        # These legs bear all the robot's weight when bipedal
        for leg_name in ['RR', 'RL']:
            offset = self.leg_offsets[leg_name]
            foot_pos = np.array([
                offset[0] + 0.05,  # Slightly forward for balance (helps prevent tipping back)
                offset[1],         # Keep same left/right position
                -height  # Negative height (feet are below body)
            ])
            # Solve IK for standing position
            angles = self.solve_leg(leg_name, foot_pos)
            # If IK fails, use a safe extended position
            if angles is None:
                angles = (0, 1.2, -2.0)  # Extended standing position
            joint_angles[leg_name] = angles

        # Return all the angles for all 4 legs
        return joint_angles
