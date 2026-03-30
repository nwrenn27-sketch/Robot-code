"""
Behavior Controllers for Unitree Go2
====================================
Implements high-level behaviors like walking, jumping, and backflips.

Each behavior is broken down into phases that execute over time.
"""

import numpy as np
import pybullet as p
from .kinematics import QuadrupedIK


class BehaviorController:
    """Base class for all behaviors."""

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.ik = QuadrupedIK()
        self.time = 0  # Behavior time
        self.is_complete = False

        # Store joint indices for easy access
        self.joint_indices = self._get_joint_indices()

    def _get_joint_indices(self):
        """Map joint names to PyBullet joint indices."""
        indices = {}
        num_joints = p.getNumJoints(self.robot_id)

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            indices[joint_name] = i

        return indices

    def set_joint_angles(self, joint_angles_dict, kp=1.0, kd=0.3):
        """
        Set target joint angles with PD control.

        Args:
            joint_angles_dict: Dict like {'FR': [hip, thigh, calf], ...}
            kp: Proportional gain (stiffness)
            kd: Derivative gain (damping)
        """
        for leg_name, angles in joint_angles_dict.items():
            hip_angle, thigh_angle, calf_angle = angles

            # Set each joint
            joints = [
                (f"{leg_name}_hip_joint", hip_angle),
                (f"{leg_name}_thigh_joint", thigh_angle),
                (f"{leg_name}_calf_joint", calf_angle)
            ]

            for joint_name, target_angle in joints:
                if joint_name in self.joint_indices:
                    p.setJointMotorControl2(
                        self.robot_id,
                        self.joint_indices[joint_name],
                        p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        positionGain=kp,
                        velocityGain=kd,
                        force=25  # Max torque
                    )

    def update(self, dt):
        """
        Update behavior (called each simulation step).

        Args:
            dt: Time step in seconds

        Returns:
            str: Status message
        """
        self.time += dt
        return "Base behavior"

    def reset(self):
        """Reset behavior to initial state."""
        self.time = 0
        self.is_complete = False


class StandingBehavior(BehaviorController):
    """Simple standing behavior - holds a stable standing pose."""

    def __init__(self, robot_id, height=0.35):
        super().__init__(robot_id)
        self.height = height

    def update(self, dt):
        self.time += dt

        # Calculate standing pose
        joint_angles = self.ik.standing_pose(self.height)

        # Apply with strong position control for stability
        self.set_joint_angles(joint_angles, kp=1.5, kd=0.5)

        return f"Standing at {self.height:.2f}m"


class BipedalTransition(BehaviorController):
    """
    Transition from quadruped to bipedal stance.

    This is done in phases:
    1. Crouch down (prepare to shift weight)
    2. Shift weight to rear legs
    3. Lift front legs
    4. Stand up on hind legs
    """

    def __init__(self, robot_id, duration=3.0):
        super().__init__(robot_id)
        self.duration = duration

    def update(self, dt):
        self.time += dt
        progress = min(self.time / self.duration, 1.0)

        if progress < 0.3:
            # Phase 1: Crouch
            phase_progress = progress / 0.3
            height = 0.35 - 0.1 * phase_progress  # Lower from 0.35 to 0.25
            joint_angles = self.ik.standing_pose(height)
            status = f"Phase 1: Crouching ({phase_progress*100:.0f}%)"

        elif progress < 0.6:
            # Phase 2: Shift weight backward
            phase_progress = (progress - 0.3) / 0.3
            # Keep low, shift weight
            joint_angles = self.ik.standing_pose(0.25)
            status = f"Phase 2: Shifting weight ({phase_progress*100:.0f}%)"

        elif progress < 0.85:
            # Phase 3: Lift front legs
            phase_progress = (progress - 0.6) / 0.25
            # Interpolate toward bipedal pose
            start_angles = self.ik.standing_pose(0.25)
            end_angles = self.ik.bipedal_pose(0.50)

            joint_angles = {}
            for leg in ['FR', 'FL', 'RR', 'RL']:
                joint_angles[leg] = tuple(
                    start_angles[leg][i] * (1 - phase_progress) +
                    end_angles[leg][i] * phase_progress
                    for i in range(3)
                )
            status = f"Phase 3: Lifting front legs ({phase_progress*100:.0f}%)"

        else:
            # Phase 4: Full bipedal stance
            phase_progress = (progress - 0.85) / 0.15
            joint_angles = self.ik.bipedal_pose(0.50)
            status = f"Phase 4: Bipedal stance ({phase_progress*100:.0f}%)"

        # Apply joint angles
        self.set_joint_angles(joint_angles, kp=1.0, kd=0.3)

        if progress >= 1.0:
            self.is_complete = True
            status += " - COMPLETE"

        return status


class BipedalWalking(BehaviorController):
    """
    Walking on hind legs.

    This uses a simple gait where the robot:
    1. Shifts weight to one leg
    2. Steps forward with the other leg
    3. Shifts weight to the stepping leg
    4. Repeats with opposite leg
    """

    def __init__(self, robot_id, step_duration=1.5, num_steps=4):
        super().__init__(robot_id)
        self.step_duration = step_duration
        self.num_steps = num_steps
        self.current_step = 0

    def update(self, dt):
        self.time += dt

        # Determine which step we're on
        step_time = self.time % self.step_duration
        step_progress = step_time / self.step_duration
        self.current_step = int(self.time / self.step_duration)

        if self.current_step >= self.num_steps:
            self.is_complete = True
            return f"Bipedal walking complete - {self.num_steps} steps taken"

        # Alternate legs (even steps = right leg, odd steps = left leg)
        is_right_leg = (self.current_step % 2 == 0)

        # Get base bipedal pose
        joint_angles = self.ik.bipedal_pose(0.50)

        # Modify the stepping leg
        if step_progress < 0.5:
            # Lift and move forward
            phase = step_progress / 0.5
            lift_height = 0.08 * np.sin(phase * np.pi)  # Arc motion

            if is_right_leg:
                leg_name = 'RR'
                # Move right leg forward and up
                offset = self.ik.leg_offsets[leg_name]
                foot_pos = np.array([
                    offset[0] + 0.1 * phase,  # Forward
                    offset[1],
                    -0.50 + lift_height  # Lift
                ])
                angles = self.ik.solve_leg(leg_name, foot_pos)
                if angles:
                    joint_angles[leg_name] = angles
            else:
                leg_name = 'RL'
                offset = self.ik.leg_offsets[leg_name]
                foot_pos = np.array([
                    offset[0] + 0.1 * phase,
                    offset[1],
                    -0.50 + lift_height
                ])
                angles = self.ik.solve_leg(leg_name, foot_pos)
                if angles:
                    joint_angles[leg_name] = angles

        # Apply joint angles
        self.set_joint_angles(joint_angles, kp=1.2, kd=0.4)

        leg_str = "right" if is_right_leg else "left"
        return f"Bipedal walking - Step {self.current_step + 1}/{self.num_steps} ({leg_str}, {step_progress*100:.0f}%)"


class JumpPreparation(BehaviorController):
    """
    Prepare for a jump or backflip by crouching and building tension.
    """

    def __init__(self, robot_id, duration=0.8):
        super().__init__(robot_id)
        self.duration = duration

    def update(self, dt):
        self.time += dt
        progress = min(self.time / self.duration, 1.0)

        # Crouch down progressively
        # Start at standing height (0.35m) and crouch to 0.20m
        height = 0.35 - 0.15 * progress

        joint_angles = self.ik.standing_pose(height)

        # Apply with high stiffness for explosive release
        self.set_joint_angles(joint_angles, kp=2.0, kd=0.5)

        if progress >= 1.0:
            self.is_complete = True
            return f"Jump preparation COMPLETE - ready to launch!"

        return f"Crouching for jump ({progress*100:.0f}%)"


class BackflipBehavior(BehaviorController):
    """
    Perform a backflip!

    This is the most complex behavior. Phases:
    1. Crouch (done by JumpPreparation)
    2. Explosive extension (jump)
    3. Tuck legs and rotate backward
    4. Extend legs for landing
    5. Absorb landing impact

    IMPORTANT: This is very dynamic and can fail in simulation.
    Test thoroughly before attempting on real hardware!
    """

    def __init__(self, robot_id, duration=2.0):
        super().__init__(robot_id)
        self.duration = duration

    def update(self, dt):
        self.time += dt
        progress = min(self.time / self.duration, 1.0)

        if progress < 0.15:
            # Phase 1: Explosive jump
            phase_progress = progress / 0.15

            # Rapidly extend legs to jump
            height = 0.20 + 0.25 * phase_progress  # Extend from crouch
            joint_angles = self.ik.standing_pose(height)

            # Apply strong torque
            self.set_joint_angles(joint_angles, kp=3.0, kd=0.3)

            # Apply additional upward and backward force
            if phase_progress > 0.5:
                base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                force = [0, 0, 500]  # Strong upward force
                p.applyExternalForce(
                    self.robot_id,
                    -1,  # Base link
                    force,
                    base_pos,
                    p.WORLD_FRAME
                )

                # Torque for backward rotation
                torque = [0, -150, 0]  # Pitch backward
                p.applyExternalTorque(
                    self.robot_id,
                    -1,
                    torque,
                    p.WORLD_FRAME
                )

            status = f"Phase 1: JUMP! ({phase_progress*100:.0f}%)"

        elif progress < 0.50:
            # Phase 2: Tuck legs and rotate
            phase_progress = (progress - 0.15) / 0.35

            # All legs tucked toward body
            joint_angles = {
                'FR': (0, 2.5, -2.5),
                'FL': (0, 2.5, -2.5),
                'RR': (0, 2.5, -2.5),
                'RL': (0, 2.5, -2.5)
            }

            # Continue rotation torque
            torque = [0, -80, 0]
            p.applyExternalTorque(self.robot_id, -1, torque, p.WORLD_FRAME)

            self.set_joint_angles(joint_angles, kp=2.0, kd=0.2)
            status = f"Phase 2: Rotating! ({phase_progress*100:.0f}%)"

        elif progress < 0.80:
            # Phase 3: Extend for landing
            phase_progress = (progress - 0.50) / 0.30

            # Gradually extend legs to landing position
            tuck_angles = {
                'FR': (0, 2.5, -2.5),
                'FL': (0, 2.5, -2.5),
                'RR': (0, 2.5, -2.5),
                'RL': (0, 2.5, -2.5)
            }
            land_angles = self.ik.standing_pose(0.40)

            joint_angles = {}
            for leg in ['FR', 'FL', 'RR', 'RL']:
                joint_angles[leg] = tuple(
                    tuck_angles[leg][i] * (1 - phase_progress) +
                    land_angles[leg][i] * phase_progress
                    for i in range(3)
                )

            self.set_joint_angles(joint_angles, kp=1.5, kd=0.4)
            status = f"Phase 3: Preparing to land ({phase_progress*100:.0f}%)"

        else:
            # Phase 4: Landing and stabilization
            phase_progress = (progress - 0.80) / 0.20

            # Strong damping for landing
            joint_angles = self.ik.standing_pose(0.35)
            self.set_joint_angles(joint_angles, kp=2.0, kd=1.0)

            status = f"Phase 4: LANDING! ({phase_progress*100:.0f}%)"

        if progress >= 1.0:
            self.is_complete = True
            status += " - BACKFLIP COMPLETE!"

        return status
