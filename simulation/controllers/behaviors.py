"""
Behavior Controllers for Unitree Go2
====================================
Implements high-level behaviors like walking, jumping, and backflips.

Each behavior is broken down into phases that execute over time.

HOW BEHAVIORS WORK:
-------------------
Each behavior has an update() method that gets called every simulation step (240 times per second).
The behavior calculates what the robot should do at this moment in time, then returns a status message.

Think of it like animation frames:
- Frame 1: Start crouching
- Frame 2: Crouch a bit more
- Frame 3: Crouch even more
- Frame 4: Begin jumping
... and so on
"""

# Import numpy for math operations
import numpy as np
# Import pybullet for physics simulation control
import pybullet as p
# Import our inverse kinematics solver
from .kinematics import QuadrupedIK


class BehaviorController:
    """Base class for all behaviors."""

    def __init__(self, robot_id):
        # Store the robot's ID (which robot in the simulation we're controlling)
        self.robot_id = robot_id

        # Create an inverse kinematics solver (converts positions to joint angles)
        self.ik = QuadrupedIK()

        # Track how long this behavior has been running (in seconds)
        self.time = 0

        # Flag to indicate when this behavior is finished
        self.is_complete = False

        # Store joint indices for easy access (build a name-to-number mapping)
        self.joint_indices = self._get_joint_indices()

    def _get_joint_indices(self):
        """Map joint names to PyBullet joint indices."""
        # Create an empty dictionary to map names like "FR_hip_joint" to numbers like 0, 1, 2...
        indices = {}

        # Ask PyBullet how many joints this robot has
        num_joints = p.getNumJoints(self.robot_id)

        # Loop through each joint
        for i in range(num_joints):
            # Get information about joint number i
            info = p.getJointInfo(self.robot_id, i)
            # Extract the name and decode it from bytes to string
            joint_name = info[1].decode('utf-8')
            # Store the mapping: name -> index number
            indices[joint_name] = i

        # Return the complete mapping dictionary
        return indices

    def set_joint_angles(self, joint_angles_dict, kp=1.0, kd=0.3):
        """
        Set target joint angles with PD control.

        Args:
            joint_angles_dict: Dict like {'FR': [hip, thigh, calf], ...}
            kp: Proportional gain (stiffness) - higher = stiffer, reaches target faster
            kd: Derivative gain (damping) - higher = more damping, less oscillation
        """
        # Loop through each leg and its desired angles
        # leg_name is like 'FR', 'FL', etc.
        # angles is a tuple/list of three values: (hip, thigh, calf)
        for leg_name, angles in joint_angles_dict.items():
            # Unpack the three angles for this leg
            hip_angle, thigh_angle, calf_angle = angles

            # Create a list of (joint_name, target_angle) pairs for this leg
            joints = [
                (f"{leg_name}_hip_joint", hip_angle),     # e.g., "FR_hip_joint" -> hip_angle
                (f"{leg_name}_thigh_joint", thigh_angle), # e.g., "FR_thigh_joint" -> thigh_angle
                (f"{leg_name}_calf_joint", calf_angle)    # e.g., "FR_calf_joint" -> calf_angle
            ]

            # Set each joint to its target angle
            for joint_name, target_angle in joints:
                # Check if this joint exists in our mapping
                if joint_name in self.joint_indices:
                    # Tell PyBullet to move this joint to the target angle
                    # This uses PD control: a control system that smoothly moves to target
                    p.setJointMotorControl2(
                        self.robot_id,                     # Which robot
                        self.joint_indices[joint_name],    # Which joint (by index number)
                        p.POSITION_CONTROL,                # Control mode: position control
                        targetPosition=target_angle,       # Where we want the joint to go
                        positionGain=kp,                   # kp: how aggressively to reach target
                        velocityGain=kd,                   # kd: how much to resist oscillation
                        force=25  # Maximum torque the motor can apply (25 Newton-meters)
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
        # Call the parent class's __init__ method (sets up robot_id, ik, time, etc.)
        super().__init__(robot_id)
        # Store how long this entire behavior should take (in seconds)
        self.duration = duration

    def update(self, dt):
        # Add the time step to our total time (dt = delta time, usually 1/240 seconds)
        self.time += dt

        # Calculate progress as a fraction from 0.0 to 1.0
        # progress = 0.0 means just started
        # progress = 0.5 means halfway done
        # progress = 1.0 means finished
        # min() ensures we never go above 1.0 even if time exceeds duration
        progress = min(self.time / self.duration, 1.0)

        # ===== PHASE 1: CROUCH (first 30% of time) =====
        if progress < 0.3:
            # Calculate progress within this phase (0.0 to 1.0)
            # When progress=0.0, phase_progress=0.0
            # When progress=0.3, phase_progress=1.0
            phase_progress = progress / 0.3

            # Gradually lower the body height
            # Start at 0.35m, end at 0.25m (10cm lower)
            # phase_progress * 0.1 goes from 0 to 0.1 as phase progresses
            height = 0.35 - 0.1 * phase_progress

            # Get joint angles for standing at this height (all 4 legs on ground)
            joint_angles = self.ik.standing_pose(height)

            # Create status message showing what phase we're in
            status = f"Phase 1: Crouching ({phase_progress*100:.0f}%)"

        # ===== PHASE 2: SHIFT WEIGHT (30% to 60% of time) =====
        elif progress < 0.6:
            # Calculate progress within this phase
            # When progress=0.3, phase_progress=0.0
            # When progress=0.6, phase_progress=1.0
            phase_progress = (progress - 0.3) / 0.3

            # Stay low (0.25m) to keep center of gravity stable
            # The robot is preparing to lift its front legs by shifting weight back
            joint_angles = self.ik.standing_pose(0.25)
            status = f"Phase 2: Shifting weight ({phase_progress*100:.0f}%)"

        # ===== PHASE 3: LIFT FRONT LEGS (60% to 85% of time) =====
        elif progress < 0.85:
            # Calculate progress within this phase
            phase_progress = (progress - 0.6) / 0.25

            # Interpolate (blend) between two poses
            # Start: crouched quadruped pose (all 4 legs down)
            start_angles = self.ik.standing_pose(0.25)
            # End: bipedal pose (front legs up, rear legs supporting)
            end_angles = self.ik.bipedal_pose(0.50)

            # Create a smooth transition between start and end poses
            joint_angles = {}
            # For each leg (Front Right, Front Left, Rear Right, Rear Left)
            for leg in ['FR', 'FL', 'RR', 'RL']:
                # For each of the 3 joints (hip, thigh, calf)
                # Linear interpolation formula: start * (1 - t) + end * t
                # When phase_progress=0, we get start_angles (100% start, 0% end)
                # When phase_progress=1, we get end_angles (0% start, 100% end)
                joint_angles[leg] = tuple(
                    start_angles[leg][i] * (1 - phase_progress) +
                    end_angles[leg][i] * phase_progress
                    for i in range(3)  # 0=hip, 1=thigh, 2=calf
                )
            status = f"Phase 3: Lifting front legs ({phase_progress*100:.0f}%)"

        # ===== PHASE 4: STABILIZE BIPEDAL (85% to 100% of time) =====
        else:
            # Final phase - hold the bipedal stance and stabilize
            phase_progress = (progress - 0.85) / 0.15
            # Get the full bipedal pose (front legs up, standing on rear legs)
            joint_angles = self.ik.bipedal_pose(0.50)
            status = f"Phase 4: Bipedal stance ({phase_progress*100:.0f}%)"

        # Apply the calculated joint angles to the robot
        # Use moderate gains (kp=1.0, kd=0.3) for smooth, controlled motion
        self.set_joint_angles(joint_angles, kp=1.0, kd=0.3)

        # Check if we've completed the entire behavior
        if progress >= 1.0:
            # Mark this behavior as complete
            self.is_complete = True
            # Add "COMPLETE" to the status message
            status += " - COMPLETE"

        # Return the status message (gets printed to show progress)
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

    HOW BACKFLIPS WORK:
    -------------------
    1. Jump UP (extend legs + apply upward force)
    2. Rotate BACKWARD (tuck body + apply backward rotation torque)
    3. Complete the flip (full 360° rotation)
    4. Extend legs before landing
    5. Absorb impact with high damping

    This behavior applies external forces to the robot body in addition
    to controlling joint angles. That's what makes it so dynamic!
    """

    def __init__(self, robot_id, duration=2.0):
        # Call parent class constructor
        super().__init__(robot_id)
        # Store how long the entire backflip should take
        self.duration = duration

    def update(self, dt):
        # Add time step to total time
        self.time += dt
        # Calculate overall progress (0.0 to 1.0)
        progress = min(self.time / self.duration, 1.0)

        # ===== PHASE 1: EXPLOSIVE JUMP (first 15% of time) =====
        if progress < 0.15:
            # Calculate progress within this phase
            phase_progress = progress / 0.15

            # Rapidly extend legs to jump
            # Start at 0.20m (crouched), extend to 0.45m (fully extended)
            # This creates the explosive upward motion
            height = 0.20 + 0.25 * phase_progress
            joint_angles = self.ik.standing_pose(height)

            # Apply VERY strong position control (kp=3.0) for explosive extension
            # High kp makes the motors push hard to reach the extended position quickly
            self.set_joint_angles(joint_angles, kp=3.0, kd=0.3)

            # Apply additional upward and backward force
            # We only do this in the second half of the jump phase (more explosive!)
            if phase_progress > 0.5:
                # Get the robot's current position in the world
                base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)

                # Apply a strong upward force at the robot's center
                # force = [x, y, z] in Newtons
                # 500N upward is a BIG push (robot weighs ~15kg × 9.8 = 147N)
                force = [0, 0, 500]  # All force in Z direction (up)
                p.applyExternalForce(
                    self.robot_id,           # Which robot
                    -1,  # Link index: -1 means the base (main body)
                    force,                   # The force vector [x, y, z]
                    base_pos,                # Where to apply the force (at body center)
                    p.WORLD_FRAME            # Coordinates relative to world (not robot)
                )

                # Apply torque for backward rotation (this starts the flip!)
                # torque = [roll, pitch, yaw] in Newton-meters
                # Negative pitch rotates the robot backward
                torque = [0, -150, 0]  # -150 N⋅m around Y-axis = pitch backward
                p.applyExternalTorque(
                    self.robot_id,    # Which robot
                    -1,               # Apply to base link
                    torque,           # The torque vector
                    p.WORLD_FRAME     # World coordinates
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
