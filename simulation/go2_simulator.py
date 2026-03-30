"""
Unitree Go2 Simulator
====================
Main simulation environment for testing behaviors before deploying to real hardware.

This provides a safe environment to test:
- Bipedal walking
- Backflips
- Other advanced movements

SAFETY FIRST: Always test in simulation before trying on real robot!
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.utils.safety_monitor import SafetyMonitor
from simulation.controllers.behaviors import (
    StandingBehavior,
    BipedalTransition,
    BipedalWalking,
    JumpPreparation,
    BackflipBehavior
)


class Go2Simulator:
    """
    Main simulator class for Unitree Go2.

    This handles:
    - PyBullet environment setup
    - Robot loading
    - Behavior execution
    - Safety monitoring
    - Visualization
    """

    def __init__(self, gui=True, real_time=True):
        """
        Initialize simulator.

        Args:
            gui: Show 3D visualization window (True) or run headless (False)
            real_time: Run at real-time speed (True) or as fast as possible (False)
        """
        self.gui = gui
        self.real_time = real_time

        # Connect to PyBullet
        if gui:
            self.client = p.connect(p.GUI)
            # Configure camera for better view
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Hide GUI controls
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-20,
                cameraTargetPosition=[0, 0, 0.3]
            )
        else:
            self.client = p.connect(p.DIRECT)

        print(f"✓ PyBullet connected (client ID: {self.client})")

        # Set up physics
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)  # 240 Hz physics simulation
        p.setRealTimeSimulation(0)  # We'll step manually for control

        # Load ground plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_id = p.loadURDF("plane.urdf")

        # Generate and load robot
        self.robot_id = self._load_robot()

        # Initialize safety monitor
        self.safety = SafetyMonitor(self.robot_id, max_body_tilt=60.0)

        # Behavior management
        self.current_behavior = None
        self.behavior_queue = []

        print("✓ Simulator initialized")
        print(f"  - Real-time mode: {real_time}")
        print(f"  - GUI enabled: {gui}")

    def _load_robot(self):
        """Generate URDF and load robot into simulation."""
        # Generate URDF
        from simulation.models.go2_urdf_generator import save_urdf
        urdf_path = save_urdf()

        # Load robot at starting position
        start_pos = [0, 0, 0.45]  # Start slightly above ground
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        robot_id = p.loadURDF(
            urdf_path,
            start_pos,
            start_orientation,
            useFixedBase=False,  # Robot can move freely
            flags=p.URDF_USE_SELF_COLLISION  # Enable self-collision detection
        )

        print(f"✓ Robot loaded (ID: {robot_id})")
        print(f"  - Number of joints: {p.getNumJoints(robot_id)}")

        # Set initial joint positions (standing pose)
        self._set_initial_pose(robot_id)

        return robot_id

    def _set_initial_pose(self, robot_id):
        """Set robot to a stable standing pose."""
        from simulation.controllers.kinematics import QuadrupedIK

        ik = QuadrupedIK()
        standing_angles = ik.standing_pose(height=0.35)

        # Get joint indices
        joint_indices = {}
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_indices[joint_name] = i

        # Set each joint
        for leg_name, angles in standing_angles.items():
            hip_angle, thigh_angle, calf_angle = angles
            joints = [
                (f"{leg_name}_hip_joint", hip_angle),
                (f"{leg_name}_thigh_joint", thigh_angle),
                (f"{leg_name}_calf_joint", calf_angle)
            ]
            for joint_name, angle in joints:
                if joint_name in joint_indices:
                    p.resetJointState(robot_id, joint_indices[joint_name], angle)

        print("✓ Robot set to standing pose")

    def queue_behavior(self, behavior):
        """
        Add a behavior to the execution queue.

        Args:
            behavior: Instance of a behavior controller
        """
        self.behavior_queue.append(behavior)
        print(f"✓ Queued behavior: {behavior.__class__.__name__}")

    def run_behavior_sequence(self, behaviors, safety_checks=True):
        """
        Run a sequence of behaviors.

        Args:
            behaviors: List of behavior controller instances
            safety_checks: Enable safety monitoring (recommended!)

        Returns:
            bool: True if all behaviors completed safely
        """
        print("\n" + "="*60)
        print("STARTING BEHAVIOR SEQUENCE")
        print("="*60)

        for i, behavior in enumerate(behaviors):
            print(f"\n[{i+1}/{len(behaviors)}] {behavior.__class__.__name__}")
            print("-" * 40)

            success = self.run_behavior(behavior, safety_checks)

            if not success:
                print(f"\n❌ Behavior {i+1} failed or unsafe - stopping sequence")
                return False

            # Brief pause between behaviors
            for _ in range(60):  # 0.25 seconds at 240Hz
                p.stepSimulation()
                if self.real_time:
                    time.sleep(1/240)

        print("\n" + "="*60)
        print("✓ ALL BEHAVIORS COMPLETED SUCCESSFULLY")
        print("="*60 + "\n")
        return True

    def run_behavior(self, behavior, safety_checks=True):
        """
        Run a single behavior until completion.

        Args:
            behavior: Behavior controller instance
            safety_checks: Monitor safety during execution

        Returns:
            bool: True if behavior completed safely
        """
        self.current_behavior = behavior
        behavior.reset()

        dt = 1/240  # Time step (240 Hz)
        max_duration = 30.0  # Safety timeout
        elapsed = 0

        while not behavior.is_complete and elapsed < max_duration:
            # Update behavior
            status = behavior.update(dt)

            # Print status (update same line)
            print(f"\r  {status}                    ", end='', flush=True)

            # Run safety checks
            if safety_checks:
                is_safe = self.safety.check_all()

                if not is_safe:
                    print("\n")
                    self.safety.print_status(verbose=True)
                    print("⚠️  Safety violation detected!")

                    # For critical errors, stop immediately
                    if self.safety.critical_errors:
                        self.safety.emergency_stop()
                        return False

            # Step simulation
            p.stepSimulation()

            # Real-time delay if enabled
            if self.real_time:
                time.sleep(dt)

            elapsed += dt

        print()  # New line after status updates

        if elapsed >= max_duration:
            print(f"⚠️  Behavior timed out after {max_duration}s")
            return False

        return True

    def get_robot_state(self):
        """
        Get current robot state information.

        Returns:
            dict: Robot state including position, orientation, joint angles
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

        # Get joint states
        joint_states = {}
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            state = p.getJointState(self.robot_id, i)
            joint_states[joint_name] = {
                'position': state[0],
                'velocity': state[1]
            }

        return {
            'position': pos,
            'orientation_euler': euler,
            'linear_velocity': lin_vel,
            'angular_velocity': ang_vel,
            'joint_states': joint_states
        }

    def reset_robot(self):
        """Reset robot to initial standing pose."""
        print("\n🔄 Resetting robot...")

        # Reset position
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [0, 0, 0.45],
            p.getQuaternionFromEuler([0, 0, 0])
        )

        # Reset velocity
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        # Reset joints
        self._set_initial_pose(self.robot_id)

        print("✓ Robot reset complete")

    def wait(self, duration=1.0):
        """
        Wait for a specified duration while maintaining current pose.

        Args:
            duration: Time to wait in seconds
        """
        steps = int(duration * 240)
        for _ in range(steps):
            p.stepSimulation()
            if self.real_time:
                time.sleep(1/240)

    def disconnect(self):
        """Close the simulation."""
        p.disconnect()
        print("\n✓ Simulator disconnected")


def main():
    """
    Main demo function showing different behaviors.
    """
    print("\n" + "="*60)
    print("UNITREE GO2 SIMULATION - BIPEDAL WALKING & BACKFLIP")
    print("="*60 + "\n")

    # Create simulator
    sim = Go2Simulator(gui=True, real_time=True)

    try:
        # Wait for robot to settle
        print("⏳ Settling robot...")
        sim.wait(2.0)

        # Create behavior sequence
        behaviors = [
            # 1. Start with stable standing
            StandingBehavior(sim.robot_id, height=0.35),

            # 2. Transition to bipedal stance
            BipedalTransition(sim.robot_id, duration=3.0),

            # 3. Walk on hind legs
            BipedalWalking(sim.robot_id, step_duration=1.5, num_steps=3),

            # 4. Prepare for backflip
            JumpPreparation(sim.robot_id, duration=0.8),

            # 5. BACKFLIP!
            BackflipBehavior(sim.robot_id, duration=2.0),
        ]

        # Manually run first behavior (standing) for a bit
        print("\n📍 Phase: Initial Standing")
        standing = behaviors[0]
        for _ in range(480):  # 2 seconds
            standing.update(1/240)
            p.stepSimulation()
            time.sleep(1/240)

        # Run the main sequence
        success = sim.run_behavior_sequence(behaviors[1:], safety_checks=True)

        if success:
            print("\n🎉 DEMONSTRATION COMPLETE!")
            print("\n⚠️  IMPORTANT NOTES FOR REAL HARDWARE:")
            print("  1. Start with simple behaviors (standing, gentle movements)")
            print("  2. Use safety harness for bipedal and backflip attempts")
            print("  3. Have emergency stop ready at all times")
            print("  4. Test on soft surface (grass, foam mat)")
            print("  5. Gradually increase difficulty - don't jump straight to backflips!")
            print("\n💡 The simulation is a guide, but real physics may differ.")
        else:
            print("\n⚠️  Demonstration ended due to safety concern or failure")

        # Keep window open
        print("\n👁️  Close the simulation window to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1/60)

    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
