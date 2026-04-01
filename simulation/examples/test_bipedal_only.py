"""
Test Bipedal Walking Only
=========================
This script tests ONLY the bipedal walking behavior.

Use this to:
1. Practice transitioning to bipedal stance
2. Test bipedal walking without the risk of backflips
3. Understand the joint angles and movements

This is safer to start with!
"""

# Import sys module - needed to modify Python's import path
import sys
# Import os module - needed for file path operations
import os

# Add the project root directory to Python's import path
# This allows us to import from the simulation package
# __file__ = this script's path
# abspath = convert to absolute path
# dirname = get directory (repeat 3 times to go up 3 levels)
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Import the main simulator class
from simulation.go2_simulator import Go2Simulator
# Import the behavior classes we'll use
from simulation.controllers.behaviors import (
    StandingBehavior,     # Makes robot stand in place
    BipedalTransition,    # Transitions from 4 legs to 2 legs
    BipedalWalking        # Walks on hind legs only
)


def main():
    # Print a header banner for the test
    print("\n" + "="*60)  # Line of 60 equal signs
    print("BIPEDAL WALKING TEST")
    print("="*60 + "\n")

    # Create the simulator
    # gui=True: Show the 3D visualization window
    # real_time=True: Run at real-time speed (not as fast as possible)
    sim = Go2Simulator(gui=True, real_time=True)

    # Use try-finally to ensure we always disconnect properly
    try:
        # Wait for the robot to settle (physics stabilization)
        # When the robot is first loaded, it might wobble slightly
        # Waiting 2 seconds lets it settle into a stable stance
        print("⏳ Settling robot...")
        sim.wait(2.0)

        # Print what we're about to do
        print("\n📋 Behavior Sequence:")
        print("  1. Standing (2s)")
        print("  2. Transition to bipedal (3s)")
        print("  3. Walk on hind legs (6s)")
        print()

        # Create a list of behaviors to execute in sequence
        behaviors = [
            # Behavior 1: Stand at 0.35m height (normal standing)
            StandingBehavior(sim.robot_id, height=0.35),
            # Behavior 2: Transition to bipedal stance over 3 seconds
            BipedalTransition(sim.robot_id, duration=3.0),
            # Behavior 3: Walk bipedally - 4 steps, 1.5 seconds per step
            BipedalWalking(sim.robot_id, step_duration=1.5, num_steps=4),
        ]

        # Run the standing behavior manually first (for 2 seconds)
        print("📍 Phase 1: Standing")
        standing = behaviors[0]  # Get the first behavior (StandingBehavior)

        # Import time and pybullet here (lazy import)
        import time
        import pybullet as p

        # Run standing behavior for 480 steps
        # 480 steps × 1/240 seconds/step = 2 seconds
        for _ in range(480):
            # Update the behavior for one time step (1/240 second)
            standing.update(1/240)
            # Step the physics simulation forward
            p.stepSimulation()
            # Wait 1/240 second to match real time
            time.sleep(1/240)

        # Run the remaining behaviors (bipedal transition and walking)
        # behaviors[1:] means "from index 1 to the end" (skips the first one)
        # safety_checks=True: Monitor for safety violations
        success = sim.run_behavior_sequence(behaviors[1:], safety_checks=True)

        # Check if all behaviors completed successfully
        if success:
            # Print success message and what to look for
            print("\n✓ Test completed successfully!")
            print("\n📊 What to observe:")
            print("  - Body should lean back during transition")
            print("  - Front legs should lift smoothly")
            print("  - Robot should maintain balance on hind legs")
            print("  - Walking should be smooth and controlled")
            print("\n💡 If the robot falls, the transition speed or")
            print("   joint angles may need adjustment.")
        else:
            # Behaviors failed or were unsafe
            print("\n⚠️  Test failed - check the warnings above")

        # Keep the visualization window open so you can see the final result
        # This infinite loop keeps stepping the simulation (robot stays frozen in place)
        print("\n👁️  Close window to exit...")
        while True:
            # Step simulation forward (keeps physics running, robot stays still)
            p.stepSimulation()
            # Sleep for 1/60 second (60 Hz refresh rate)
            time.sleep(1/60)

    # Catch Ctrl+C (keyboard interrupt) to exit gracefully
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")
    # Finally block always executes, even if there's an error
    finally:
        # Disconnect from PyBullet to clean up resources
        sim.disconnect()


# This checks if the script is being run directly (not imported as a module)
if __name__ == "__main__":
    # If run directly, execute the main function
    main()
