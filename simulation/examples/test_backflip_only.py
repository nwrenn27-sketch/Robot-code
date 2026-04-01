"""
Test Backflip Only
==================
This script tests ONLY the backflip behavior.

⚠️  WARNING: This is an advanced, high-risk behavior!

IMPORTANT:
- Test thoroughly in simulation first
- Use safety harness on real robot
- Have emergency stop ready
- Use soft landing surface
- Don't attempt without supervision
"""

# Import sys module - needed to modify Python's import path
import sys
# Import os module - needed for file path operations
import os
# Add the project root directory to Python's import path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Import the main simulator class
from simulation.go2_simulator import Go2Simulator
# Import the behavior classes we need for backflip
from simulation.controllers.behaviors import (
    StandingBehavior,     # Stand in place before jumping
    JumpPreparation,      # Crouch down to prepare for explosive jump
    BackflipBehavior      # THE BACKFLIP!
)


def main():
    # Print warning header
    print("\n" + "="*60)
    print("⚠️  BACKFLIP TEST - HIGH RISK BEHAVIOR ⚠️")
    print("="*60 + "\n")

    # Ask for confirmation - this is a dangerous behavior!
    # input() waits for the user to type something and press Enter
    response = input("Are you sure you want to test backflip? (yes/no): ")
    # .lower() converts to lowercase so "YES", "Yes", "yes" all work
    if response.lower() != 'yes':
        # If they didn't type "yes", cancel the test
        print("Test cancelled.")
        return  # Exit the function early

    # Create the simulator with GUI and real-time mode
    sim = Go2Simulator(gui=True, real_time=True)

    # Use try-finally to ensure cleanup
    try:
        # Wait for robot to settle (physics stabilization)
        print("\n⏳ Settling robot...")
        sim.wait(2.0)

        # Print what we're about to do
        print("\n📋 Behavior Sequence:")
        print("  1. Standing (1s)")
        print("  2. Crouch preparation (0.8s)")
        print("  3. BACKFLIP! (2s)")
        print()

        # Create the behavior sequence
        behaviors = [
            # Behavior 1: Stand at normal height
            StandingBehavior(sim.robot_id, height=0.35),
            # Behavior 2: Crouch down to prepare for jump (0.8 seconds)
            JumpPreparation(sim.robot_id, duration=0.8),
            # Behavior 3: PERFORM THE BACKFLIP! (2 seconds total)
            BackflipBehavior(sim.robot_id, duration=2.0),
        ]

        # Run standing behavior manually for 1 second
        print("📍 Phase 1: Standing")
        standing = behaviors[0]

        # Import time and pybullet
        import time
        import pybullet as p

        # Run standing for 240 steps = 1 second
        for _ in range(240):
            standing.update(1/240)  # Update behavior
            p.stepSimulation()       # Step physics
            time.sleep(1/240)        # Wait to match real time

        # Run preparation and backflip behaviors
        # behaviors[1:] = skip standing, run preparation and backflip
        success = sim.run_behavior_sequence(behaviors[1:], safety_checks=True)

        # Check if backflip completed successfully
        if success:
            # Success! Print celebration and analysis questions
            print("\n🎉 BACKFLIP COMPLETED!")
            print("\n📊 Check the simulation:")
            print("  - Did the robot complete full rotation?")
            print("  - Did it land on its feet?")
            print("  - Was the landing stable?")
            print("\n⚠️  BEFORE TRYING ON REAL ROBOT:")
            print("  1. Use safety harness/support")
            print("  2. Test on soft surface (foam mat, grass)")
            print("  3. Start with lower jumps first")
            print("  4. Have emergency stop ready")
            print("  5. Work with your teacher/supervisor")
        else:
            # Failed! DO NOT try on real robot
            print("\n❌ Backflip failed in simulation")
            print("   DO NOT attempt on real robot yet!")

        # Get final robot state to see where it ended up
        state = sim.get_robot_state()
        # Extract the Z position (height) from the position tuple
        # state['position'] is [x, y, z], we want z (index 2)
        # :.3f formats the number to 3 decimal places
        print(f"\n📍 Final height: {state['position'][2]:.3f}m")

        # Keep the visualization window open
        print("\n👁️  Close window to exit...")
        while True:
            # Keep stepping simulation to maintain visualization
            p.stepSimulation()
            # Sleep 1/60 second (60 Hz)
            time.sleep(1/60)

    # Catch keyboard interrupt (Ctrl+C)
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")
    # Finally block always runs - clean up resources
    finally:
        # Disconnect from PyBullet simulation
        sim.disconnect()


# Check if this script is being run directly
if __name__ == "__main__":
    # If run directly, execute the main function
    main()
