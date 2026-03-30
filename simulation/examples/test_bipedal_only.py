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

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from simulation.go2_simulator import Go2Simulator
from simulation.controllers.behaviors import (
    StandingBehavior,
    BipedalTransition,
    BipedalWalking
)


def main():
    print("\n" + "="*60)
    print("BIPEDAL WALKING TEST")
    print("="*60 + "\n")

    # Create simulator
    sim = Go2Simulator(gui=True, real_time=True)

    try:
        # Wait for settling
        print("⏳ Settling robot...")
        sim.wait(2.0)

        # Create behavior sequence
        print("\n📋 Behavior Sequence:")
        print("  1. Standing (2s)")
        print("  2. Transition to bipedal (3s)")
        print("  3. Walk on hind legs (6s)")
        print()

        behaviors = [
            StandingBehavior(sim.robot_id, height=0.35),
            BipedalTransition(sim.robot_id, duration=3.0),
            BipedalWalking(sim.robot_id, step_duration=1.5, num_steps=4),
        ]

        # Run standing behavior first
        print("📍 Phase 1: Standing")
        standing = behaviors[0]
        import time
        import pybullet as p
        for _ in range(480):  # 2 seconds
            standing.update(1/240)
            p.stepSimulation()
            time.sleep(1/240)

        # Run the rest
        success = sim.run_behavior_sequence(behaviors[1:], safety_checks=True)

        if success:
            print("\n✓ Test completed successfully!")
            print("\n📊 What to observe:")
            print("  - Body should lean back during transition")
            print("  - Front legs should lift smoothly")
            print("  - Robot should maintain balance on hind legs")
            print("  - Walking should be smooth and controlled")
            print("\n💡 If the robot falls, the transition speed or")
            print("   joint angles may need adjustment.")
        else:
            print("\n⚠️  Test failed - check the warnings above")

        # Keep window open
        print("\n👁️  Close window to exit...")
        while True:
            p.stepSimulation()
            time.sleep(1/60)

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")
    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
