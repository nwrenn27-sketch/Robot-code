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

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from simulation.go2_simulator import Go2Simulator
from simulation.controllers.behaviors import (
    StandingBehavior,
    JumpPreparation,
    BackflipBehavior
)


def main():
    print("\n" + "="*60)
    print("⚠️  BACKFLIP TEST - HIGH RISK BEHAVIOR ⚠️")
    print("="*60 + "\n")

    response = input("Are you sure you want to test backflip? (yes/no): ")
    if response.lower() != 'yes':
        print("Test cancelled.")
        return

    # Create simulator
    sim = Go2Simulator(gui=True, real_time=True)

    try:
        # Wait for settling
        print("\n⏳ Settling robot...")
        sim.wait(2.0)

        print("\n📋 Behavior Sequence:")
        print("  1. Standing (1s)")
        print("  2. Crouch preparation (0.8s)")
        print("  3. BACKFLIP! (2s)")
        print()

        behaviors = [
            StandingBehavior(sim.robot_id, height=0.35),
            JumpPreparation(sim.robot_id, duration=0.8),
            BackflipBehavior(sim.robot_id, duration=2.0),
        ]

        # Run standing
        print("📍 Phase 1: Standing")
        standing = behaviors[0]
        import time
        import pybullet as p
        for _ in range(240):  # 1 second
            standing.update(1/240)
            p.stepSimulation()
            time.sleep(1/240)

        # Run preparation and backflip
        success = sim.run_behavior_sequence(behaviors[1:], safety_checks=True)

        if success:
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
            print("\n❌ Backflip failed in simulation")
            print("   DO NOT attempt on real robot yet!")

        # Get final state
        state = sim.get_robot_state()
        print(f"\n📍 Final height: {state['position'][2]:.3f}m")

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
