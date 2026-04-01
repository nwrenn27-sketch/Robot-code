# Unitree Go2 Robot Simulation 🤖

A safe simulation environment for testing advanced robot behaviors before deploying to real hardware.

## What This Project Does

This project simulates a **Unitree Go2 quadruped robot** (a four-legged robot dog) and lets you test cool movements like:

- 🚶 **Bipedal Walking** - Standing and walking on just the back two legs (like a person!)
- 🤸 **Backflips** - Full backward rotation in the air
- 🦘 **Jumping** - Dynamic explosive movements
- 🧍 **Advanced Standing** - Balanced poses and transitions

**Why simulation?** Testing these behaviors on a real robot without practice is dangerous and could break the $3,000+ robot. This lets you experiment safely!

## Project Structure

```
JailBreakDoggo1/
├── simulation/
│   ├── models/
│   │   └── go2_urdf_generator.py      # Creates the robot model
│   ├── controllers/
│   │   ├── kinematics.py              # Calculates joint angles
│   │   └── behaviors.py               # Robot behaviors (walk, flip, etc.)
│   ├── utils/
│   │   └── safety_monitor.py          # Monitors for dangerous movements
│   ├── examples/
│   │   ├── test_bipedal_only.py       # Test walking on two legs
│   │   └── test_backflip_only.py      # Test backflips
│   └── go2_simulator.py               # Main simulator
├── requirements.txt                    # Python packages needed
└── README.md                          # This file!
```

## How It Works

### 1. **Physics Simulation (PyBullet)**
- Uses PyBullet physics engine to simulate real-world physics
- Gravity, friction, collisions, and forces all work like reality
- Runs at 240 Hz (240 times per second) for accuracy

### 2. **Robot Model (URDF)**
- Robot is defined in URDF format (XML file describing robot structure)
- Includes body, 4 legs, 12 joints (3 per leg: hip, thigh, calf)
- Based on real Unitree Go2 specifications

### 3. **Inverse Kinematics (IK)**
- Calculates joint angles needed to place feet at desired positions
- Uses trigonometry and the Law of Cosines
- Answers: "To put the foot HERE, what angles do I need?"

### 4. **Behaviors**
- High-level actions broken into phases
- Example: Bipedal transition has 4 phases (crouch → shift weight → lift front legs → stabilize)
- Each behavior updates 240 times per second

### 5. **Safety Monitoring**
- Checks joint limits (can't bend joints too far)
- Monitors body tilt (detects if robot is falling)
- Checks height (detects if robot has collapsed)
- Emergency stop if anything goes wrong

## Installation

### Requirements
- **Python 3.7+**
- **PyBullet** (physics engine)
- **NumPy** (math library)
- **Matplotlib** (graphing)

### Quick Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/nwrenn27-sketch/Robot-code.git
   cd JailBreakDoggo1
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Run a test:**
   ```bash
   # Test bipedal walking (safer to start with)
   python simulation/examples/test_bipedal_only.py

   # Test backflip (advanced!)
   python simulation/examples/test_backflip_only.py
   ```

## Usage Examples

### Run Full Demo (Walking + Backflip)
```bash
python simulation/go2_simulator.py
```

### Test Bipedal Walking Only
```bash
python simulation/examples/test_bipedal_only.py
```

### Create Custom Behavior
```python
from simulation.go2_simulator import Go2Simulator
from simulation.controllers.behaviors import *

# Create simulator
sim = Go2Simulator(gui=True, real_time=True)

# Define custom behavior sequence
behaviors = [
    StandingBehavior(sim.robot_id, height=0.35),
    BipedalTransition(sim.robot_id, duration=3.0),
    BipedalWalking(sim.robot_id, step_duration=1.5, num_steps=4),
]

# Run behaviors with safety monitoring
sim.run_behavior_sequence(behaviors, safety_checks=True)
```

## Key Concepts Explained

### Inverse Kinematics (IK)
**Problem:** "I want the foot at position (x, y, z). What joint angles do I need?"

**Solution:** Use trigonometry to work backwards from desired position to joint angles.

```
Forward Kinematics:  Joint Angles → Foot Position (easy)
Inverse Kinematics:  Foot Position → Joint Angles (hard!)
```

### PD Control
**P (Proportional):** How aggressively to reach the target
- Higher kp = stiffer, reaches target faster
- Lower kp = softer, slower motion

**D (Derivative):** How much to resist oscillation
- Higher kd = more damping, less bouncing
- Lower kd = less damping, might oscillate

### Behavior Phases
Complex behaviors are broken into phases that execute over time:

**Bipedal Transition Example:**
1. **Crouch** (30% of time) - Lower body to shift weight
2. **Shift Weight** (30%) - Move center of mass over hind legs
3. **Lift Front** (25%) - Raise front legs off ground
4. **Stand** (15%) - Stabilize in bipedal stance

## Safety Features

⚠️ **IMPORTANT:** Always test in simulation before real robot!

The simulator includes safety monitoring:

- ✅ **Joint Limit Checking** - Prevents joints from exceeding safe angles
- ✅ **Body Orientation Monitoring** - Detects if robot is falling over
- ✅ **Height Checking** - Detects if robot has collapsed
- ✅ **Velocity Limiting** - Prevents dangerous speeds
- ✅ **Emergency Stop** - Immediately stops all motion if unsafe

## Understanding the Code

All code files have **detailed line-by-line comments** explaining:
- What each line does
- Why it's there
- How the math works
- What values mean in the real world

### Example Comment Style:
```python
# Calculate hip angle using arctangent
# arctan2(z, y) gives us the angle in the y-z plane
# This determines how far the leg swings outward from the body
hip_angle = np.arctan2(z, y_adjusted)
```

Pick any random line and you'll understand what it's doing!

## Technical Details

### Simulation Parameters
- **Physics Rate:** 240 Hz (4.17ms per step)
- **Gravity:** -9.81 m/s² (Earth gravity)
- **Robot Mass:** ~15 kg
- **Leg Segments:** Hip: 8cm, Thigh: 21.3cm, Calf: 21.3cm

### Joint Limits (approximate)
- **Hip:** -46° to +46° (sideways movement)
- **Thigh:** -86° to +172° (forward/backward)
- **Calf:** -155° to -46° (knee bends backward only)

### Forces in Backflip
- **Jump Force:** 500 N upward (robot weighs ~147 N)
- **Rotation Torque:** -150 N⋅m backward pitch
- **Duration:** 2 seconds total (jump → rotate → land)

## Learning Resources

### Understanding the Physics
- [PyBullet Documentation](https://pybullet.org/)
- [Inverse Kinematics Explained](https://robotacademy.net.au/)

### Robot Specifications
- [Unitree Go2 Official](https://www.unitree.com/)

### Mathematics Used
- **Trigonometry:** sin, cos, arctan, arctan2
- **Law of Cosines:** For IK calculations
- **Linear Interpolation:** For smooth transitions

## Real Robot Deployment

⚠️ **Before trying on real hardware:**

1. ✅ Test thoroughly in simulation
2. ✅ Start with simple behaviors (standing, gentle movements)
3. ✅ Use safety harness for bipedal/backflip attempts
4. ✅ Have emergency stop ready
5. ✅ Test on soft surface (foam mat, grass)
6. ✅ Work with experienced supervisor
7. ✅ Gradually increase difficulty

**DO NOT jump straight to backflips on real hardware!** 🚨

## Troubleshooting

### Robot Falls Over in Simulation
- **Cause:** Transition too fast or unstable pose
- **Fix:** Increase behavior duration or adjust joint angles

### "CRITICAL: Body tilt too extreme"
- **Cause:** Safety system detected potential fall
- **Fix:** This is working correctly! Review what caused it.

### Import Errors
```bash
# Make sure you're in project root
cd JailBreakDoggo1

# Try running with python -m
python -m simulation.go2_simulator
```

### Robot Model Doesn't Load
```bash
# Regenerate URDF
python simulation/models/go2_urdf_generator.py
```

## Contributing

Contributions welcome! This is an educational project.

1. Fork the repository
2. Create feature branch (`git checkout -b feature/cool-behavior`)
3. Add detailed comments to new code
4. Test in simulation
5. Commit changes (`git commit -m 'Add cool behavior'`)
6. Push to branch (`git push origin feature/cool-behavior`)
7. Open Pull Request

## License

MIT License - See LICENSE file for details

## Acknowledgments

- **Unitree Robotics** - For creating the Go2 robot
- **PyBullet Team** - For the excellent physics engine
- Educational project for learning robotics and simulation

## Contact

- **GitHub:** [nwrenn27-sketch/Robot-code](https://github.com/nwrenn27-sketch/Robot-code)
- **Issues:** Report bugs or ask questions via GitHub Issues

---

## Quick Start Summary

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run a test
python simulation/examples/test_bipedal_only.py

# 3. Watch the robot walk on two legs!
# 4. Close the window when done
```

**Remember: Simulation first, real robot later!** 🛡️

Built with ❤️ for safe robotics experimentation and learning.
