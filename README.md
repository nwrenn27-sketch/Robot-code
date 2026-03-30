# Unitree Go2 Bipedal Walking & Backflip Simulation

**Safe simulation environment for testing advanced quadruped robot behaviors before deploying to real hardware.**

![Status](https://img.shields.io/badge/status-experimental-orange)
![Python](https://img.shields.io/badge/python-3.7%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

## ⚠️ Safety Warning

**ALWAYS test behaviors in simulation before attempting on real hardware!**

This project includes high-risk behaviors (bipedal walking, backflips) that can damage your robot or cause injury if not executed properly. Please:

- ✅ Test thoroughly in simulation first
- ✅ Use safety harness when testing on real robot
- ✅ Work with experienced supervisor
- ✅ Have emergency stop ready
- ✅ Use soft landing surfaces
- ❌ Never attempt backflips without proper safety measures

## 🎯 Project Goals

This simulation helps you:

1. **Visualize** advanced robot behaviors before running them on hardware
2. **Test** bipedal walking and dynamic movements safely
3. **Understand** the joint angles and physics involved
4. **Develop** behaviors that can be adapted to the real Unitree Go2 SDK

## 🚀 Features

### Implemented Behaviors

- ✅ **Standing**: Stable quadruped standing pose
- ✅ **Bipedal Transition**: Smooth transition to standing on hind legs
- ✅ **Bipedal Walking**: Walking on two hind legs with alternating gait
- ✅ **Jump Preparation**: Crouch and prepare for explosive movements
- ✅ **Backflip**: Full backward rotation with tuck and landing

### Safety Features

- 🛡️ Real-time joint limit monitoring
- 🛡️ Body orientation checking (detects falls)
- 🛡️ Velocity and torque limiting
- 🛡️ Emergency stop functionality
- 🛡️ Comprehensive safety warnings

## 📦 Installation

### Requirements

- Python 3.7 or higher
- PyBullet (free, open-source physics engine)
- NumPy

### Setup

1. **Clone this repository**:
   ```bash
   git clone https://github.com/nwrenn27-sketch/Robot-code.git
   cd Robot-code
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Test the installation**:
   ```bash
   python simulation/go2_simulator.py
   ```

You should see a 3D visualization window with the robot performing the full behavior sequence!

## 🎮 Usage

### Quick Start - Full Demo

Run the complete demonstration (bipedal walking + backflip):

```bash
python simulation/go2_simulator.py
```

### Test Individual Behaviors

**Bipedal Walking Only** (safer to start with):
```bash
python simulation/examples/test_bipedal_only.py
```

**Backflip Only** (advanced):
```bash
python simulation/examples/test_backflip_only.py
```

### Custom Behavior Script

Create your own test by combining behaviors:

```python
from simulation.go2_simulator import Go2Simulator
from simulation.controllers.behaviors import *

# Create simulator
sim = Go2Simulator(gui=True, real_time=True)

# Create custom behavior sequence
behaviors = [
    StandingBehavior(sim.robot_id, height=0.35),
    BipedalTransition(sim.robot_id, duration=3.0),
    # Add more behaviors here...
]

# Run behaviors
sim.run_behavior_sequence(behaviors, safety_checks=True)
```

## 📁 Project Structure

```
JailBreakDoggo1/
├── simulation/
│   ├── models/
│   │   ├── go2_urdf_generator.py    # Generates robot model
│   │   └── unitree_go2.urdf         # Generated robot description
│   ├── controllers/
│   │   ├── kinematics.py            # Inverse kinematics solver
│   │   └── behaviors.py             # Behavior implementations
│   ├── utils/
│   │   └── safety_monitor.py        # Safety checking system
│   ├── examples/
│   │   ├── test_bipedal_only.py     # Bipedal walking test
│   │   └── test_backflip_only.py    # Backflip test
│   └── go2_simulator.py             # Main simulator
├── requirements.txt
└── README.md
```

## 🎓 Understanding the Code

### Key Components

#### 1. **Kinematics (`kinematics.py`)**
Calculates joint angles needed to place feet at desired positions.

```python
from simulation.controllers.kinematics import QuadrupedIK

ik = QuadrupedIK()
# Get joint angles for standing at 0.35m height
angles = ik.standing_pose(height=0.35)
```

#### 2. **Behaviors (`behaviors.py`)**
High-level behaviors broken into phases.

Each behavior has an `update(dt)` method called each simulation step:

```python
class MyBehavior(BehaviorController):
    def update(self, dt):
        # Your behavior logic here
        joint_angles = self.ik.standing_pose(0.35)
        self.set_joint_angles(joint_angles)
        return "Status message"
```

#### 3. **Safety Monitor (`safety_monitor.py`)**
Monitors robot state and prevents dangerous movements.

```python
safety = SafetyMonitor(robot_id)
is_safe = safety.check_all()  # Check all safety constraints
```

### Behavior Phases

#### Bipedal Transition
1. **Crouch** (30% of duration): Lower body to shift weight
2. **Shift Weight** (30%): Move center of mass over hind legs
3. **Lift Front** (25%): Raise front legs off ground
4. **Stand** (15%): Stabilize in bipedal stance

#### Backflip
1. **Jump** (15%): Explosive leg extension + upward force
2. **Rotate** (35%): Tuck legs + apply rotation torque
3. **Extend** (30%): Extend legs for landing
4. **Land** (20%): Absorb impact with high damping

## 🔧 Adapting to Real Robot

The simulation uses joint positions and velocities similar to the Unitree SDK. To adapt behaviors to real hardware:

### 1. Get Joint Angles from Simulation

```python
# In simulation
state = sim.get_robot_state()
joint_angles = state['joint_states']

# This gives you angles for each joint:
# {'FR_hip_joint': {'position': 0.1, 'velocity': 0.0}, ...}
```

### 2. Map to Unitree SDK Commands

```python
# Example mapping (you'll need to work with your teacher on this)
from unitree_sdk import Robot  # Hypothetical - check actual SDK

robot = Robot()

# Set joint position for front right hip
robot.set_joint_position('FR_hip', angle)
```

### 3. Key Differences to Consider

| Simulation | Real Robot |
|------------|------------|
| Perfect sensors | Sensor noise/delay |
| Instant torque | Motor response time |
| Simplified friction | Complex contact dynamics |
| No battery limits | Battery affects performance |

**Always start with slower, gentler movements on real hardware!**

## 🐛 Troubleshooting

### Robot Falls Over in Simulation

- **Cause**: Transition too fast or unstable pose
- **Fix**: Increase behavior duration or adjust joint angles

### "CRITICAL: Body tilt too extreme"

- **Cause**: Safety system detected potential fall
- **Fix**: This is working as intended! Review the behavior that caused it.

### Import Errors

```bash
# Make sure you're in the project root directory
cd JailBreakDoggo1

# Try running with python -m
python -m simulation.go2_simulator
```

### Robot Model Doesn't Load

```bash
# Regenerate the URDF
python simulation/models/go2_urdf_generator.py
```

## 📚 Learning Resources

### PyBullet
- [PyBullet Quickstart](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
- [PyBullet Documentation](https://pybullet.org/)

### Quadruped Robotics
- [Unitree Robotics Official Site](https://www.unitree.com/)
- [Inverse Kinematics Tutorial](https://robotacademy.net.au/)

### Safety Practices
- Always work with supervision
- Test incrementally (don't jump to backflips!)
- Document what works and what doesn't

## 🤝 Contributing

This is an educational project! Contributions welcome:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-behavior`)
3. Commit your changes (`git commit -m 'Add amazing behavior'`)
4. Push to branch (`git push origin feature/amazing-behavior`)
5. Open a Pull Request

## ⚖️ License

MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- **Unitree Robotics** for creating the Go2 robot
- **PyBullet** team for the excellent physics engine
- Your **teacher** for supporting this ambitious project!

## 📞 Support

Having issues? Found a bug?

1. Check the [Troubleshooting](#-troubleshooting) section
2. Open an issue on GitHub
3. Ask your teacher/supervisor for guidance

## ⚠️ Final Safety Reminder

**This simulation is a tool for learning and testing, but:**

- Simulated physics ≠ Real physics
- Start with simple behaviors on real robot
- Use proper safety equipment
- Work with experienced supervision
- Have emergency stop always ready

**Your safety and the robot's safety come first!** 🛡️

---

**Good luck with your bipedal robot adventures!** 🤖🦘

*Built with ❤️ for learning and safe robotics experimentation*
