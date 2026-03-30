"""
Unitree Go2 URDF Generator
==========================
This script generates a simplified URDF model for the Unitree Go2 robot.
Since we don't have the official URDF, we create one based on publicly available specs.

URDF = Unified Robot Description Format (XML file describing robot structure)

Unitree Go2 Specifications (approximate):
- Body length: 0.65m
- Body width: 0.28m
- Body height: 0.40m (standing)
- Leg length segments: Hip-Thigh: 0.08m, Thigh: 0.213m, Calf: 0.213m
- Weight: ~15kg
- 12 motors total (3 per leg: hip, thigh, calf)
"""

import os

def generate_go2_urdf():
    """
    Generate a simplified Unitree Go2 URDF file.
    This creates a robot with:
    - 1 body link
    - 4 legs (FR, FL, RR, RL = Front Right, Front Left, Rear Right, Rear Left)
    - 3 joints per leg (hip, thigh, calf) = 12 joints total
    """

    urdf_content = """<?xml version="1.0"?>
<robot name="unitree_go2">

  <!-- BASE BODY LINK -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.45"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="body_color">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

"""

    # Define leg parameters
    # FR = Front Right, FL = Front Left, RR = Rear Right, RL = Rear Left
    legs = {
        'FR': {'hip_pos': [0.18, -0.13, 0], 'hip_sign': 1},
        'FL': {'hip_pos': [0.18, 0.13, 0], 'hip_sign': -1},
        'RR': {'hip_pos': [-0.18, -0.13, 0], 'hip_sign': 1},
        'RL': {'hip_pos': [-0.18, 0.13, 0], 'hip_sign': -1}
    }

    # Segment lengths (in meters)
    hip_length = 0.08
    thigh_length = 0.213
    calf_length = 0.213

    for leg_name, leg_data in legs.items():
        x, y, z = leg_data['hip_pos']
        sign = leg_data['hip_sign']  # Used to mirror left/right legs

        # ============================================
        # HIP JOINT AND LINK (abduction/adduction - side to side)
        # ============================================
        urdf_content += f"""
  <!-- {leg_name} Hip Joint (moves leg sideways) -->
  <joint name="{leg_name}_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="{leg_name}_hip"/>
    <origin xyz="{x} {y} {z}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.8" upper="0.8" effort="25" velocity="21"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="{leg_name}_hip">
    <inertial>
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <mass value="0.8"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 {hip_length} 0.04"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.8 0.3 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 {hip_length} 0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- {leg_name} Thigh Joint (forward/backward movement) -->
  <joint name="{leg_name}_thigh_joint" type="revolute">
    <parent link="{leg_name}_hip"/>
    <child link="{leg_name}_thigh"/>
    <origin xyz="0 {sign * hip_length} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="3.0" effort="25" velocity="21"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="{leg_name}_thigh">
    <inertial>
      <origin xyz="0 0 -{thigh_length/2}" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -{thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="{thigh_length}"/>
      </geometry>
      <material name="limb_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -{thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="{thigh_length}"/>
      </geometry>
    </collision>
  </link>

  <!-- {leg_name} Calf Joint (knee joint) -->
  <joint name="{leg_name}_calf_joint" type="revolute">
    <parent link="{leg_name}_thigh"/>
    <child link="{leg_name}_calf"/>
    <origin xyz="0 0 -{thigh_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.7" upper="-0.8" effort="25" velocity="21"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="{leg_name}_calf">
    <inertial>
      <origin xyz="0 0 -{calf_length/2}" rpy="0 0 0"/>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.00005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -{calf_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="{calf_length}"/>
      </geometry>
      <material name="limb_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -{calf_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="{calf_length}"/>
      </geometry>
    </collision>
  </link>

  <!-- {leg_name} Foot (contact point with ground) -->
  <link name="{leg_name}_foot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="foot_color">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <!-- High friction for good ground contact -->
      <contact>
        <lateral_friction value="1.0"/>
        <restitution value="0.0"/>
      </contact>
    </collision>
  </link>

  <joint name="{leg_name}_foot_fixed" type="fixed">
    <parent link="{leg_name}_calf"/>
    <child link="{leg_name}_foot"/>
    <origin xyz="0 0 -{calf_length}" rpy="0 0 0"/>
  </joint>
"""

    urdf_content += "\n</robot>"

    return urdf_content


def save_urdf():
    """Save the generated URDF to a file"""
    urdf_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(urdf_dir, "unitree_go2.urdf")

    urdf_content = generate_go2_urdf()

    with open(urdf_path, 'w') as f:
        f.write(urdf_content)

    print(f"✓ URDF file created at: {urdf_path}")
    return urdf_path


if __name__ == "__main__":
    save_urdf()
