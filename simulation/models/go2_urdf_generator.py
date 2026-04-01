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

# Import the 'os' module - this lets us work with file paths and directories
import os

def generate_go2_urdf():
    """
    Generate a simplified Unitree Go2 URDF file.
    This creates a robot with:
    - 1 body link
    - 4 legs (FR, FL, RR, RL = Front Right, Front Left, Rear Right, Rear Left)
    - 3 joints per leg (hip, thigh, calf) = 12 joints total
    """

    # Create a string variable that will hold all the XML content for the URDF file
    # This is XML format - a markup language similar to HTML
    # The triple quotes allow us to write multi-line strings
    urdf_content = """<?xml version="1.0"?>
<robot name="unitree_go2">

  <!-- BASE BODY LINK -->
  <!-- This is the main body/torso of the robot - everything attaches to this -->
  <link name="base_link">
    <!-- Inertial properties define how the body moves and rotates -->
    <inertial>
      <!-- origin: where the center of mass is located (x, y, z) and rotation (roll, pitch, yaw) -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- mass: how heavy the body is in kilograms (10kg) -->
      <mass value="10.0"/>
      <!-- inertia: resistance to rotation around each axis (affects how easily it spins) -->
      <!-- ixx, iyy, izz = moment of inertia around x, y, z axes -->
      <!-- ixy, ixz, iyz = cross products (usually 0 for simple shapes) -->
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.45"/>
    </inertial>
    <!-- Visual properties define how the body looks in the 3D viewer -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!-- The body is a box: length=0.4m, width=0.2m, height=0.1m -->
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <!-- Material defines the color: RGB values (0.2, 0.2, 0.2) = dark gray, A=1.0 = fully opaque -->
      <material name="body_color">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
    <!-- Collision properties define the shape used for physics collisions -->
    <!-- Often the same as visual, but can be simpler for performance -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

"""

    # Define leg parameters - this is a Python dictionary
    # FR = Front Right, FL = Front Left, RR = Rear Right, RL = Rear Left
    # hip_pos: [x, y, z] position where the hip joint attaches to the body
    # hip_sign: 1 for right side legs (positive Y), -1 for left side legs (negative Y)
    legs = {
        'FR': {'hip_pos': [0.18, -0.13, 0], 'hip_sign': 1},   # Front Right: forward (+X), right side (-Y)
        'FL': {'hip_pos': [0.18, 0.13, 0], 'hip_sign': -1},   # Front Left: forward (+X), left side (+Y)
        'RR': {'hip_pos': [-0.18, -0.13, 0], 'hip_sign': 1},  # Rear Right: backward (-X), right side (-Y)
        'RL': {'hip_pos': [-0.18, 0.13, 0], 'hip_sign': -1}   # Rear Left: backward (-X), left side (+Y)
    }

    # Segment lengths (in meters) - these define how long each part of the leg is
    hip_length = 0.08      # Distance from hip joint to thigh joint (8 centimeters)
    thigh_length = 0.213   # Length of the upper leg bone (21.3 centimeters)
    calf_length = 0.213    # Length of the lower leg bone (21.3 centimeters)

    # Loop through each leg in the dictionary
    # leg_name will be 'FR', 'FL', 'RR', or 'RL'
    # leg_data will be the dictionary containing hip_pos and hip_sign
    for leg_name, leg_data in legs.items():
        # Extract the x, y, z coordinates from the hip_pos list
        x, y, z = leg_data['hip_pos']
        # Get the sign value (1 or -1) - used to mirror left/right legs
        sign = leg_data['hip_sign']

        # ============================================
        # HIP JOINT AND LINK (abduction/adduction - side to side)
        # ============================================
        # Add XML for this leg's hip joint to our urdf_content string
        # f-string (f"...") allows us to insert Python variables using {variable_name}
        urdf_content += f"""
  <!-- {leg_name} Hip Joint (moves leg sideways) -->
  <!-- This joint connects the body to the hip link -->
  <joint name="{leg_name}_hip_joint" type="revolute">
    <!-- parent link: the body -->
    <parent link="base_link"/>
    <!-- child link: the hip piece of this leg -->
    <child link="{leg_name}_hip"/>
    <!-- origin: position {x}, {y}, {z} in meters, no rotation -->
    <origin xyz="{x} {y} {z}" rpy="0 0 0"/>
    <!-- axis: this joint rotates around the X-axis (1 0 0) = sideways motion -->
    <axis xyz="1 0 0"/>
    <!-- limit: can rotate from -0.8 to +0.8 radians (about -46° to +46°) -->
    <!-- effort: maximum torque in Newton-meters (25 Nm) -->
    <!-- velocity: maximum rotation speed in radians/second (21 rad/s) -->
    <limit lower="-0.8" upper="0.8" effort="25" velocity="21"/>
    <!-- dynamics: damping slows motion, friction resists motion -->
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- The hip link itself - the physical piece that rotates -->
  <link name="{leg_name}_hip">
    <inertial>
      <!-- Center of mass is offset in Y direction by sign * 0.04 meters -->
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <!-- Hip piece weighs 0.8 kg -->
      <mass value="0.8"/>
      <!-- Inertia values for rotation (small piece, so small values) -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <!-- Visually positioned at sign * 0.04 meters in Y direction -->
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <geometry>
        <!-- Small box: 0.06m x {hip_length}m x 0.04m -->
        <box size="0.06 {hip_length} 0.04"/>
      </geometry>
      <!-- Orange color for joint pieces -->
      <material name="joint_color">
        <color rgba="0.8 0.3 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <!-- Collision shape matches visual shape -->
      <origin xyz="0 {sign * 0.04} 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 {hip_length} 0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- {leg_name} Thigh Joint (forward/backward movement) -->
  <!-- This connects the hip to the thigh (upper leg) -->
  <joint name="{leg_name}_thigh_joint" type="revolute">
    <!-- Parent is the hip link we just created -->
    <parent link="{leg_name}_hip"/>
    <!-- Child is the thigh link -->
    <child link="{leg_name}_thigh"/>
    <!-- Position: at the end of the hip piece (sign * hip_length in Y direction) -->
    <origin xyz="0 {sign * hip_length} 0" rpy="0 0 0"/>
    <!-- axis: rotates around Y-axis (0 1 0) = forward/backward motion -->
    <axis xyz="0 1 0"/>
    <!-- limit: -1.5 to +3.0 radians (about -86° to +172°) - allows leg to swing forward a lot -->
    <limit lower="-1.5" upper="3.0" effort="25" velocity="21"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- The thigh link - upper leg bone -->
  <link name="{leg_name}_thigh">
    <inertial>
      <!-- Center of mass halfway down the thigh (negative Z because it hangs down) -->
      <origin xyz="0 0 -{thigh_length/2}" rpy="0 0 0"/>
      <!-- Thigh weighs 1.0 kg (heavier than hip) -->
      <mass value="1.0"/>
      <!-- Higher inertia values because it's longer and heavier -->
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <!-- Cylinder positioned at center, extending down -->
      <origin xyz="0 0 -{thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <!-- Cylinder: radius 2.5cm, length {thigh_length} meters -->
        <cylinder radius="0.025" length="{thigh_length}"/>
      </geometry>
      <!-- Gray color for limbs -->
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
  <!-- This is the knee - connects thigh to calf (lower leg) -->
  <joint name="{leg_name}_calf_joint" type="revolute">
    <!-- Parent is the thigh -->
    <parent link="{leg_name}_thigh"/>
    <!-- Child is the calf (lower leg) -->
    <child link="{leg_name}_calf"/>
    <!-- Position: at the bottom of the thigh (negative Z = down) -->
    <origin xyz="0 0 -{thigh_length}" rpy="0 0 0"/>
    <!-- axis: rotates around Y-axis (0 1 0) = knee bending -->
    <axis xyz="0 1 0"/>
    <!-- limit: -2.7 to -0.8 radians (about -155° to -46°) - knee only bends backwards -->
    <!-- Note: both values are negative because the knee bends backward -->
    <limit lower="-2.7" upper="-0.8" effort="25" velocity="21"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- The calf link - lower leg bone -->
  <link name="{leg_name}_calf">
    <inertial>
      <!-- Center of mass halfway down the calf -->
      <origin xyz="0 0 -{calf_length/2}" rpy="0 0 0"/>
      <!-- Calf weighs 0.2 kg (lighter than thigh) -->
      <mass value="0.2"/>
      <!-- Smaller inertia values because it's lighter -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.00005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -{calf_length/2}" rpy="0 0 0"/>
      <geometry>
        <!-- Slightly thinner cylinder: radius 2cm, length {calf_length} meters -->
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
  <!-- The foot is the part that touches the ground -->
  <link name="{leg_name}_foot">
    <inertial>
      <!-- Center of mass at origin (it's a small sphere) -->
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- Foot weighs only 0.1 kg (very light) -->
      <mass value="0.1"/>
      <!-- Very small inertia (tiny sphere) -->
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!-- Small sphere with 2.5cm radius -->
        <sphere radius="0.025"/>
      </geometry>
      <!-- Almost black color for feet -->
      <material name="foot_color">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <!-- Contact properties: how the foot interacts with ground -->
      <contact>
        <!-- lateral_friction: 1.0 = high friction, prevents slipping -->
        <lateral_friction value="1.0"/>
        <!-- restitution: 0.0 = no bounce, foot doesn't bounce off ground -->
        <restitution value="0.0"/>
      </contact>
    </collision>
  </link>

  <!-- Fixed joint attaching foot to calf - this joint doesn't move -->
  <joint name="{leg_name}_foot_fixed" type="fixed">
    <!-- Parent is the calf -->
    <parent link="{leg_name}_calf"/>
    <!-- Child is the foot -->
    <child link="{leg_name}_foot"/>
    <!-- Position: at the very bottom of the calf -->
    <origin xyz="0 0 -{calf_length}" rpy="0 0 0"/>
  </joint>
"""

    # Close the robot XML tag - this marks the end of the robot definition
    urdf_content += "\n</robot>"

    # Return the complete URDF string we just built
    return urdf_content


def save_urdf():
    """Save the generated URDF to a file"""
    # Get the directory where this Python script is located
    # __file__ is the path to this script
    # os.path.abspath() converts it to an absolute path
    # os.path.dirname() gets the directory part (without the filename)
    urdf_dir = os.path.dirname(os.path.abspath(__file__))

    # Create the full path to the URDF file by joining the directory and filename
    # This will be something like: /path/to/simulation/models/unitree_go2.urdf
    urdf_path = os.path.join(urdf_dir, "unitree_go2.urdf")

    # Call our function to generate the URDF content (returns the XML string)
    urdf_content = generate_go2_urdf()

    # Open a file for writing ('w' mode = write mode)
    # 'with' ensures the file is properly closed after writing
    # 'f' is the file object we can write to
    with open(urdf_path, 'w') as f:
        # Write the URDF content string to the file
        f.write(urdf_content)

    # Print a success message showing where the file was saved
    print(f"✓ URDF file created at: {urdf_path}")

    # Return the path so other code can use it
    return urdf_path


# This special if statement checks if this script is being run directly
# (as opposed to being imported as a module by another script)
if __name__ == "__main__":
    # If run directly, generate and save the URDF file
    save_urdf()
