# SMARTmBOT Description Package

This package contains the URDF and SDF model files for the SMARTmBOT robot.

## Package Contents

- `urdf/smartmbot.urdf.xacro` - URDF robot description with Gazebo plugins
- `models/smartmbot/` - SDF model files for Gazebo simulation
- `meshes/` - Robot mesh files (currently using primitive shapes)

## Robot Specifications

### Physical Dimensions
- **Base**: 42cm x 31cm x 18cm rectangular chassis
- **Wheels**: 10cm radius, 4cm width differential drive wheels  
- **Caster**: 5cm radius front caster wheel
- **Lidar**: 5cm radius x 10cm height sensor

### Sensors
- **Lidar**: 360-degree laser scanner
  - Range: 0.1m to 30m
  - Resolution: 1cm
  - Update rate: 10 Hz
  - 360 samples per scan

### Actuators  
- **Differential Drive**: Two motorized wheels
- **Wheel Separation**: 36cm
- **Wheel Radius**: 10cm

### Gazebo Plugins
- **Differential Drive System**: Provides `/cmd_vel` topic control
- **Joint State Publisher**: Publishes joint states
- **Lidar Sensor**: GPU-accelerated laser scanner

## Usage

This package is typically used as a dependency by other SMARTmBOT packages.

### View URDF in RViz
```bash
ros2 launch robot_state_publisher view_robot.launch.py model:=$(ros2 pkg prefix SMARTmBOT_description)/share/SMARTmBOT_description/urdf/smartmbot.urdf.xacro
```

### Use in Your Own Launch Files
```python
from ament_index_python.packages import get_package_share_directory

pkg_description = get_package_share_directory('SMARTmBOT_description')
urdf_file = os.path.join(pkg_description, 'urdf', 'smartmbot.urdf.xacro')
```

## Model Files

### URDF (smartmbot.urdf.xacro)
- Complete robot description with materials, inertial properties
- Gazebo plugin configuration
- Joint and link definitions
- Sensor specifications

### SDF (models/smartmbot/model.sdf)  
- Native Gazebo model format
- Optimized for Gazebo simulation
- Includes visual, collision, and inertial properties
- Sensor and plugin definitions