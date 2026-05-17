# Manual Controller

ROS/C++ manual control package for a robot system.

## What This Repository Contains

- `src/` - C++ source files.
- `include/` - headers.
- `config/` - configuration files.
- `launch/` - ROS launch files.
- `msg/` - custom ROS message definitions.
- `package.xml` - ROS package metadata.
- `CMakeLists.txt` - CMake/catkin build configuration.
- `rosconsole.conf` - ROS console logging configuration.

## Build

Place this package inside a ROS workspace and build with the workspace tooling used by the project, for example:

```bash
catkin_make
source devel/setup.bash
```

## Safety

Before running against hardware, verify joystick/control mappings, topic names, robot limits, and emergency-stop behavior. Test command output without actuators enabled first.

## Notes

Document the ROS distribution, required messages, launch procedure, and connected hardware as the package is cleaned up.