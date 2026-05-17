# Manual Controller

ROS/C++ manual control package for a robot system.

## Contents

- `src/` - C++ source files.
- `include/` - headers.
- `config/` - configuration files.
- `launch/` - ROS launch files.
- `msg/` - custom ROS message definitions.
- `package.xml` - ROS package metadata.
- `CMakeLists.txt` - CMake/catkin build configuration.
- `rosconsole.conf` - ROS console logging configuration.

## Build

Place this package in a ROS workspace and build with the workspace tooling used by the project, for example:

```bash
catkin_make
```

or the equivalent command for your configured ROS environment.

## Notes

Before running against hardware, verify joystick/control mappings, topic names, robot limits, and emergency-stop behavior.