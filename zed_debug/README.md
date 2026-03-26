# ZED ROS2 Wrapper Debugging

## Build with Debug Symbols

Go to the root of your ROS2 workspace.

Clean the previous build if they do not contain debug symbols:

```bash
rm -rf install
rm -rf build
rm -rf logs
```

Build the ZED ROS2 wrapper with debug symbols:

```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug --parallel-workers $(nproc)
```

or use `RelWithDebInfo` build type for optimized builds with debug symbols (normally required by the ZED SDK):

```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers $(nproc)
```

## Debug with VSCode

It's possible to debug the ZED ROS2 nodes using VSCode and the `ros2 launch` command with a `gdbserver` prefix.

### Setup VSCode launch configuration

1) Open VSCode on your workspace.
2) Go to your side bar, 'Run and Debug' section.
3) Add a new configuration by clicking on 'create a launch.json file'.
4) Select 'C++ (GDB/LLDB)'.
5) Replace the content of the generated `launch.json` file with the following:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Debugger",
            "request": "launch",
            "type": "cppdbg",
            "miDebuggerServerAddress": "localhost:3000",
            "cwd": "${workspaceFolder}",
            "program": "install/zed_debug/lib/zed_debug/zed_debug_proc",
            "stopAtEntry": true
        }
    ]
}
```

with `"stopAtEntry": true` the node will stop at the beginning of the `main` function. This allows you to set breakpoints before the execution continues.

Learn more about VSCode debugging configuration:

* [Debug C++ in Visual Studio Code](https://code.visualstudio.com/docs/cpp/cpp-debug)
* [Configure C/C++ debugging](https://code.visualstudio.com/docs/cpp/launch-json-reference)

### Start the node with gdbserver

The `zed_camera_debug.launch.py` launch file has been modified to accept a `cmd_prefix` argument that allows you to add a prefix to the node executable.

Run the launch file with the `gdbserver` prefix:

```bash
ros2 launch zed_debug zed_camera_debug.launch.py camera_model:=<your_camera_model> cmd_prefix:='gdbserver localhost:3000'
```

## Debug with `GDB`

You can also debug the ZED ROS2 nodes using `gdb` directly from the command line.

```bash
ros2 launch zed_debug zed_camera_debug.launch.py camera_model:=<your_camera_model> cmd_prefix:='gdb --args'
```

You can modify `gdb --args` to add any other `gdb` options you may need.

## Debug with `Valgrind`

You can also run the ZED ROS2 nodes with `valgrind` to check for memory leaks and other memory-related issues.

Run the launch file with the `valgrind` prefix:

```bash
ros2 launch zed_debug zed_camera_debug.launch.py camera_model:=<your_camera_model> cmd_prefix:='valgrind --leak-check=full --track-origins=yes'
```

You can modify `valgrind --leak-check=full --track-origins=yes` to add any other `valgrind` options you may need.

## Known Issues

When the `isaac_ros_managed_nitros` package is installed and the ZED Camera Components are linked against Isaac ROS libraries, you may experience issues when trying to start the `zed_debug` executable. This appears to be an issue with the Isaac ROS libraries when used with static ROS 2 Composition instead of dynamic composition (using launch files). We are working with NVIDIA to resolve this issue. In the meantime, if you encounter this problem and want to debug the ZED Components, please uninstall the `isaac_ros_managed_nitros` package from your system and rebuild.
