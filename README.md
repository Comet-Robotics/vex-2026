# vex-2026
Repo containing all of our code for the 2025-2026 VEX game Push Back. This includes all VEX brain code and jetson code.


## Installation

Follow these steps to set up your development environment and ensure VS Code IntelliSense can find the ROS Humble headers.

### 1. Install ROS Humble

If you haven't already, install ROS Humble following the official instructions:

[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

By default, ROS will install headers to `/opt/ros/humble/include`.

---

### 2. Add ROS include paths for VS Code IntelliSense

VS Code uses `c_cpp_properties.json` to configure include paths. To avoid committing machine-specific paths, you can set this up locally.

1. Open your project in VS Code.
2. Go to `File` → `Preferences` → `Settings` → `Extensions` → `C/C++` → `Edit Configurations (JSON)`.

3. Add `/opt/ros/humble/include/**` to your `includePath`. Example:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/**"
            ],
            "defines": [],
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "compilerPath": "/usr/bin/g++-12",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
