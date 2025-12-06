# vex-2026
Repo containing all of our code for the 2025-2026 VEX game Push Back. This includes all VEX brain code and jetson code.


## Installation

Follow these steps to set up your development environment and ensure VS Code IntelliSense can find ROS Humble headers.

### 1. Install ROS Humble

If you haven't already, install ROS Humble following the official instructions:

[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

By default, ROS installs headers to `/opt/ros/humble/include`.

---

### 2. Add ROS include paths using the VS Code UI

1. Open your project in VS Code.
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS) to open the **Command Palette**.
3. Type `C/C++: Edit Configurations (UI)` and select it.
4. In the **Include path** section:
   - Click **Add Include Path**.
   - Enter `/opt/ros/humble/include/**`.
5. Make sure `${workspaceFolder}/**` and `/usr/include/**` are also listed, so your project headers and system headers are included.
6. Click **OK** to save.

> ⚠️ Note: These changes are saved locally in `.vscode/c_cpp_properties.json`, so they will **not** be pushed to GitHub.

---

### 3. Verify

1. Open a C++ file that uses ROS headers, for example:

```cpp
#include <rclcpp/rclcpp.hpp>

