# Repository Guidelines

## Project Structure & Module Organization
This workspace targets leader–follower teleoperation for a Doosan M1013 follower. Core robot infrastructure lives under `src/doosan-robot2/` (bringup, controllers, MoveIt, Gazebo assets, hardware/emulator utilities). The teleoperation node and helper scripts live in `src/teleop_doosan/teleop_doosan/`, with Python tests under `src/teleop_doosan/test/`. Launch and configuration files stay package-local (`launch/`, `config/`), while C++ headers sit in each package’s `include/` directory.

## Teleoperation Pipeline Overview
The teleop node subscribes to `trajectory_msgs/JointTrajectory` on `/leader/joint_trajectory`, reorders `joint1`–`joint6` as needed, ignores `rh_r1_joint`, and streams positions to the M1013 via Doosan control interfaces. Commands are applied immediately (no timing interpolation) and the node is tuned for 100–1000 Hz in Gazebo or on hardware, so keep executor and QoS settings lean. When expanding to other robots, document joint remaps alongside the launch file.

## Build, Test, and Development Commands
Run `colcon build --symlink-install --packages-up-to teleop_doosan dsr_bringup2` to compile the teleop stack plus required robot dependencies. Use `colcon test --event-handlers console_direct+ --packages-select dsr_tests teleop_doosan` for emulator-driven system checks and Python linters. `bash src/doosan-robot2/test.sh` mirrors the CI test selection. After building, source `install/setup.bash` and launch `ros2 run teleop_doosan sample_joint_rt` to sanity-check topic wiring.

## Coding Style & Naming Conventions
Python uses 4-space indentation, `snake_case` modules/functions, and module docstrings to satisfy `ament_pep257`. C++ additions follow ROS 2 style: `CamelCase` classes, `snake_case` methods, `ALL_CAPS` constants, and headers free of private macros. Enable `ament_lint_auto` and run `ament_clang_format` when touching C++ code.

## Testing Guidelines
Install the emulator once with `bash src/doosan-robot2/install_emulator.sh`, then ensure `docker start emulator` exposes port `12345` before running high-rate tests. Place new pytest files in `test/` using `test_<feature>.py` naming, leverage `pytest.mark.rostest`, and keep Gazebo simulations deterministic. For joint mirroring scenarios, record leader input bags to replay against regression tests.

## Commit & Pull Request Guidelines
Follow existing prefixes (`feat`, `fix`, `chore`, etc.), keep subjects ≤72 characters, and reference issues with `(#123)`. Target the `humble` branch, call out emulator or hardware prerequisites, and attach logs or GIFs demonstrating leader–follower tracking. PR descriptions should list affected packages and confirm `colcon build` plus `colcon test` runs.

## Emulator & Environment Notes
Default launch files assume namespace `dsr01`, model `m1013`, and Gazebo simulation. If you change rates, QoS policies, or joint mappings for hardware deployments, update the corresponding launch parameters and note the requirements in `AGENTS.md` or the package README.
