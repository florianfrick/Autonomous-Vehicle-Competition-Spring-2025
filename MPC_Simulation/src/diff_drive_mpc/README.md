# diff_drive_mpc

This package implements a differential drive robot simulator and a linear MPC controller in ROS 2 using Python. It includes real-time path following of a square reference trajectory and visualization via RViz2.

---

## 🛠 Prerequisites

- ROS 2 (tested with Humble/Foxy)

Install dependencies:
```bash
pip install cvxpy numpy scipy
```

---

## 📁 Project Structure

```
diff_drive_mpc/
├── diff_drive_mpc/
│   ├── mpc_controller.py     # Linear MPC node
│   ├── simulator.py          # Unicycle model simulation node
│   ├── utils.py              # Helper functions (dynamics, discretization)
│   └── __init__.py
├── launch/
│   └── sim_launch.py         # Launches simulator + controller
├── resource/
│   └── diff_drive_mpc        # Required by ament
├── setup.py                  # Package declaration
├── setup.cfg                 # Ensures correct install paths
└── package.xml               # ROS2 package manifest
```

---

## 🚀 Building the Package

From your ROS2 workspace root:

```bash
colcon build --packages-select diff_drive_mpc --symlink-install
source install/setup.bash
```

> Using `--symlink-install` means you **do not need to rebuild** after editing Python code.

---

## ▶️ Running the Simulation

```bash
ros2 launch diff_drive_mpc sim_launch.py
```

This starts:
- `simulator`: simulates robot state using a unicycle model
- `mpc_controller`: solves linear MPC to follow a square trajectory

---

## 🖼 Visualizing in RViz2

```bash
rviz2
```

- Set **Fixed Frame**: `odom`
- Add the following displays:
  - `Path` on topic `/ref_path`
  - `Odometry` on topic `/odom`

You’ll see:
- Square reference trajectory
- Real-time robot motion from the simulator

---

## 🔁 Development Tips

- Edit Python files freely
- No rebuild needed if using `--symlink-install`
- If problems persist:
  ```bash
  rm -rf build/ install/ log/
  colcon build --symlink-install
  source install/setup.bash
  ```

---

