# Underwater\_Bot — Simulation & Launch Instructions

This repository contains the **Gazebo 8.9.0** + **ROS 2 Jazzy Jalisco** simulation for the underwater bot.

---

##  Build & Launch Instructions

Follow these steps to build and launch the simulation.

### 1️⃣ Clean, Build & Source

```bash
rm -rf ~/.build ~/.log ~/.install
colcon build
source install/setup.bash
```

### 2️⃣ Launch the Simulation

```bash
ros2 launch simple_auv launch_it.py
```

### 3️⃣ Run Yolobot Recognition

```bash
ros2 run yolobot_recognition yolov8_ros2_pt.py
```

---

## ⚠️ Thrust Commanding Issue

If you face an issue where thrust commanding does not work, check the following:

* Ensure that the **`gazebo_ros2_control`** plugin is loaded in the URDF.
* Verify that all thruster joints have **effort interfaces** and **transmissions** defined.
* Spawn the **`joint_state_broadcaster`** and **effort controllers** after launching Gazebo.
* Confirm that controller names in your YAML match the URDF joint names.

> 💡 **Alternative Approach:** You *can* use the **`diff_drive_controller`** and model your thrusters as wheels. However, this is only recommended if you want simple forward/backward motion and yaw control. For realistic underwater 6-DOF control, stick to thruster effort controllers.

---

## 🧰 Notes for Contributors

When modifying URDFs, plugins, or controller setups:

* Use `effort` interfaces for thruster joints.
* Add `gazebo_ros2_control` plugin to the URDF.
* Keep controller YAML and joint names consistent.
* Debug using:

  ```bash
  ros2 topic echo /joint_states
  ros2 control list_controllers
  ```

---

## 🤝 Contributing

If you know how to fix the Gazebo thruster plugin issue, contributions are welcome! Please submit a Pull Request or open an Issue in this repository.
