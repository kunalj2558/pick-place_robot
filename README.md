# 🤖 Synora: 6-DOF Pick-and-Place Robotic Arm Simulation

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo Sim](https://img.shields.io/badge/Simulator-Gazebo_Harmonic_(gz--sim_8)-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![ros2_control](https://img.shields.io/badge/Control-ros2__control-brightgreen)](https://control.ros.org/jazzy/index.html)
[![Language](https://img.shields.io/badge/Language-Python_3.12-yellow?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey.svg)](LICENSE)

An industrial-grade autonomous pick-and-place simulation featuring a **6-DOF articulated cobot with a two-finger parallel gripper**, developed from scratch on **ROS 2 Jazzy** and **Gazebo Harmonic**.

The system performs a complete automated warehouse workflow: identifying a dynamic vitamin bottle inside an open storage tote, planning sub-millimeter trajectory waypoints, securely clamping it with tuned contact physics, and gently placing it into a shipping carton before returning home.

---

## 📖 Story & Motivation

Most introductory robotics simulations stop at moving an arm through empty space. In real-world automation, however, the hardest challenges happen at the contact boundary:

- **Collision Boundaries:** A container cannot be a single solid block in collision geometry, or gripper fingers will collide with the rim before grasping the object.
- **Physical Grasping:** Lifting a dynamic object purely with physical friction requires precise normal force, realistic surface contact parameters ($K_p$, $K_d$, and $\mu$), and proper controller tolerances.
- **Kinematic Alignment:** Reaching into containers demands strict downward tool orientation ($\vec{z} = [0, 0, -1]$) while compensating for transverse joint offsets.

I built this project to learn and master the complete modern ROS 2 robotics stack — from mathematical modeling in URDF/Xacro to `ros2_control` hardware abstraction, Gazebo Harmonic integration, and multi-threaded Python action client orchestration.

---

## 🦾 Robot Anatomy & Specifications

The robot arm (inspired by Universal Robots UR cobots and Robotiq 2F-85 grippers) consists of 6 revolute arm joints and 2 prismatic finger joints:

| Joint Name | Type | Axis | Range / Limits | Role |
| :--- | :--- | :--- | :--- | :--- |
| `shoulder_pan_joint` | Revolute | `[0, 0, 1]` (Yaw) | $\pm 180^\circ$ ($\pm 3.142$ rad) | Base turret rotation |
| `shoulder_lift_joint` | Revolute | `[0, 1, 0]` (Pitch) | $\pm 120^\circ$ ($\pm 2.094$ rad) | Main arm boom pitch |
| `elbow_joint` | Revolute | `[0, 1, 0]` (Pitch) | $\pm 150^\circ$ ($\pm 2.618$ rad) | Forearm extension / flexion |
| `wrist_1_joint` | Revolute | `[0, 1, 0]` (Pitch) | $\pm 180^\circ$ ($\pm 3.142$ rad) | Wrist elevation & tilt |
| `wrist_2_joint` | Revolute | `[0, 0, 1]` (Yaw) | $\pm 180^\circ$ ($\pm 3.142$ rad) | Wrist lateral roll |
| `wrist_3_joint` | Revolute | `[0, 0, 1]` (Spin) | $\pm 180^\circ$ ($\pm 3.142$ rad) | Tool flange continuous spin |
| `left_finger_joint` | Prismatic | `[0, 1, 0]` | $0$ to $35$ mm | Symmetrical finger sliding |
| `right_finger_joint` | Prismatic | `[0, -1, 0]` | $0$ to $35$ mm | Symmetrical finger sliding |

- **Reach:** $\approx 0.85$ m
- **Payload Capacity:** Lightweight model optimized for consumer laptops ($\approx 8$ GB RAM).
- **Gripper Opening:** $16$ mm (closed) to $86$ mm (fully open).

---

## 🎯 Pick-and-Place Sequence

```text
 [HOME] ────> [APPROACH CRATE] ────> [DESCEND] ────> [CLAMP BOTTLE]
   ▲                                                       │
   │                                                       ▼
 [HOME] <──── [RETRACT BOX] <──── [PLACE & RELEASE] <──── [LIFT & TRANSFER]
```

1. **Open Gripper:** Fingers open symmetrically to $35$ mm each (total gap: $86$ mm).
2. **Approach Crate:** Arm moves to pre-grasp pose $(0.42, 0.15, 0.45)$ m with tool pointing straight down.
3. **Descend Around Bottle:** Gripper enters the hollow crate, lowering pads around the bottle body ($z = 0.295$ m) with tips $5$ mm above crate floor.
4. **Firm Clamping:** Fingers close with $40$ N continuous effort; settling pause allows friction force to stabilize.
5. **Lift:** Bottle is lifted vertically out of the crate ($z = 0.50$ m).
6. **Transfer:** Cobot articulates across the workstation table above the cardboard box $(0.45, -0.18, 0.50)$ m.
7. **Lower & Place:** Bottle descends inside the box ($z = 0.315$ m) and rests gently on the carton floor.
8. **Release & Retract:** Gripper opens fully, clears the box vertically, and smoothly returns to the upright `Home` pose.

---

## 🏗️ System Architecture

```text
┌────────────────────────────────────────────────────────┐
│                   ROS 2 Jazzy Node                     │
│         (synora_arm_controller - Python CLI)           │
└───────────▲───────────────────────────────┬────────────┘
            │ /joint_states                 │ Action Goals
            │ (sensor_msgs/JointState)      │ (control_msgs/FollowJointTrajectory)
┌───────────┴───────────────────────────────▼────────────┐
│                    ros2_control                        │
│ ┌──────────────────────┐   ┌─────────────────────────┐ │
│ │  arm_controller      │   │  gripper_controller     │ │
│ │(JointTrajectoryCtrl) │   │ (JointTrajectoryCtrl)   │ │
│ └──────────┬───────────┘   └─────────────┬───────────┘ │
│            │                             │             │
│            ▼                             ▼             │
│        gz_ros2_control (GazeboSimSystem Plugin)        │
└──────────────────────────┬─────────────────────────────┘
                           │ Joint Commands & States
┌──────────────────────────▼─────────────────────────────┐
│                 Gazebo Harmonic (gz-sim 8)             │
│  • DART Physics (1ms step size)                        │
│  • Contact Surface Mechanics (Kp=1e6, mu=5.0)          │
│  • Dynamic Pickable Bottle (m=0.08 kg)                 │
└────────────────────────────────────────────────────────┘
```

---

## 🚀 Getting Started

### Prerequisites

- **OS:** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2:** Jazzy Jalisco (Desktop Install)
- **Simulator:** Gazebo Harmonic (`gz-sim 8`)
- **Key Packages:**
  ```bash
  sudo apt-get update
  sudo apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro
  ```

### Installation & Build

1. **Clone the repository into your ROS 2 workspace `src/` folder:**
   ```bash
   mkdir -p ~/Desktop/ros2_ws/src
   cd ~/Desktop/ros2_ws/src
   git clone https://github.com/kunalj2558/pick-place_robot.git synora_robot_arm
   ```

2. **Build the package:**
   ```bash
   cd ~/Desktop/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select synora_robot_arm
   ```

3. **Source the workspace:**
   ```bash
   source ~/Desktop/ros2_ws/install/setup.bash
   ```

---

## 🎮 Running the Simulation

### 1. Launch Gazebo Simulation & Controllers

In your first terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ros2_ws/install/setup.bash
ros2 launch synora_robot_arm simulation.launch.py
```
*(If Gazebo loads in a paused state, click the orange **Play** button in the lower-left corner.)*

To launch with RViz 2 visualizer enabled simultaneously:
```bash
ros2 launch synora_robot_arm simulation.launch.py rviz:=true
```

### 2. Interactive Terminal Controller

In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ros2_ws/install/setup.bash
ros2 run synora_robot_arm arm_controller
```

You will enter the interactive CLI:
```text
=============================================================
           Synora 6-DOF Robot Arm Interactive CLI
=============================================================
synora_arm> pick
```

#### Available CLI Commands:

| Command | Description |
| :--- | :--- |
| `pick` / `demo` | Executes the complete autonomous pick-and-place sequence |
| `ready` | Moves the cobot into an articulated forward-reaching ready pose |
| `home` | Moves all joints straight up to zero reference pose `[0, 0, 0, 0, 0, 0]` |
| `move <j1> ... <j6>` | Commands individual joint angles in radians (e.g. `move 0.3 0.2 1.1 1.8 0 0`) |
| `open` | Opens gripper fully ($35$ mm per finger) |
| `close` | Closes gripper completely |
| `grip <pos_m>` | Sets gripper opening in metres (e.g. `grip 0.015`) |
| `status` | Prints live joint telemetry, positions, velocities, and finger gap |
| `help` | Displays available commands and joint limits |
| `quit` | Cleanly exits the controller |

---

## 📁 Repository Layout

```text
synora_robot_arm/
├── config/
│   ├── robot_controllers.yaml    # Controller Manager & JointTrajectoryController settings
│   └── robot.rviz                # Custom RViz2 camera and display configuration
├── launch/
│   ├── simulation.launch.py      # Main simulation launch orchestrator with controller sequencing
│   └── rviz.launch.py            # Standalone RViz2 launch file
├── resource/
│   └── synora_robot_arm          # Ament resource marker
├── synora_robot_arm/
│   ├── __init__.py
│   ├── arm_controller.py         # Multi-threaded interactive CLI node with inverse kinematics
│   ├── gripper_controller.py     # Standalone gripper action client utility
│   └── joint_state_monitor.py    # Real-time /joint_states console telemetry monitor
├── urdf/
│   ├── robot_arm.urdf.xacro      # 6-DOF cobot links, joints, collision, and visual macros
│   └── ros2_control.xacro        # gz_ros2_control hardware interfaces and plugins
├── worlds/
│   └── robot_arm.world.sdf       # Workstation, hollow tote, cardboard carton, and bottle models
├── package.xml                   # ROS 2 package manifest
├── setup.cfg
├── setup.py                      # Python packaging and entry point definitions
└── README.md
```

---

## 🔬 Engineering Deep Dive

### 1. Hollow Collision Containers
In simulation engines like Gazebo Sim (DART), if a storage tote is modeled using a single solid bounding box, any descent of the gripper fingers into the tote triggers unintended collision reactions. The crate and carton in this package are modeled with individual floor plates and thin perimeter walls ($1$ cm), leaving a fully hollow interior for realistic container-picking operations.

### 2. High-Friction Dynamic Grasping
Parallel grippers rely on normal contact force and Coulomb friction to lift objects. The finger pads and bottle feature:
- Surface Contact Stiffness: $K_p = 10^6\,\text{N/m}$
- Minimum Contact Depth: $0.002\,\text{m}$
- Static & Dynamic Friction: $\mu = 5.0$ / $\mu = 10.0$
- Controlled Effort: $40\,\text{N}$ per prismatic finger

This allows the lightweight dynamic bottle ($0.08\,\text{kg}$) to be held securely during multi-axis trajectory transfers without slip.

---

## 🤝 Contributing & License

Contributions, improvements, and feedback are welcome!
Feel free to open an issue or submit a pull request.

Distributed under the **Apache-2.0 License**. See `package.xml` for details.

---

**Developed with ❤️ by [Kunal Jadhav](https://github.com/kunalj2558)**  
*Robotics Engineer | ROS 2 Enthusiast*
