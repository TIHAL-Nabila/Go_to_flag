# Go_to_flag

## Project Overview
This project involves simulating a multi-robot system in ROS and Gazebo. The goal is to coordinate three robots to move to their respective flags (robot1 → flag1, robot2 → flag2, robot3 → flag3) while avoiding collisions. The simulation demonstrates strategies for robot movement, including obstacle avoidance and timing-based coordination.

### Key Features
- **Robots**: Equipped with ultrasonic sensors (range: 5 meters), motorized wheels, and pose awareness.
- **Environment**: Simulated using Gazebo.
- **Strategies**:
  - Single robot navigation.
  - Multi-robot coordination with collision avoidance.
  - Timing-based strategy for safe multi-robot operation.

---

### Prerequisites
- **ROS1** installed on your system.
- **Gazebo** for simulation visualization.
- Clone the repository:
  ```bash
  git clone https://github.com/username/Mission_Coordination_Project.git
  cd Mission_Coordination_Project
  ```

### Steps to Run the Simulation
1. **Set up the environment**:
   - Navigate to the ROS workspace:
     ```bash
     cd /home/user/catkin_ws/src/Mission_Coordination_Project/
     ```
   - Make the Python script executable:
     ```bash
     chmod +x src/evry_project_strategy/nodes/agent.py
     ```

2. **Run the Simulation**:
   - Launch Gazebo with the robots:
     ```bash
     roslaunch evry_project_description simu_robot.launch
     ```
   - Execute the Python strategy script:
     ```bash
     roslaunch evry_project_strategy agent.launch nbr_robot:=3
     ```

3. **Observe the Simulation**:
   - Open Gazebo to visualize robot movements.
   - Stop the simulation using `CTRL + C`.

---

## Contributors
- **Lecturer**: Sofiane Ahmed-Ali
- **Teacher Assistant**: Boris KIEMA
- **Group Members**: Nabila TIHAL/ Chahinaz HAMIDECHE.

---
