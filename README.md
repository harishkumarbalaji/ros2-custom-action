# ROS2 Mission Control System

## Introduction
This repository demonstrates how to integrate a Python REST API with ROS2 nodes for mission control operations. It consists of a REST API for managing mission data and two ROS2 nodes: a client node for fetching and processing mission data, and a server node for executing missions.

It also provides an example of accessing the ROS2 Action Server using a Websocket server to get the data outside the ROS2 environment.

Overall, the project is containerized and made easy to deploy using Docker.

## Setup Instructions

### Clone the Repository
1. Clone the Repository:
   ```bash
   git clone https://github.com/yourusername/mission_control_system.git
   cd mission_control_system
   ```

2. Install Dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set Up ROS2 Workspace:
   ```bash
   source /opt/ros/foxy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Run Instructions

### Running the REST API
1. Navigate to the API Directory:
   ```bash
   cd api
   ```

2. Start the Flask Server:
   ```bash
   python app.py
   ```
   The API will be running on [http://localhost:5000](http://localhost:5000).

### Running the ROS2 Nodes
1. Open a New Terminal:
   ```bash
   source install/setup.bash
   ```

2. Run the Mission Client Node:
   ```bash
   ros2 run your_package_name mission_client_node
   ```

3. Run the Mission Server Node:
   ```bash
   ros2 run your_package_name mission_server_node
   ```

## System Overview

### REST API
- **POST /mission:** Accepts JSON data for a new mission.
- **GET /mission:** Retrieves the latest mission data in JSON format.

### ROS2 Nodes

#### Mission Client Node
- **Functionality:** 
  - Polls the GET /mission endpoint every second for new mission data.
  - Processes the JSON data into a custom ROS action.
  - Sends the action to the Mission Server Node using an Action Client.

#### Mission Server Node
- **Functionality:** 
  - Hosts an Action Server that listens for incoming actions.
  - Executes the received action and prints the mission details.

## Custom Messages
The custom action message is defined as follows:

```
# Request
string description
---
# Result
string result
---
# Feedback
int32 countdown
```

## Deployment with Docker
To deploy the system using Docker, follow these steps:

1. Build the Docker Image:
   ```bash
   docker build -t mission_control_system .
   ```

2. Run the Docker Container:
   ```bash
   docker run -it --name mission_control -p 5000:5000 mission_control_system
   ```

## Extra Functionality
- **Custom ROS Messages:** Defined a custom action message for mission data.
- **Real-time Data Processing:** Mission Client Node polls the API every second for real-time data.
- **Action Feedback:** Mission Server Node provides feedback during mission execution.

## Contributing
We welcome contributions! If you find issues or have suggestions, please open an issue or submit a pull request.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

