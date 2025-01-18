# ROS2 Mission Control System

## Introduction
This repository demonstrates how to integrate a Python REST API with ROS2 nodes for mission control operations(an example of ROS2 Action Server). It consists of a **REST API** for managing mission data and two ROS2 nodes: a client node for fetching and processing mission data, and a server node for executing missions.

It also provides an example of accessing the ROS2 Action using a **Websocket server** to get the data outside the ROS2 environment.

Overall, the project is containerized and made easy to deploy using Docker.

## Setup Instructions
### Prerequisites
Make sure you have **ROS2** and **Docker** installed on your system.
### Clone the Repository
1. Clone the Repository:
   ```bash
   git clone https://github.com/harishkumarbalaji/ros2-custom-action.git
   cd ros2-custom-action
   ```

2. Install ROS Dependencies:
   ```bash
   rosdep update
   rosdep install -y --from-path src --ignore-src
   ```

3. Build the Docker image (Replace `ROS_DISTRO` with your ROS2 distribution) in `build.sh`:
   ```bash
   bash build.sh
   ```
It will build the ROS2 workspace and create a Docker image with humble-ros-core as the base image and install the required dependencies.

## Run Instructions

### Run the Servers( Rest API, ROS2 Action Server, Websocket server)
In one terminal, bring up all the servers by running,
   ```bash
   bash run_server.sh
   ```

The REST API server will run on port `5000` and the websocket server will run on port `9090`.

### Run the Action Client
Once the servers are running, you can run the action client using in another terminal by running,
   ```bash
   bash run_client.sh
   ```

Once the action client is running, you can see the `/get_action` action is being called and you can see the feedback being published in the server terminal.

you can send a mission using the POST `/mission` endpoint and retrieve the result using the GET `/mission` endpoint.

Once the new mission is sent, the ROS2 Action Server will process the mission data and the client will receive the the updated mission data.

## API Interaction

### REST
1. Update the mission data using the POST `/mission` endpoint.
   ```bash
   curl -X POST -H "Content-Type: application/json" -d '{"id": 0, "description": "Go to Rack 9"}' http://127.0.0.1:5000/update_mission

   ```
2. Retrieve the latest mission data using the GET `/mission` endpoint.
   
   ```bash
   curl -X GET http://127.0.0.1:5000/get_mission
   ```
### Websocket
The docker entry point script includes running the ros websocket server which is this awesome ros package [rosbridge_suite](https://wiki.ros.org/rosbridge_suite).

You can use the following commands to connect to the websocket server and trigger the action in ros2 environment and receive the feedback.

> Note: You can use any websocket client to connect to the server. For example, we are going to use `websocat` which is a cargo package.
> Install Rust and Cargo using the instructions [here](https://www.rust-lang.org/tools/install). or simple run `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

Install the `websocat` application using,
```bash
cargo install websocat
```
Once the `websocat` is installed, connect to the websocker server using,

```bash
 websocat -t ws://127.0.0.1:9090
```

Now, once the client is connected(you can check the client connection logs in the server terminal), you can send the action using the following command. (just copy and paste the json below)

```
{ "op": "send_action_goal",
  "action": "get_mission",
  "action_type": "mission_action_interfaces/action/Mission",
 "feedback": true
}
```

You can see the feedback being published in the server terminal and also it is being received through the websocket server.

There are more features available in the websocket server like subscribing to topics, sending messages, etc. You can find the documentation [here](https://github.com/RobotWebTools/rosbridge_suite).

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

## Extra Functionality
- **Custom ROS Messages:** Defined a custom action message for mission data.
- **Real-time Data Processing:** Mission Client Node polls the API every second for real-time data.
- **Action Feedback:** Mission Server Node provides feedback during mission execution.
- **Websocket Server:** Runs a ROS2 WebSocket Server to receive and send actions in a ROS2 environment.