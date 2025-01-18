# Base image
FROM ros:humble-ros-core

# Update package lists and install necessary tools
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install -U pip && \
    pip3 install -U setuptools && \
    rm -rf /var/lib/apt/lists/*

# Copy the python requirements file
COPY ./requirements.txt /workspace/requirements.txt

# Copy the built artifacts into the container
COPY ./install /workspace/install

# Copy the server file
COPY ./server.py /workspace/mission_server.py

# Set the working directory
WORKDIR /workspace

# Install the python dependencies
RUN python3 -m pip install -r requirements.txt

# install ros2 websocket server
RUN apt-get update && \
    apt-get install -y ros-humble-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

# Create a script to run the server nodes
RUN echo '#!/bin/bash' > /start.sh && \
    echo 'set -e' >> /start.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start.sh && \
    echo 'source /workspace/install/setup.bash' >> /start.sh && \
    echo 'python3 /workspace/mission_server.py & ' >> /start.sh && \
    echo 'ros2 launch mission_processor mission_action_server.launch.py & ' >> /start.sh && \
    echo 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' >> /start.sh && \
    chmod +x /start.sh

# Expose the port your Flask server and websocket server is running on
EXPOSE 5000 9090

# Run the start script
CMD ["/bin/bash", "/start.sh"]