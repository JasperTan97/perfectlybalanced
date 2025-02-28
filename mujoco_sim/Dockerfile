# Use an Ubuntu base image
FROM ubuntu:22.04

# Install required dependencies
RUN apt update && apt install -y \
    cmake \
    g++ \
    gdb \
    git \
    libgl1-mesa-dev \
    libglew-dev \
    libglfw3 \
    libglfw3-dev \ 
    libx11-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    x11-xserver-utils \
    x11-apps \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /mujoco

# Download MuJoCo
RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-linux-x86_64.tar.gz && \
    tar -xvf mujoco-3.3.0-linux-x86_64.tar.gz  && \
    rm mujoco-3.3.0-linux-x86_64.tar.gz

# Set environment variables
ENV MUJOCO_PATH=/mujoco/mujoco-3.3.0
ENV LD_LIBRARY_PATH=$MUJOCO_PATH/bin:$LD_LIBRARY_PATH
ENV PATH=$MUJOCO_PATH/bin:$PATH

# Copy project files (this will be mapped using a volume)
COPY src /mujoco/src

WORKDIR /mujoco/src

CMD ["/bin/bash"]
