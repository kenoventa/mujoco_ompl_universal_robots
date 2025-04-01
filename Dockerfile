# Copy the OMPL install script into the container
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Base system dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    lsb-release \
    curl \
    git \
    wget \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    python3-distutils \
    python3-setuptools \
    libeigen3-dev \
    libboost-all-dev \
    libode-dev \
    libflann-dev \
    libyaml-cpp-dev \
    castxml \
    && rm -rf /var/lib/apt/lists/*

# Ensure Python 3.10 is the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# Python modules for OMPL bindings
RUN pip3 install --no-cache-dir \
    pygccxml \
    pyplusplus

# Clone OMPL
RUN git clone https://github.com/ompl/ompl.git /opt/ompl

# Build OMPL
WORKDIR /opt/ompl
RUN mkdir -p build/Release
WORKDIR /opt/ompl/build/Release

# Configure and build with Python bindings
RUN cmake ../..
RUN make -j 20 update_bindings
RUN make -j 20
RUN make install


#### MUJOCO ####
RUN apt-get update && apt-get install -y \
    libglfw3-dev \
    libosmesa6-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libpython3-dev \
    unzip \
    libxi-dev \
    libxmu-dev \
    libxinerama-dev \
    libxcursor-dev \ 
    && rm -rf /var/lib/apt/lists/*


# Clone Mujoco
RUN git clone https://github.com/deepmind/mujoco.git /opt/mujoco && \
    mkdir -p /opt/mujoco/build && \
    cd /opt/mujoco/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build . --parallel $(nproc) && \
    cmake --install .

# 4. Install Python packages
RUN apt-get update && pip install \
    mujoco \
    numpy \
    ur-rtde \
    glfw>=2.5 \
    && rm -rf /var/lib/apt/lists/*   


# 5. Add UR5e MJCF model (replace with your actual path)
COPY ur5e.xml /root/.mujoco/ur5e.xml

# 6. Verify installation
RUN python3 -c "import mujoco; print(f'MuJoCo {mujoco.__version__} loaded successfully')"

WORKDIR /opt/mujoco

# FROM python:3.10

# RUN apt update \
#  && pip install  \
#     ur-rtde \
#  && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y \
#     software-properties-common \   
#     wget \
#     && rm -rf /var/lib/apt/lists/*

# RUN add-apt-repository ppa:sdurobotics/ur-rtde -y && \
#     apt-get update && \
#     apt-get install -y \
#     librtde \
#     librtde-dev \
#     && rm -rf /var/lib/apt/lists/*

# # 2. Clone and build ur_rtde
# RUN git clone https://gitlab.com/sdurobotics/ur_rtde.git /ur_rtde && \
#     cd /ur_rtde && \
#     git submodule update --init --recursive && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make && \
#     make install

# RUN apt-get update && apt-get install -y \
#     python3-pip \
#     && pip3 install ur_rtde \
#     && rm -rf /var/lib/apt/lists/*

# WORKDIR /app