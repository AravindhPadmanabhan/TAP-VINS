# Use ROS Noetic as the base image
FROM osrf/ros:noetic-desktop-full

# Set non-interactive mode to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install dependencies for Ceres Solver
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-pip \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /home

# Clone Ceres Solver
RUN git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout 1.14.x && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install

# Ensure Python and pip are up to date
RUN python3 -m pip install --upgrade pip && \
    pip install catkin_pkg rospkg empy && \
    pip install catkin-tools

WORKDIR /home

# Clone and build TAP-VINS
RUN git clone https://github.com/AravindhPadmanabhan/TAP-VINS.git && \
    cd TAP-VINS && \
    git submodule update --init --recursive && \
    cd catkin_ws && \
    bash -c "source /opt/ros/noetic/setup.bash && catkin build"

WORKDIR /home

RUN cd TAP-VINS/catkin_ws/tap/co-tracker && \
    pip install -e . && \
    pip install matplotlib flow_vis tqdm tensorboard && \
    pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117 && \
    pip install imageio[ffmpeg] && \
    mkdir -p checkpoints && \
    cd checkpoints && \
    wget https://huggingface.co/facebook/cotracker3/resolve/main/scaled_online.pth && \
    wget https://huggingface.co/facebook/cotracker3/resolve/main/scaled_offline.pth


WORKDIR /home

RUN sudo apt update && \
    sudo apt install -y python3-tk

# Clone and install Track-On
RUN cd TAP-VINS/catkin_ws/tap/track_on && \
    pip install mmcv==2.2.0 -f https://download.openmmlab.com/mmcv/dist/cu117/torch1.13/index.html && \
    pip install -r requirements.txt && \
    pip install numpy==1.24.4 && \
    pip install -e . && \
    mkdir checkpoints && \
    cd checkpoints && \
    wget https://huggingface.co/gaydemir/track_on/resolve/main/track_on_checkpoint.pt

WORKDIR /home

# Clone and install TAPNext
RUN cd TAP-VINS/catkin_ws/tap/tapnet && \
    pip install -r requirements.txt && \
    pip install -e . && \
    mkdir checkpoints && \
    cd checkpoints && \
    wget https://storage.googleapis.com/dm-tapnet/tapnext/bootstapnext_ckpt.npz

# Set default shell to bash
CMD ["/bin/bash"]
