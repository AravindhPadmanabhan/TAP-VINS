# TAP-VINS

TAP-VINS integrates advanced **Tracking Any Point (TAP)** algorithms into the VINS-Mono pipeline for improved visual-inertial odometry performance. It combines ROS-based visual odometry with modern deep learning trackers like CoTracker, Track-On, and TAPNext.

---

## 🚀 How to Clone the Repository

First, clone the TAP-VINS repository **with all submodules**:

```bash
git clone --recursive https://github.com/AravindhPadmanabhan/TAP-VINS.git
```

If you already cloned without `--recursive`, initialize and update submodules like this:

```bash
git submodule update --init --recursive
```

## 🐳 Using Docker

We provide a Docker environment to simplify dependencies and ensure reproducibility.

---

### 1. Go to the Docker Folder

Navigate to the docker folder inside your TAP-VINS repository:

```bash
cd docker
```

---

### 2. Build the Docker Image

Build the Docker image using the provided Dockerfile:

```bash
docker build -t tap-vins-img .
```

This may take a while depending on your system and internet speed.

---

### 3. Run the Docker Container

Run the container using the provided script:

```bash
./run_docker.sh
```

This script:

- Starts the container interactively
- Mounts the TAP-VINS codebase into the container
- Sets up display forwarding so you can use tools like RViz
- Downloads weights for the TAP models

Once inside the container, you’ll have a fully configured environment to build and run TAP-VINS.

---

## ✅ Running TAP-VINS

After starting the Docker container, the VINS-Mono submodule is checked out to the `master` branch, which uses the default KLT tracker. To run TAP-VINS with one of the integrated TAP models, switch VINS-Mono to the corresponding branch as shown below:

- Navigate to the VINS-Mono directory:

  ```bash
  cd catkin_ws/src/VINS-Mono
  ```

- Check out the desired branch for your chosen TAP model:

  - **CoTracker3:**

    ```bash
    git checkout cotracker
    ```

  - **Track-On:**

    ```bash
    git checkout trackon
    ```

  - **TAPNext:**

    ```bash
    git checkout tapnext
    ```

- Return to the workspace root and rebuild:

  ```bash
  cd ../..
  catkin build
  source devel/setup.bash
  ```

- Launch TAP-VINS:

  ```bash
  roslaunch vins_estimator euroc.launch
  ```

- Play EuRoC bag files as needed.

- Visualize results in RViz:

  ```bash
  roslaunch vins_estimator vins_rviz.launch
  ```

Refer to individual package READMEs or your thesis documentation for specific usage details.

## ✅ Evaluation on the EuRoC MAV dataset
After checking out to the branch corresponding to the TAP model you want to evaluate and building:
  ```bash
  cd catkin_ws/src/VINS-Mono
  chmod +x euroc_eval.sh
  ./euroc_eval.sh <path-to-euroc-rosbags> <branch-name> <rosbag-speed>
  ```
branch-name is a name you want to append to the bags that have recorded ground truth and estimated trajectories. It is also the name of the directory where the recorded bags will be stored, inside the euroc bags directory.

Once you have the bags recorded, run the following commands to evaluate each bag on [evo](https://github.com/MichaelGrupp/evo):
 ```bash
 pip install evo
 evo_rpe bag <bag-name> /benchmark_publisher/odometry /vins_estimator/odometry -a -d 1 -u m
 ```

## 📝 Requirements

- NVIDIA GPU with drivers compatible with CUDA (if using GPU acceleration)
- Docker
- ROS Noetic (inside the container)
- Sufficient disk space for EuRoC dataset and deep learning models

---

## 🧩 Included TAP Algorithms

This repo integrates:

- [CoTracker](https://github.com/AravindhPadmanabhan/co-tracker)
- [Track-On](https://github.com/AravindhPadmanabhan/track_on)
- [TAPNext](https://github.com/AravindhPadmanabhan/tapnet)

Each included as a submodule under the `tap/` directory.

---

## 📂 Repo Structure

```
TAP-VINS/
├── catkin_ws/              # ROS workspace
├──  ├── tap/               # TAP algorithms as submodules
│    |    ├── co-tracker
│    |    ├── tapnet
│    |    └── track_on
|    └── src/               
│         ├── VINS-Mono
├── docker/                 # Docker setup
└── README.md
```

---

**Happy experimenting with TAP-VINS! 🚀**
