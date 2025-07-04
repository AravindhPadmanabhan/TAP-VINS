# TAP-VINS

TAP-VINS integrates advanced **Tracking Any Point (TAP)** algorithms into the VINS-Mono pipeline for improved visual-inertial odometry performance. It combines ROS-based visual odometry with modern deep learning trackers like CoTracker3, Track-On, and TAPNext.

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

- Go to the Docker Folder
  ```bash
  cd docker
  ```

- Build the Docker Image
  ```bash
  docker build -t tap-vins-img .
  ```
  This may take a while depending on your system and internet speed.

- Run the Docker Container
  ```bash
  ./run_docker.sh
  ```

- To open additional terminals in the container with GUI support:  
  ```bash
  docker exec -it -e DISPLAY=<your_DISPLAY> -e XAUTHORITY=path/to/TAP-VINS/docker/.docker.xauth tap_vins bash
  ```
  - Note: On remote servers with X forwarding, the DISPLAY value may differ between terminals. Always set DISPLAY to match the output of `echo $DISPLAY` in the terminal where the container was launched.

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


## 📊 Evaluation on the EuRoC MAV Dataset

After checking out the branch corresponding to the TAP model you want to evaluate and rebuilding the workspace, run the evaluation script as follows:

```bash
cd catkin_ws/src/VINS-Mono
chmod +x euroc_eval.sh
./euroc_eval.sh <path-to-euroc-rosbags> <branch-name> <rosbag-speed> <eval-metric>
```

- `<path-to-euroc-rosbags>`: Path to the directory containing your EuRoC bag files.
- `<branch-name>`: A custom label you choose, which will:
  - Be appended to the names of the newly recorded bags containing estimated trajectories and ground truth.
  - Define the name of the output directory (inside your EuRoC bags folder) where the recorded bags will be saved.
- `<rosbag-speed>`: Playback speed for the rosbag (e.g. `0.3` for slower playback).
- `<eval-metric>`: The evaluation metric to compute. Valid options are:
  - `"RPE"` for Relative Pose Error (computed over 1-meter trajectory segments).
  - `"APE"` for Absolute Pose Error.

---

### Example Usage

To evaluate the CoTracker branch on EuRoC bags at half playback speed with RPE analysis:
```bash
./euroc_eval.sh /home/username/data/euroc cotracker_eval 0.5 RPE
```
This will generate new bags like `MH_01_cotracker_eval.bag` in a directory `/home/username/data/euroc/cotracker_eval/` and display RPE results for all sequences once evaluation is complete.

---

### Enabling Pose-Graph Optimization

For evaluations using the backend (i.e. global pose-graph optimization):

1. Open the config file:
   ```
   VINS-Mono/config/euroc/euroc_config.yaml
   ```
2. Set:
   ```yaml
   loop_closure: 1
   ```
3. In the launch file:
   ```
   VINS-Mono/vins_estimator/launch/euroc.launch
   ```
   - Uncomment the block that launches the pose-graph optimization node.


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
