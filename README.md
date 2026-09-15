# Repo Structure and Running the Pipeline

```text
jetson_board
    ├── run_vio_pipeline.sh
    └── src
triggering_board
    ├── Core
    ├── Drivers
    ├── STM32G474CBTX_FLASH.ld
    ├── STM32G474CBTX_RAM.ld
    ├── triggering_board Debug.launch
    └── triggering_board.ioc
```

The ```jetson_board``` directory is a ROS2 worksapce intended for Humble and the ```triggering_board``` directory is an STM32 workspace.

Within the ROS2 workspace there are 6 packages:

```text
└── src
    ├── amz_vio_pipeline_msgs
    │   ├── CMakeLists.txt
    │   ├── msg
    │   └── package.xml
    ├── bringup
    │   ├── CMakeLists.txt
    │   ├── launch
    │   └── package.xml
    ├── camera_driver
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── LICENSE
    │   ├── package.xml
    │   └── src
    ├── can_driver
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── LICENSE
    │   ├── package.xml
    │   └── src
    ├── orchestrator
    │   ├── CMakeLists.txt
    │   ├── config
    │   ├── include
    │   ├── LICENSE
    │   ├── package.xml
    │   └── src
    └── tools
        ├── CMakeLists.txt
        ├── package.xml
        └── src
```

The ```amz_vio_pipeline_msgs``` contains custom message definitions and ```bringup``` contains the Python launcher file for the pipeline.  ```camera_driver```, ```can_driver```, and ```orchestrator``` are the actual pipeline nodes that get run.  To run the pipeline you must be inside the ```jetson_board``` directory and the packages must be built and sourced.  The pipeline is run by executing the ```run_vio_pipeline.sh``` script (which internally configures the SocketCAN interface and executes the Python launch file:

```bash
cd ~/AMZ_VIO_Pipeline/jetson_board
colcon build
source install/setup.bash
./run_vio_pipeline.sh
```

The ```tools``` package includes various tools to process ROSbags.

# Recording ROSbags and Running Kalibr

To record a ROSbag of the pipeline running use:

```bash
ros2 bag record -s mcap -o <filename> /left_synchronized_images /right_synchronized_images /imu_and_timestamps
```
Once a bag is recorded it can be used to perform a Kalibr calibration or run through OpenVINS to try to extract a trajectory.  To use Kalibr you will either need a computer with ROS1 or if you are on a computer with ROS2 you will need to get a Docker container.  Because the recorded ROSbag is in mcap format it cannot be directly used with Kalibr.  It first needs to be converted to a bag format.  This tool can be used to perform the conversion: https://gitlab.com/ternaris/rosbags.

Once we have the bag file we can run Kalibr using the following command.  The entire process is explained in detail in https://www.youtube.com/watch?v=BtzmsuJemgI&t=2310s.

```bash
rosrun kalibr kalibr_calibrate_imu_camera --imu-models calibrated --reprojection-sigma 1.0 --target ../data/targets/april_6x6_80x80cm.yaml --imu <imu_yaml_file> --cams <camchain_yaml_file> --bag <bag_file>
```
For the target this April grid can be printed on an A0 piece of paper: https://github.com/ethz-asl/kalibr/issues/514 (see the target pdf april_6x6_80x80cm_A0.pdf in the forum post).  For the imu_yaml_file, this file can be used: https://github.com/kgabrielalvarez/AMZ_VIO_Pipeline/blob/kevin_gabriel_alvarez/allan_variance/results/imu_noise_model/imu.yaml.  For the camchain_yaml_file, this file can be used: https://github.com/kgabrielalvarez/AMZ_VIO_Pipeline/blob/kevin_gabriel_alvarez/allan_variance/results/camera_calibration_results/camera_calibration_v2-camchain.yaml







