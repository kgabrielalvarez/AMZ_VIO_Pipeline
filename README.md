How to run the pipeline:

```bash
cd ~/AMZ_VIO_Pipeline/jetson_board
colcon build
source install/setup.bash
./run_vio_pipeline.sh
```

Check that messages are being sent:

```bash
ros2 topic echo 
```
