# Piper Follower Robot Module
This folder contains code for controlling and interfacing with the PIPER Follower robotic arm using the Piper SDK. It provides:

- Robot Control Class: `piper_follower.py` implements the PIPERFollower class, supporting connection, calibration, action/observation retrieval, and camera integration.
- Camera Integration: Supports RealSense and OpenCV cameras for visual feedback and dataset recording.
- Data Recording: `record_data.py` enables recording robot actions and observations for dataset creation.
- Client Scripts: Example clients (`piper_client_gr00t.py`, `piper_client_lerobot.py`) show how to interact with the robot, send actions, and receive observations, including integration with external servers.
- Testing: `piper_test.py` and `realsense_test.py` provide basic usage and camera tests.

## Typical Usage:

### LeRobot format dataset create
Initiate robot:

`cd /home/breeze/Desktop/workplace/EmbodiedAI/piper_sdk/piper_sdk`

`bash can_activate.sh can0 1000000`

1. Date record
`python src/lerobot/robots/piper_follower/record_data.py`

2. Data visualization
`python -m lerobot.scripts.visualize_dataset     --repo-id breezewrf/PiperLeRobot     --episode-index 0`

### Run Asynchronous Inference 
- Server

`python src/lerobot/scripts/server/policy_server.py     --host=127.0.0.1     --port=8080`

- Client

```sh
python src/lerobot/scripts/server/robot_client.py --server_address=127.0.0.1:8080 --robot.type=piper_follower --robot.port=can0 --robot.cameras="{ cam_1: {serial_number_or_name: 130322273975, width: 640, height: 480, fps: 15}, right_camera: {serial_number_or_name: 130322272857, width: 640, height: 480, fps: 15}}" --policy_type=act --pretrained_name_or_path=./output/train/act_piper_PPTape_TwoCam_Train_v2/checkpoints/last/pretrained_model --policy_device=cuda --actions_per_chunk=50 --chunk_size_threshold=0.5 --aggregate_fn_name=weighted_average --debug_visualize_queue_size=True

OR

python src/lerobot/robots/piper_follower/piper_client_lerobot.py 
```

### Try some test
`python src/lerobot/robots/piper_follower/piper_test.py` will connect to the piper arm, and output current observations and actions.

`python src/lerobot/robots/piper_follower/realsense_test.py` will connect to the cameras using Camera Config.