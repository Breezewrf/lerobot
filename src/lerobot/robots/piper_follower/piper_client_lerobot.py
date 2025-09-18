import threading
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.scripts.server.configs import RobotClientConfig
from lerobot.scripts.server.robot_client import RobotClient
from lerobot.scripts.server.helpers import visualize_action_queue_size
from lerobot.robots.piper_follower.config_piper_follower import PIPERFollowerConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
import json_numpy
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.robots.piper_follower.piper_follower import PIPERFollower as PIPERLeader

# 1. Create the robot instance
"""Check out the cameras available in your setup by running `python lerobot/find_cameras.py`"""
# these cameras must match the ones expected by the policy
# check the config.json on the Hub for the policy you are using
camera_cfg = {
    "top": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30),
    "side": OpenCVCameraConfig(index_or_path=1, width=640, height=480, fps=30)
}

json_numpy.patch()

TASK_DESCRIPTION = "Put the tape in the paper cup"

leader_arm_config = PIPERFollowerConfig(
    port="can0",
    cameras={
        "cam_1": RealSenseCameraConfig(
                serial_number_or_name="130322273975",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=True,
                rotation=Cv2Rotation.NO_ROTATION
        ),
        "right_camera": RealSenseCameraConfig(
            serial_number_or_name="130322272857",
            fps=15,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            use_depth=True,
            rotation=Cv2Rotation.NO_ROTATION
        )
    }
)
# leader_arm_config.port = "can0"  # Example port, adjust as necessary
# leader_arm_config.cameras = {
#     "cam_1": RealSenseCameraConfig(
#         serial_number_or_name="130322273975",
#         fps=15,
#         width=640,
#         height=480,
#         color_mode=ColorMode.RGB,
#         use_depth=True,
#         rotation=Cv2Rotation.NO_ROTATION
#     ),
#     "right_camera": RealSenseCameraConfig(
#         serial_number_or_name="130322272857",
#         fps=15,
#         width=640,
#         height=480,
#         color_mode=ColorMode.RGB,
#         use_depth=True,
#         rotation=Cv2Rotation.NO_ROTATION
#     )
# }

leader_arm = PIPERLeader(leader_arm_config)
# leader_arm.initiate_position()
# 3. Create client configuration
client_cfg = RobotClientConfig(
    robot=leader_arm_config,
    server_address="localhost:8080",
    policy_device="cuda",
    policy_type="act",
    pretrained_name_or_path="./output/train/act_piper_PPTape_TwoCam_Train_v2/checkpoints/last/pretrained_model",
    chunk_size_threshold=0.5,
    actions_per_chunk=50,  # make sure this is less than the max actions of the policy
)

# 4. Create and start client
client = RobotClient(client_cfg)

# 5. Specify the task
task = "Don't do anything, stay still"

if client.start():
    # Start action receiver thread
    action_receiver_thread = threading.Thread(target=client.receive_actions, daemon=True)
    action_receiver_thread.start()

    try:
        # Run the control loop
        client.control_loop(task)
    except KeyboardInterrupt:
        client.stop()
        action_receiver_thread.join()
        # (Optionally) plot the action queue size
        visualize_action_queue_size(client.action_queue_size)