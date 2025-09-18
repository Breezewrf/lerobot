from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.record import record_loop
from lerobot.robots.lekiwi.config_lekiwi import LeKiwiClientConfig
from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import _init_rerun
from lerobot.robots.piper_follower.config_piper_follower import PIPERFollowerConfig
from lerobot.robots.piper_follower.piper_follower import PIPERFollower as PIPERLeader
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation
import numpy as np
import time
from lerobot.datasets.utils import build_dataset_frame
NUM_EPISODES = 30
FPS = 30
EPISODE_TIME_SEC = 10
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "Put the tape in the paper cup"

keyboard_config = KeyboardTeleopConfig()
keyboard = KeyboardTeleop(keyboard_config)

leader_arm_config = PIPERFollowerConfig
leader_arm_config.port = "can0"  # Example port, adjust as necessary
leader_arm_config.cameras = {
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

leader_arm = PIPERLeader(leader_arm_config)

# Configure the dataset features
action_features = hw_to_dataset_features(leader_arm.action_features, "action")
obs_features = hw_to_dataset_features(leader_arm.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

# Create the dataset
dataset = LeRobotDataset.create(
    repo_id="breezewrf/PiperLeRobot_PPTape_TwoCam_Train_v2.1www",
    fps=FPS,
    features=dataset_features,
    robot_type=leader_arm.name,
    use_videos=True,
    image_writer_threads=4,
)

leader_arm.connect()
keyboard.connect()

listener, events = init_keyboard_listener()

if not leader_arm.is_connected or not keyboard.is_connected:
    raise ValueError("Robot, leader arm of keyboard is not connected!")

recorded_episodes = 0
while recorded_episodes < NUM_EPISODES:
    input("Press Enter to start recording episode...")
    log_say(f"Recording episode {recorded_episodes}")
    for t in range(int(FPS * EPISODE_TIME_SEC)):
        # Get observation from robot
        print("recorded_episodes:", recorded_episodes)
        obs = leader_arm.get_observation()
        # Get action from teleop (if available)
        action = leader_arm.get_action()
        # print("obs:", obs)
        print("action:", action)
        # Merge observation and action into one frame
        frame = {'action': action, 'observation.state': obs, 'observation.images.cam_1': obs['cam_1']}
        # 假设关节顺序是固定的
        joint_names = ['joint_1.pos', 'joint_2.pos', 'joint_3.pos', 'joint_4.pos', 
                    'joint_5.pos', 'joint_6.pos', 'gripper.pos']

        # 将action字典转换为numpy数组
        action_values = [
            frame['action']['joint_1.pos'],
            frame['action']['joint_2.pos'],
            frame['action']['joint_3.pos'],
            frame['action']['joint_4.pos'],
            frame['action']['joint_5.pos'],
            frame['action']['joint_6.pos'],
            frame['action']['gripper.pos']
        ]
        action_array = np.array(action_values, dtype=np.float32)

        # 将observation.state字典转换为numpy数组(除了cam_1)
        obs_state_values = [
            frame['observation.state']['joint_1.pos'],
            frame['observation.state']['joint_2.pos'],
            frame['observation.state']['joint_3.pos'],
            frame['observation.state']['joint_4.pos'],
            frame['observation.state']['joint_5.pos'],
            frame['observation.state']['joint_6.pos'],
            frame['observation.state']['gripper.pos']
        ]
        obs_state_array = np.array(obs_state_values, dtype=np.float32)

        # 构造新的frame字典
        new_frame = {
            'action': action_array,
            'observation.state': obs_state_array,
            'observation.images.cam_1': frame['observation.state']['cam_1'],  # 保持相机数据不变
            'observation.images.right_camera': frame['observation.state']['right_camera'],
        }
        # Add frame to dataset buffer
        dataset.add_frame(new_frame, task=TASK_DESCRIPTION)

        # Sleep to maintain FPS
        time.sleep(1.0 / FPS)

        print(f"Recorded frame {t + 1} for episode {recorded_episodes + 1}")

    # dataset.save_episode()

    # Upload to hub and clean up
    # dataset.push_to_hub()
    # recorded_episodes += 1

leader_arm.disconnect()
keyboard.disconnect()
    
