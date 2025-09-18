from lerobot.robots.piper_follower.piper_follower import PIPERFollower
from lerobot.robots.piper_follower.config_piper_follower import PIPERFollowerConfig
import time
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation

def main():
    # Create a config object (customize as needed)
    cameras_cfg = {
        "cam2_1": RealSenseCameraConfig(
                serial_number_or_name="130322273975",
                fps=15,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                use_depth=True,
                rotation=Cv2Rotation.NO_ROTATION
            )
    }
    config = PIPERFollowerConfig(
        port="can0",
        cameras=cameras_cfg
    )
    print("Robot config:", config)
    # config.port = "can0"  # Example port, adjust as necessary
    # config.cameras = {
    #         "cam_1": RealSenseCameraConfig(
    #             serial_number_or_name="130322273975",
    #             fps=15,
    #             width=640,
    #             height=480,
    #             color_mode=ColorMode.RGB,
    #             use_depth=True,
    #             rotation=Cv2Rotation.NO_ROTATION
    #         )
    #     }
    robot = PIPERFollower(config)
    try:
        robot.connect()
        print("Robot connected:", robot.is_connected)
    except Exception as e:
        print("Connection failed:", e)
    while robot.is_connected:
        # Example of reading the robot's state
        print("Robot state:", robot.get_observation())
        print("Master Action:", robot.get_action())
        time.sleep(1)
        # robot.disconnect()
    
if __name__ == "__main__":
    main()