"""Camera keys of the cell (shared by record_dataset.py and record_raw_cut.py). Standard library only."""

# Topics of ~/igus_rebel_ros2 `ros2 launch rebel_cameras cameras.launch.py` (2026-10-20). All JPEG: on domain 11
# the Orbbecs' raw color/image_raw (3-6 MB/frame) arrived at 5-10 Hz with gaps up to 1.9 s, their image_transport
# .../compressed (~100-150 KiB) at the camera rate (15 fps). Gripper cameras: usb_cam camera JPEG, 30 fps.
CELL_CAMERAS = {
    "head": "/camera/color/image_raw/compressed",
    "wrist_left_gripper": "/arm_left/gripper_camera/image_raw/compressed",
    "wrist_left_knife": "/arm_left/wrist_camera/color/image_raw/compressed",
    "wrist_right_gripper": "/arm_right/gripper_camera/image_raw/compressed",
    "wrist_right_knife": "/arm_right/wrist_camera/color/image_raw/compressed",
    "knife": "/knife_camera/image_raw/compressed",   # Logitech C930e, ros2 launch knife_camera knife_camera.launch.py
    # side_cut_vision probe overlays (~3 Hz; only while the probe runs): published in the ROBOT domain
    "left_vision": "/side_cut_vision/arm_left/overlay/compressed",
    "right_vision": "/side_cut_vision/arm_right/overlay/compressed",
}
MAIN_DOMAIN_CAMERAS = ("left_vision", "right_vision")   # always read in the robot domain, never in --camera-domain
NOT_IN_CELL_PRESET = ("knife",) + MAIN_DOMAIN_CAMERAS
CAMERA_PRESETS = {
    "cell": [k for k in CELL_CAMERAS if k not in NOT_IN_CELL_PRESET],
    "cut-all": list(CELL_CAMERAS),   # every key: head, 305s, grippers, knife webcam, vision overlays (the launch default)
    "cut": ["head", "wrist_left_gripper", "wrist_right_gripper"],  # cameras.launch.py defaults (wrist 305s are off)
    "cell-left": ["head", "wrist_left_gripper", "wrist_left_knife"],
    "cell-right": ["head", "wrist_right_gripper", "wrist_right_knife"],
}
