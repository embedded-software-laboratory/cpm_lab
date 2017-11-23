# Intrinsic Camera Calibration


TODO


# Extrinsic Camera Calibration

Record an image form the installed camera, where the coordinate system gird markers are visible.

Then run:

    rosrun ips extract_camera_pose_calibration_points /path/to/image.tif | tee /tmp/point_pairs

This detects the coordinate system markers and outputs corresponding image (2D) and world (3D) points. Check `/tmp/point_pairs` to see if the points are plausible. You can also add points manually.

Then run the following, replacing `<serial_no>` with the camera serial number.

    cd .../ROS/src/ips/cfg/cameras/<serial_no>/
    rosrun ips calculate_camera_pose_calibration intrinsic_parameters.yaml </tmp/point_pairs | tee extrinsic_parameters.yaml

This calculates and saves the camera rotation matrix and translation vector.