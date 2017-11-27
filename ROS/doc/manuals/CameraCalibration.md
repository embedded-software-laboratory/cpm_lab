# Intrinsic Camera Calibration


Open MATLAB and run `cameraCalibrator`. 

Follow this guide: [https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html](https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html)

When you have the `cameraParams` data in your MATLAB workspace, run the following code snippet:

    clc
    k = [cameraParams.RadialDistortion 0];
    p = struct(...
        'fx',cameraParams.FocalLength(1),...
        'fy',cameraParams.FocalLength(2),...
        'cx',cameraParams.PrincipalPoint(1),...
        'cy',cameraParams.PrincipalPoint(2),...
        'k1',k(1), 'k2',k(2), 'k3',k(3),...
        'p1',cameraParams.TangentialDistortion(1),...
        'p2',cameraParams.TangentialDistortion(2));
    for f = fieldnames(p)'
        fprintf('%s: %f\n',f{1},p.(f{1}));
    end

Save the resulting text in the file `.../ROS/src/ips/cfg/cameras/<serial_no>/intrinsic_parameters.yaml` where `<serial_no>` is the camera serial number.

# Extrinsic Camera Calibration

Record an image form the installed camera, where the coordinate system gird markers are visible.

Then run:

    rosrun ips extract_camera_pose_calibration_points /path/to/image.tif | tee /tmp/point_pairs

This detects the coordinate system markers and outputs corresponding image (2D) and world (3D) points. Check `/tmp/point_pairs` to see if the points are plausible. You can also add points manually.

Then run the following, replacing `<serial_no>` with the camera serial number.

    cd .../ROS/src/ips/cfg/cameras/<serial_no>/
    rosrun ips calculate_camera_pose_calibration intrinsic_parameters.yaml </tmp/point_pairs | tee extrinsic_parameters.yaml

This calculates and saves the camera rotation matrix and translation vector.

With the following commands you can visualize the calibrated coordinate system and check whether it matches the real coordinate system.

    cd .../ROS/src/ips/cfg/cameras/<serial_no>/
    rosrun ips validate_camera_calibration /path/to/image.tif intrinsic_parameters.yaml extrinsic_parameters.yaml