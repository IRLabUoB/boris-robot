rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.035 image:=left_camera/rgb/image_raw camera_name:=left_camera --no-service-check


rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.035 image:=right_camera/rgb/image_raw camera_name:=right_camera --no-service-check
