# rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.035 image:=external_camera/rgb/image_raw camera_name:=external_camera/rgb --no-service-check
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.035 image:=/external_camera/ir/image camera_name:=external_camera/ir --no-service-check
