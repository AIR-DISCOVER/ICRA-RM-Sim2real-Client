docker run -it --rm --gpus all --network net-1 --name cli\
	--cpus=5.6 -m 8192M \
	-v /dev:/dev -e DISPLAY=:2 -e QT_X11_NO_MITSHM=1 \
    -e ROS_MASTER_URI=http://rosmaster:11311 \
	-v /dev/bus/usb:/dev/bus/usb \
    -v /dev/video0:/dev/video0 \
    -v /dev/video1:/dev/video1 \
    -v /dev/video2:/dev/video2 \
    -v /dev/video3:/dev/video3 \
    -v /dev/video4:/dev/video4 \
    -v /dev/video5:/dev/video5 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    cli_test $@
