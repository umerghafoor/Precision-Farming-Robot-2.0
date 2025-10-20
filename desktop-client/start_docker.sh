sudo docker run -it --rm --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/1C00FF7F00FF5DE8/Users/Github/Precision-Farming-Robot-2.0/desktop-client:/workspace/desktop-client \
  ros2-jazzy-qt6


xhost +local:root

