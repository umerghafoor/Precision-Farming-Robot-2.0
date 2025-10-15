apt update
apt install -y qt6-base-dev qt6-tools-dev qt6-tools-dev-tools

sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libx11-xcb1 libxrender1 libxrandr2 libxi6 libxfixes3



docker run -it --rm --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/1C00FF7F00FF5DE8/Users/Github/Precision-Farming-Robot-2.0/desktop-client:/workspace/desktop-client \
  osrf/ros:jazzy-desktop
