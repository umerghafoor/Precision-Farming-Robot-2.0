# allow container to connect to X # allow container to connect to X server (temporary, security tradeoff)
xhost +local:root

# run container (adjust tag name if different)
docker run --rm -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID=0 \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTRTPS_DEFAULT_PROFILES_FILE="" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/workspace:rw \
  -w /workspace/desktop-client \
  --network host \
  --privileged \
  --ipc=host \
  --pid=host \
  precisionfarming-desktop:jazzy \
  bash