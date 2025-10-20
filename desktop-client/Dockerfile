FROM osrf/ros:jazzy-desktop

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      qt6-base-dev qt6-tools-dev qt6-tools-dev-tools \
      libxcb-xinerama0 libxkbcommon-x11-0 libx11-xcb1 \
      libxrender1 libxrandr2 libxi6 libxfixes3 && \
    rm -rf /var/lib/apt/lists/*
    
WORKDIR /workspace/desktop-client

