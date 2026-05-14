FROM ros:humble-ros-core

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential cmake \
      qt6-base-dev qt6-tools-dev qt6-tools-dev-tools libqt6opengl6-dev \
      libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libxcb-xfixes0 libxcb-shape0 libxcb-randr0 libxcb-glx0 \
      libxkbcommon-x11-0 libx11-xcb1 \
      libxrender1 libxrandr2 libxi6 libxfixes3 \
      libgl1-mesa-dev libglu1-mesa-dev libopengl0 libglvnd-dev libglx-dev \
      libglx0 libgl1-mesa-glx libglvnd0 \
      mesa-utils mesa-va-drivers \
      libxkbcommon-dev && \
    rm -rf /var/lib/apt/lists/*
    
WORKDIR /workspace/desktop-client

