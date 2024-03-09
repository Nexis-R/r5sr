#!/bin/sh

echo "Installing Dev dependencies..."
sudo apt update -qq
sudo apt install -y -qq ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-mixin \
  mold \
  ccache \
  ninja-build \
  clang \
  libstdc++-12-dev \
  clangd \
  clang-format \
  cmake-format \
  wget \
  busybox \
  python3-pyocr \
  git-lfs

if ! command -v just >/dev/null 2>&1; then
  echo "Installing just..."
  wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | sudo tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1>/dev/null
  echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | sudo tee /etc/apt/sources.list.d/prebuilt-mpr.list
  sudo apt update
  sudo apt install just
fi
