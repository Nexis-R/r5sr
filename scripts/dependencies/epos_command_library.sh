#!/bin/sh

# 必要なパッケージがインストールされているか確認し、なければインストール
for pkg in wget busybox; do
  if ! command -v $pkg >/dev/null 2>&1; then
    echo "$pkg is not installed. Installing..."
    sudo apt update -qq
    sudo apt install -y -qq $pkg
  else
    echo "$pkg is already installed."
  fi
done

wget https://maxonjapan.com/wp-content/uploads/manual/epos/EPOS_Linux_Library_E.zip -O - | busybox unzip - -d /tmp

echo "Installing library..."
cd /tmp/EPOS_Linux_Library && sudo bash ./install.sh
