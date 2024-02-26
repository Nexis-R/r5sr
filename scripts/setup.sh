#!/bin/bash

SCRIPT_DIR=$(dirname "$0")

if [ "$1" = "robot" ]; then
  # Install epos command library
  # Check existance
  FILE_PATTERN="/opt/EposCmdLib_*/lib"
  if [ -e $FILE_PATTERN ]; then
    echo "Epos command library is already installed."
  else
    echo "Epos command library is not installed. Installing..."
    ${SCRIPT_DIR}/dependencies/epos_command_library.sh
  fi

  # Install udev rules
  echo "Installing udev rules..."
  ${SCRIPT_DIR}/udev_rules/install.sh

elif [ "$1" = "teleop" ]; then
  echo "Installing teleop dependencies..."
else
  echo "Invalid argument. Please use 'robot' or 'teleop'."
  exit 1
fi

# Install dev tools
echo "Installing dev tools..."
${SCRIPT_DIR}/dependencies/dev.sh

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# Add settings to .bashrc
echo "Adding settings to .bashrc..."
# スクリプトファイルのあるディレクトリの絶対パスを取得
ABS_DIR=$(
  cd ${SCRIPT_DIR}/.. &
  >/dev/null && pwd
)

# 'install/setup.bash'のフルパスを生成
FULL_PATH="$ABS_DIR/install/setup.bash"

# ~/.bashrc に追加する設定
SETTINGS=(
  "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  "source $FULL_PATH"
)

# ~/.bashrc のパス
BASHRC="$HOME/.bashrc"

for SETTING in "${SETTINGS[@]}"; do
  # .bashrc に設定が既に存在するか確認
  if ! grep -qF -- "$SETTING" "$BASHRC"; then
    # 設定が存在しない場合、.bashrc の末尾に追加
    echo "$SETTING" >>"$BASHRC"
    echo "Added '$SETTING' to $BASHRC"
  else
    echo "'$SETTING' is already in $BASHRC"
  fi
done
