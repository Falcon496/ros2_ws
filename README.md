# ros2_ws

## Requirement

* Ubuntu 22.04
* ros2 humble
* gazebo  Fortress
* cmake version 3.29.4

## ガゼボインストール

(https://gazebosim.org/docs/fortress/install_ubuntu)

## 参考URL

https://qiita.com/N622/items/9d89f77d85d9da0af29e
https://qiita.com/sunrise_lover/items/810977fede4b979c382b
https://zenn.dev/tasada038/articles/f2f5b500cdc36a
https://github.com/nek009/key_event/tree/master

## 実行コードメモ

sudo apt install ros-humble-ros-ign
sudo apt install xterm
sudo apt install ros-humble-slam-toolbox

## scan統合ツール

git clone https://github.com/MSDRobotics/ira_laser_tools/tree/humble
rosdep install -r -y -i --from-paths .

