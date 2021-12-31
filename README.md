# ros_rober
ROS対応の自作ロボットカー

## Howto
ワークスペースのディレクトリの`src`上にこのレポジトリをクローン後、ビルド

```
cd ~/catkin_ws/src
git clone https://github.com/Miura55/ros_rober
cd ~/catkin_ws
catkin_make
```
## トラブルシューティング
ビルド時に`Could not find a package configuration file provided by "geometry_msgs"`と表示されるときは、Raspberry Piにメッセージオブジェクトが入っていない可能性が高い。

以下のコマンドでメッセージオブジェクトのレポジトリをクローンして再度ビルドする

```
cd ~/ros_catkin_ws/src
git clone https://github.com/ros/common_msgs.git
cd ~/ros_catkin_ws
catkin_make
```
