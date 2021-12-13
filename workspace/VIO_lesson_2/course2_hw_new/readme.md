代码有两个压缩包,一个是ros, 一个非ros

#### 1. 对非 ros : 生成运动 imu 数据

1. 编译 

2. 执行bin/data_gen, 生成数据 

3. 执行python_tools/draw_trajectory.py 画出轨迹

4. 换成中值积分, 再重做一遍上述1,2,3过程

#### 2. 对ROS: 专门生成静止 imu 数据，用于 allan 方差标定

##### 2.1 使用 imu_utils 完成 allan 标定

1. ros下编译 
2. 执行, 生成 imu.bag 
3. rosbag play -r 500 imgimu_utils.bag 回放
4. 用imu_utils进行接收和分析
5. 用imu_utils下的scripts/下的matlab 脚本画allan曲线

##### 2.2 使用 kalibr_allan 完成 allan 标定（推荐）

1. ros下编译 
2. 执行, 生成 imu.bag 
3. 用kalibr_allan的bagconver把bag转成 mat文件, 见readme
4. 用kalibr_allan的m脚本对mat文件进行分析, (需修改m文件中的mat文件路径)
5. 用kalibr_allan的m脚本画allan曲线, (需修改m文件中的result文件路径)

m 脚本运行需要matlab, 安装耗时,  最好提前做. 最好是2018版本
