# mavros订阅消息
1. global_position订阅GPS数据

   - 消息名称: mavros/global_position/global
   - 类型名称: sensor_msgs::NavSatFix.h
   - 类型所在头文件: sensor_msgs/NavSatFix.h
   - 常用类成员变量:
   ```C
    float64 latitude *//经*
    float64 lontitude *//纬*
    float64 altitude *//海拔*
    float64[9] position_covariance
    uint8 position_covariance_type
   ```
2. imu_pub订阅IMU消息

   - 消息名称: 滤波后的mavros/imu/data
   - 类型名称: sensor_msgs::Imu
   - 类型所在头文件: sensor_msgs/Imu.h
   - 常用类成员变量:
   ```C
    geometry_msgs::Quaternion orientation *//旋转四元数（xyzw）*
    float64[9] orientation_covariance *//方差*
    geometry_msgs::Vector3 angular_velocity *//3轴角速度（xyz）*
    float64[9] angular_velocity_covariance *//方差*
    geometry_msgs::Vector3 linear_accleration *//线性加速度（xyz）*
    float64[9] linear_accleration_covariance *//方差*

   ```
3. local_position订阅本地位置数据
   - 消息名称: mavros/local_position/pose
   - 类型名称: geometry_msgs::PoseStamped
   - 类型所在头文件: geometry_msgs/PoseStamped.h
   - 常用类成员变量:
     ```C
     geometry_msgs::Pose pose
     ```

4. manual_control 订阅遥控器的值
   - 消息名称：mavros/manual_control/control

   - 类型名称：mavros_msgs::ManualControl

   - 类型所在头文件：mavros_msgs::ManualControl.h
     
5. sys_status 查询系统状态

   - 消息名称：mavros/state
  
   - 类型名称：mavros_msgs::State
  
   - 类型所在头文件：mavros_msgs/State.h

6. waypoint 航点信息

   - 消息名称：mavros/mission/waypoint
  
   - 类型名称：geometry_msgs::WaypointList
  
   - 类型所在头文件：mavros_msgs/WaypointList.h
# mavros发布消息
1. actuator_control 控制飞控IO输出（混控器）

    - 消息名称：mavros/actuator_control
    
    - 类型名称：geometry_msgs::PoseStamped
    
    - 类型所在头文件：mavros_msgs/Actuator_Control.h
    
    - 常用类成员变量：
    ```C
    uint8 group_mix *//要控制的混控器分组1-8（control group）*
    float32[8] controls *//控制量（前四个分别是：roll、pitch、yaw、thrust）*
    
    ```
2. setpoint_accel 控制期望的加速度

    - 消息名称：mavros/setpoint_accel/accel
    
    - 类型名称：geometry_msgs::Vector3Stamped
    
    - 类型所在的头文件：geometry_msgs/Vector3Stamped.h
    
    -  常用类成员变量：
    ```C
    geometry_msgs::Vector3 vector *//三轴加速度*
    ```
3. setpoint_attitude 控制期望的姿态
  
    - 消息名称：mavros/setpoint_attitude/attitude
    
    - 类型名称：geometry_msgs::PoseStamped
    
    - 类型所在的头文件：geometry_msgs/PoseStamped.h
    
    - 常用类成员变量：
    ```C
    geometry_msgs::Pose pose *//三个欧拉角，或者是四元数任选其一*
    ```
4. setpoint_position 控制期望的位置（相对坐标）
  
    - 消息名称：mavros/setpoint_position/local
    
    - 类型名称：geometry_msgs::PoseStamped
    
    - 类型所在的头文件：geometry_msgs/PoseStamped.h
    
    - 常用类成员变量：
    ```C
    geometry_msgs::Pose pose *//NED坐标系下的位置（xyz），只有position成员变量生效*
    
    ```
5. setpoint_velocity 控制期望的速度
  
    - 消息名称：mavros/setpoint_velocity/cmd_vel
    
    - 类型名称：geometry_msgs::TwistStamped
    
    - 类型所在的头文件：geometry_msgs/TwistStamped.h
    
    - 常用类成员变量：
    ```C
    geometry_msgs::Twist twist *//三轴速度*
    geometry_msgs::Twist类成员变量：
    geometry_msgs::Vector3 linear *//三轴线性速度*
    geometry_msgs::Vector3 angular *//三轴角速度*
    ```
6. setpoint_position控制期望的位置（GPS坐标）

    - 消息名称：mavros/setpoint_position/global
    
    - 类型名称：mavros_msgs::GlobalPositionTarget
    
    - 类型所在的头文件：mavros_msgs/GlobalPositionTarget.h
    
    - 常用类成员变量：
    ```C
    uint8 coordinate_frame *//5为绝对GPS坐标系，6为相对高度GPS坐标*
    uint16 type_mask
    
    ```

# mavros服务
1. arming Services 加解锁服务
  
    - 消息名称：mavros/cmd/arming
    
    - 类型名称：mavros_msgs::CommandBool
    
    - 类型所在的头文件：mavros_msgs/CommandBool.h

2. 模式切换消息名称：mavros/set_mode

    - 类型名称：mavros_msgs::SetMode
    
    - 类型所在的头文件：mavros_msgs/SetMode.h
