# Control & FSM
控制机和move_base调用部分都写在move_control包中  

# Perception
感知部分，使用mid360进行slam建图定位  
同时通过pointcloud_to_laserscan将点云转换为激光雷达数据供move_base使用，生成代价地图


# Pathplanner
编写了自定义局部路径规划器MyPlanner  
适配该机器人实际任务  
可以通过user_config 配置默认参数，并且启动 rqt_reconfigure 动态调整参数   

# Hardware Connection
硬件层——采用串口与下位机通信，波特率115200  
三种控制帧  
id = 1  
id = 2  
id = 3  

# Simulation
仿真相关，使用