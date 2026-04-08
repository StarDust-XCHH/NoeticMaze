# 开发日志-StarDustXCHH



## 2026年3月29日

- 完善了lidar部分的readme`E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\App\lidar\README.md`
- 改写了仿真python程序，目前雷达参考帧由icp节点维护`E:\EBU6475MicroprocessorSystemsDesign\NoeticMaze\asset\Util\sim_client.py`
- 改写了icp程序，添加了define变量，用于维护icp参考帧`E:\EBU6475MicroprocessorSystemsDesign\ICPnoetic\Core\Inc\icp.h`
- 后续开发可以从讲仿真icp进行多线程移植尝试开始



## 2026年3月30日

- 在icp端维护了4bit的2cm分辨率的静态地图`E:\EBU6475MicroprocessorSystemsDesign\ICPnoetic\Core\Src\main.c`