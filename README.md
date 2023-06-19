# PL-VI-ORBSLAM3
This system has been modified based on the monocular + IMU mode of ORB-SLAM3 by integrating line features, thereby achieving a monocular + IMU SLAM system that utilizes both point and line features. 
To track line features, we have incorporated the constraints described in the paper 'Structure-SLAM: Low-Drift Monocular SLAM in Indoor Environments' into the system."（https://github.com/yanyan-li/Structure-SLAM-PointLine）.
This code has been adapted from the open-source code available at https://github.com/Giannis-Alamanos/ORB-LINE-SLAM. The necessary environment dependencies for running the code can be installed based on the requirements specified in the provided link (OpenCV version 4.2 is utilized).

# Log
2023.0608 添加重定位线程，完成关键帧获取、删除、完成整体框架

2023.0609 仿照ORB-SLAM3中的词带，编写词带匹配部分，完成简单的词带检测，后续还需要改进。

2023.0614 重定位代码整体已完成，后续将对真实数据进行测试。

2023.0617 yaml文件中新增bRelocalization项，当它设为1时，启动重定位线程，当它设置为0时，不开启重定位线程。

2023.0619 整体基本实现，后续计划用深度学习的方法代替词带进行重定位。

