# SLAM assignment

这是我上学校slam入门课的作业。数据来源于Kitti，通过对极线矫正的双目图像进行局部块匹配求解出双目图像的视差，利用非线性最小二乘恢复三维点坐标，最后统一到世界坐标系，[视差结果](result/disparity.bmp),[点云结果1](result/rst02.png),[点云结果2](result/rst03.png)所示。

## Requirement

OpenCV
Eigen
