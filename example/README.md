## Example
以Kitti00为例跑通算法，完整数据集从网盘(https://blog.csdn.net/m0_60355964/article/details/125995064)下载

1. 算法输入要求
- W系点云

2. Kitti数据格式
- L系点云
- 12列格式pose，且为T_W_C
- 1列格式pose时间

3. 需用`script/kitti_to_Tum.py`脚本结合外参转换到为T_W_L和Tum格式
