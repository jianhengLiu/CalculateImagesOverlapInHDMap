# CalculateImagesOverlapInHDMap
# 1.目标
**输入/已有数据**
- 3D稠密点云地图
- 相机位姿
- 相机内参
**输出**
- 两张图像间的IoU

# 2. Pipeline
![pipeline](img/pipeline.png)
**思路:**
1. 将稠密点云地图依据位姿转换到相机坐标系下
2. 将所有点云重投影到某一位姿像素平面, 依据相机模型过滤出FOV内的点云(不超出长宽, z>0)
3. 进行ID匹配找到两个位姿下FOV重合部分点云
4. 将重合点云重投影到某一位姿像素平面
5. 形态学处理, 填补空洞
6. 计算有重投影的像素在图片的占比

# 3. 结果

- **实验设置**
    - 相机模型
        - 640x480
        - fx: 380, fy:380
        - cx: 320, cy:240

**3D地图上的效果:**

- 白色:原地图
- 红色: camera1
- 绿色: camera2
- 蓝色: overlap
    - Match Numbers:436838

![project_3dmap](img/project_3dmap.png)

**重投影到camera1上的图像效果: 膨胀→腐蚀**

- 白色: overlap点云重投影
    - Occupied Numbers:169602
    - Occupied Ratio:0.55209
- 黑色: 无重投影点

![reproject_img](img/reproject_img.png)

# **4. 改进空间**

**现存问题**

- 效率低
    - 需要遍历整个点云地图
- 地图点云密度小, 或者相机FOV覆盖点云数量少(靠近墙面等情况), 可能造成重投影空洞多

**改进空间**

- 依据相机模型进行粗滤
- 在地图坐标系下进行FOV点云采集
    - GPU并行raycast


# 1. Usage
1. Download Vicon Room Datasets in EuRoC MAV Dataset which contains a HD pointcloud map
2. replace `ply_path` in `CalculateImagesOverlapInHDMap.cpp` with `PATH_TO_DATASET/V2_01_easy/mav0/pointcloud0/data.ply`
3. run