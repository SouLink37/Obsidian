![[VINS Ros Node Graph.png]]

![[vins整体流程.png]]


### 回环检测中的2种位姿：
	vio位姿：滑窗优化后的位姿
	 最终位姿：回环优化后调整的位姿

### 输入:
```
VIO系统 (VINS)
    ↓
【Topic 1】/odometry → VioCallback
    └─ 提供：实时位姿 (用于修正输出)
    
【Topic 2】/keyframe_pose → PoseCallback  
    └─ 提供：关键帧位姿
    
【Topic 3】/keyframe_point → PointCallback
    └─ 提供：关键帧的3D点 + 2D投影点
    
【Topic 4】compressed_image_topic → ImageCallback
    └─ 提供：关键帧图像（RGB压缩格式）
         解码 → 分左右 → 转灰度
    
【Topic 5】/extrinsic → ExtrinsicCallback
    └─ 提供：IMU-相机外参
    
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Process线程】同步上述数据 → 构建当前关键帧
```




### odometry vs keyframe_pose
|特性|/odometry (VIO实时位姿)|/keyframe_pose (关键帧位姿)|
|---|---|---|
|频率|很高（每帧都有）|较低（只有关键帧）|
|用途|🔄 实时位姿修正与发布|📍 构建关键帧用于回环检测|
|处理方式|✅ 直接处理|⏳ 缓存同步|
|是否入队|❌ 否|✅ 是|
|应用场景|发送给下游应用|回环检测与优化|
|影响范围|实时控制输出|离线约束优化|


### 关键帧

```
KeyFrame *keyframe = new KeyFrame(
    timestamp,          // 时间戳
    frame_index,        // 帧索引
    T,                  // ← 【1】关键帧位姿 - 平移
    R,                  // ← 【1】关键帧位姿 - 旋转
    image,              // ← 【2】当前灰度图
    point_3d,           // ← 【3】3D关键点（VIO坐标系）
    point_2d_uv,        // ← 【3】2D投影（像素坐标）
    point_2d_normal,    // ← 【3】2D投影（归一化坐标）
    point_id,           // ← 【3】点的ID
    sequence            // 序列号
);
```

|判决条件|含义|结论|
|---|---|---|
|前两帧|初始化需要|📍 必须是关键帧|
|追踪点<20|前一帧特征丢失多|📍 需要新关键帧重新定位|
|长期点<40|特征容易丢|📍 需要新关键帧保存|
|新点>旧点50%|场景变化大|📍 需要新关键帧记录场景|
|视差充足|机器人运动够大|📍 保留这一帧作关键帧|
|视差不足|机器人运动太小|⏭️ 跳过，不要关键帧|



```
【时刻T-1】旧关键帧
    │
    └─ 光流追踪特征点
           ↓
    【时刻T】当前帧（非关键帧）
           ├─ last_track_num = 15 (追踪到的点变少了!)
           ├─ 判决：last_track_num < 20
           └─ 结论：需要新关键帧！
           ↓
【时刻T】做成新关键帧
           ├─ setMask() (给旧特征点划圈，禁止检测区域)
           ├─ 在mask外用goodFeaturesToTrack检测新角点
           │   (新角点数 = max_cnt - cur_pts.size())
           └─ addPoints() 把新角点加到cur_pts里
           ↓
【时刻T+1】下一帧
           ├─ 对【时刻T】的所有点进行光流追踪
           │   (包括旧的15个点 + 新检测的N个点)
           ├─ last_track_num = 15 + N - 丢失的
           └─ ✓ 追踪点数恢复了！
```

### 流程图：

```
【时刻T】Process 线程
    ├─ 同步接收3个话题
    │  ├─ /keyframe_pose → 位姿 (T, R)
    │  ├─ /keyframe_point → 点 (point_3d, point_2d_uv)
    │  └─ image_topic → 图像 (image)
    │
    ├─ 构建 KeyFrame 对象
    │  └─ kf = KeyFrame(T, R, image, point_3d, point_2d_uv, ...)
    │
    └─ 加入 PoseGraph
       └─ pose_graph_.AddKeyFrame(kf, 1)
            ↓
【PoseGraph 后端线程】
    ├─ 存储所有历史关键帧
    │  └─ keyframe_list_ = [kf0, kf1, kf2, ..., kfT]
    │
    ├─ 【步骤1】词袋查询（用 【2】图像 + 【1】位姿）
    │  ├─ 提取 kfT->image 的 BRIEF 特征
    │  ├─ 在词袋数据库中查询相似帧
    │  └─ 返回候选帧列表 [kf0, kf50, kf100]
    │
    ├─ 对每个候选帧（如 kf0）
    │  │
    │  ├─ 【步骤2】特征匹配（用 【3】关键点）
    │  │  ├─ kfT->point_2d_uv (当前帧的2D点)
    │  │  ├─ vs kf0->keypoints (历史帧的2D点)
    │  │  ├─ 用 Hamming 距离匹配
    │  │  └─ 输出：匹配对 [(P_cur, P_old), ...]
    │  │
    │  ├─ 【步骤3】F-Matrix RANSAC（用 【3】2D投影）
    │  │  ├─ kfT->point_2d_normal (当前帧2D点)
    │  │  ├─ vs kf0->keypoints_norm (历史帧2D点)
    │  │  ├─ 几何验证
    │  │  └─ 过滤外点
    │  │
    │  └─ 【步骤4】PnP RANSAC（用 【3】3D点 + 【1】位姿）
    │     ├─ kfT->point_3d (当前帧3D点，VIO坐标系)
    │     ├─ 投影到 kf0 的相机坐标系
    │     ├─ 用 kf0->origin_vio_R, origin_vio_T 初始化
    │     ├─ 估计相对位姿
    │     └─ 输出：回环约束
    │
    └─ 后端优化
       ├─ 用回环约束优化位姿图
       └─ 更新全局漂移修正 (r_drift_, t_drift_)
```



### 当前关键帧：

```
当前关键帧 (Current KeyFrame)
│
├─ 【特征类型1】Window BRIEF (来自VIO的3D点)
│  ├─ 来源：point_3d (VIO输出的3D特征点)
│  ├─ 投影：投影到图像平面 → point_2d_uv (像素坐标)
│  ├─ 提取：computeWindowBRIEFPoint()
│  │       ↓ 计算BRIEF特征描述 → window_brief_descriptors
│  │
│  └─ 成员变量：
│      - point_3d: vector<Point3f> (3D点在VIO坐标系)
│      - point_2d_uv: vector<Point2f> (图像像素坐标)
│      - point_2d_norm: vector<Point2f> (归一化坐标)
│      - window_keypoints: vector<KeyPoint>
│      - window_brief_descriptors: vector<BRIEF::bitset>
│
├─ 【特征类型2】Global BRIEF (FAST角点)
│  ├─ 来源：图像本身 (灰度图)
│  ├─ 检测：FAST算法 (threshold=20)
│  ├─ 提取：computeBRIEFPoint()
│  │       ↓ 计算BRIEF特征描述 → brief_descriptors
│  │
│  └─ 成员变量：
│      - keypoints: vector<KeyPoint>
│      - keypoints_norm: vector<KeyPoint>
│      - brief_descriptors: vector<BRIEF::bitset>
│
└─ 【核心数据】
   - image: 灰度图
   - vio_T_w_i, vio_R_w_i: VIO位姿
   - sequence: 序列号
```


### PNP
```
	solvePnPRansac(
    matched_3d,              // 【参数1】objectPoints
    matched_2d_old_norm,     // 【参数2】imagePoints
    K,                       // 【参数3】cameraMatrix
    D,                       // 【参数4】distCoeffs
    rvec,                    // 【参数5】rvec (输入+输出)
    t,                       // 【参数6】tvec (输入+输出)
    true,                    // 【参数7】useExtrinsicGuess
    100,                     // 【参数8】iterationsCount
    20.0 / 460.0,            // 【参数9】reprojectionError
    0.99,                    // 【参数10】confidence
    inliers);                // 【参数11】inliers (输出)
```
#### 【参数1】objectPoints - 输入的3D点
```
matched_3d
    ↓
const vector<cv::Point3f> &matched_3d

【含义】
    ├─ 物体上的3D特征点坐标
    ├─ 单位：米(m)
    ├─ 坐标系：【当前帧相机坐标系】
    ├─ 数量：N个匹配的特征点
    │
    └─ 【例】
       matched_3d[0] = (0.5, 0.3, 5.0)   第1个特征点
       matched_3d[1] = (1.2, -0.1, 4.8)  第2个特征点
       ...
       matched_3d[n] = (...)             第N个特征点

【来源】
    点_3d 来自 /keyframe_point 话题
    这些是VIO系统三角化的3D特征点
```
#### 【参数2】imagePoints - 输入的2D投影
```
matched_2d_old_norm
    ↓
const vector<cv::Point2f> &matched_2d_old_norm

【含义】
    ├─ 【旧帧图像】中对应3D点的2D投影
    
    ├─ 单位：【归一化坐标】(不是像素！)
    ├─ 坐标范围：约 [-1, 1] (根据图像大小)
    ├─ 每个imagePoints[i]对应objectPoints[i]
    │
    └─ 【例】
       matched_2d_old_norm[0] = (0.1, 0.2)    第1个特征在旧帧的位置
       matched_2d_old_norm[1] = (-0.05, 0.15) 第2个特征在旧帧的位置
       ...
       matched_2d_old_norm[n] = (...)         第N个特征在旧帧的位置

【坐标系】
    【归一化坐标】 不是 【像素坐标】
    ├─ 像素坐标：(x_pixel, y_pixel) 
    │  例：(320, 240) - 图像中心
    │
    ├─ 转换到归一化坐标：
    │  x_norm = (x_pixel - cx) / fx
    │  y_norm = (y_pixel - cy) / fy
    │  其中 cx, cy 是主点，fx, fy 是焦距
    │
    └─ 优点：
       不依赖具体图像分辨率
       标准化便于计算

【为什么用归一化】
    相机内参中的 fx, fy 会在 K 矩阵中再次应用
    用归一化坐标避免重复应用
```
#### 【参数3】cameraMatrix - 相机内参矩阵K
```
K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

【结构】
    K = [fx  0  cx]     (3x3矩阵)
        [ 0 fy  cy]
        [ 0  0   1]

【含义】
    fx = 焦距 (x方向，像素单位)
    fy = 焦距 (y方向，像素单位)
    cx = 主点X坐标 (光心在图像中的x位置)
    cy = 主点Y坐标 (光心在图像中的y位置)

【代码中的值】
    K = [1.0  0   0]
        [ 0  1.0  0]
        [ 0   0   1]
    
    ❓ 为什么都是1.0？
    └─ 因为 imagePoints 是【归一化坐标】！
       如果是【像素坐标】就需要真实的内参
       但用了归一化坐标，所以K是单位矩阵

【投影模型】
    p_pixel = K * p_norm  (如果用像素坐标)
    p_norm = K^-1 * p_pixel (如果反向)
```
#### 【参数4】distCoeffs - 畸变系数D
```
cv::Mat D;   // 空矩阵（未初始化）

【含义】
    ├─ 相机的畸变参数
    ├─ 通常5个系数：[k1, k2, p1, p2, k3]
    │  k1, k2: 径向畸变系数
    │  p1, p2: 切向畸变系数
    │  k3: 第二径向畸变系数
    │
    └─ 代码中 D 是空的：
       └─ 表示【没有畸变】或【畸变已校正】

【为什么是空的】
    ├─ 从VIO接收的特征点已经经过畸变校正
    ├─ /keyframe_point 的点是已校正的
    └─ 所以不需要再次畸变校正
```
#### 【参数5】rvec - 旋转向量（输入+输出）
```
cv::Mat rvec;  // 旋转向量，3x1 矩阵

【输入】(初值)
    rvec = R_initial 当前帧【相机】相对于VINS坐标系的【反向】变换
    └─ 第214-222行计算的初值

【输出】(优化结果)
    rvec = PnP 优化后的旋转向量
    └─ 旧帧相机【相对于当前帧】的旋转。

【旋转向量格式】
    rvec = [rx, ry, rz]^T  (3x1列向量)
    
    物理含义：
    ├─ 方向：旋转轴方向
    ├─ 大小：旋转角度(弧度)
    └─ 例：rvec = [0.1, 0, 0] 表示绕X轴旋转0.1弧度

【转成旋转矩阵】
    cv::Rodrigues(rvec, R);  // rvec → R
    cv::Rodrigues(R, rvec);  // R → rvec
```
#### 【参数6】tvec - 平移向量（输入+输出）
```
cv::Mat t;  // 平移向量，3x1 矩阵

【输入】(初值)
    t = P_initial 当前帧【相机】相对于VINS坐标系的【反向】变换
    └─ 初始的平移估计

【输出】(优化结果)
    t = PnP 优化后的平移向量
    └─ 旧帧相机【相对于当前帧】的位移

【结构】
    t = [tx, ty, tz]^T  (3x1列向量)
    
    含义：
    ├─ tx = X方向位移 (米)
    ├─ ty = Y方向位移 (米)
    └─ tz = Z方向位移 (米)

【例】
    t = [0.5, -0.1, 3.2]^T
    └─ 旧帧相对当前帧偏移 (0.5m, -0.1m, 3.2m)
```
#### 【参数7】useExtrinsicGuess - 是否使用外参初值
```
true  // 使用初值

【含义】
    ├─ true:  使用提供的 rvec, tvec 作为初值
    │         从这个位置开始优化
    │
    └─ false: 从零初始化
             需要更多迭代才能收敛

【为什么设为true】
    ├─ 我们已经有了合理的初值
    │  (从当前帧的VIO位姿)
    ├─ 使用初值可以加速收敛
    ├─ 提高鲁棒性
    └─ 更容易找到全局最优解
```
#### 【参数8】iterationsCount - RANSAC迭代次数
```
100  // 进行100次迭代

【含义】
    ├─ RANSAC 算法的迭代次数
    ├─ 每次迭代随机选择4个点
    ├─ 用这4个点计算一个位姿假设
    ├─ 验证这个假设有多少inliers
    └─ 保留最好的假设

【迭代过程】
    迭代1: 随机选4个点 → 计算位姿 → 计数inliers
    迭代2: 随机选4个点 → 计算位姿 → 计数inliers
    ...
    迭代100: 随机选4个点 → 计算位姿 → 计数inliers
    
    最后：选择inliers最多的位姿

【为什么是100】
    ├─ 权衡速度和精度
    ├─ 100次足以找到好的内点集
    ├─ 太少可能错过最优解
    ├─ 太多浪费计算
    └─ 实验证明100次比较合适
```
#### 【参数9】reprojectionError - 重投影误差阈值
```
20.0 / 460.0  ≈ 0.0435  (归一化单位)

【含义】
    ├─ 点被认为是inlier的最大误差
    ├─ 在【归一化坐标】下测量
    ├─ 单位：【无单位的归一化坐标】
    │
    └─ 【从何而来】
       20.0 像素 / 460.0 焦距
       = 像素误差 / 焦距
       = 角度的正切值(近似)

【具体含义】
    如果某个点的重投影误差 < 0.0435
    └─ 这个点被认为是 inlier（可信点）
    
    如果某个点的重投影误差 ≥ 0.0435
    └─ 这个点被认为是 outlier（外点）

【重投影误差定义】
    error = || p_image - project(p_3d) ||
    
    其中：
    ├─ p_image = 在图像中检测到的2D位置
    ├─ p_3d = 3D点通过估计的位姿投影后
    └─ 误差 = 这两者的距离

【为什么是20像素】
    ├─ 相机焦距约460像素
    ├─ 20像素相当于视角约2.5°
    ├─ 这是一个合理的内点阈值
    ├─ 避免太严格(遗漏真实匹配)
    └─ 也避免太松散(接受外点)
```
#### 【参数10】confidence - 置信度
```
0.99  // 99% 置信度

【含义】
    ├─ 在这个置信度下，至少有一次迭代
    │  会找到一个纯内点集合(没有外点)
    ├─ 99% 表示"基本确定"能找到
    ├─ 100% 不现实(永不停止)
    └─ 太低(如50%)可能早停

【RANSAC的作用】
    根据confidence和当前inliers比例
    自动计算所需的迭代次数
    
    公式(简化)：
        如果当前inlier比例 = w
        N = log(1 - confidence) / log(1 - w^4)
    
    例：
    ├─ w = 0.7 (70% inlier)
    │  N = log(0.01) / log(1 - 0.7^4) ≈ 9次
    │
    ├─ w = 0.5 (50% inlier)
    │  N = log(0.01) / log(1 - 0.5^4) ≈ 75次
    │
    └─ w = 0.3 (30% inlier)
       N = log(0.01) / log(1 - 0.3^4) ≈ 300次

【为什么是0.99】
    ├─ 很保险
    ├─ 允许足够的迭代
    ├─ 99%确定能找到好结果
    └─ 通常的选择
```
#### 【参数11】inliers - 输出的内点标记
```
cv::Mat inliers;  // 输出参数

【含义】
    ├─ 一个整数向量，记录哪些点是inlier
    ├─ 长度 = matched_3d 的点数
    ├─ 值：0或1
    │
    └─ inliers[i] = 1  → matched_3d[i] 是内点
       inliers[i] = 0  → matched_3d[i] 是外点

【例】
    inliers = [1, 1, 0, 1, 0, 1, ..., 1]
    
    含义：
    ├─ 第1个点：内点 ✓
    ├─ 第2个点：内点 ✓
    ├─ 第3个点：外点 ✗
    ├─ 第4个点：内点 ✓
    ├─ 第5个点：外点 ✗
    ├─ 第6个点：内点 ✓
    └─ ...

【后续处理】(第274-277行)
    for (int i = 0; i < inliers.rows; i++) {
        int n = inliers.at<int>(i);
        status[n] = 1;  // 只保留inlier的点
    }
    
    然后 reduceVector 删除所有外点
```