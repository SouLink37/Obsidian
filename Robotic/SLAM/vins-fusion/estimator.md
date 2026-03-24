## 1 主优化器 (vins/src/estimator/) - 滑动窗口优化
优化变量：

位姿 (Pose): 7维 [tx, ty, tz, qx, qy, qz, qw]
速度和偏置 (SpeedBias): 9维 [vx, vy, vz, ba_x, ba_y, ba_z, bg_x, bg_y, bg_z]
相机-IMU外参 (Extrinsic): 7维
特征点逆深度 (Feature): 1维
时间偏移 (Td): 1维
代价函数：

### 1.1 IMUFactor (imu_factor.h)

残差维度: 15维
优化变量: 位姿i(7) + 速度偏置i(9) + 位姿j(7) + 速度偏置j(9)
作用: IMU预积分约束，连接相邻帧
### 1.2 ProjectionTwoFrameOneCamFactor (projectionTwoFrameOneCamFactor.h)

残差维度: 2维 (重投影误差)
优化变量: 位姿i(7) + 位姿j(7) + 外参(7) + 特征深度(1) + 时间偏移(1)
作用: 单目视觉约束，两帧观测同一特征点
### 1.3 ProjectionTwoFrameTwoCamFactor (projectionTwoFrameTwoCamFactor.h)

残差维度: 2维
优化变量: 位姿i(7) + 位姿j(7) + 外参0(7) + 外参1(7) + 特征深度(1) + 时间偏移(1)
作用: 双目视觉约束，两帧两相机
### 1.4 ProjectionOneFrameTwoCamFactor (projectionOneFrameTwoCamFactor.h)

残差维度: 2维
优化变量: 外参0(7) + 外参1(7) + 特征深度(1) + 时间偏移(1)
作用: 双目立体视觉约束，同一帧两相机
### 1.5 MarginalizationFactor (marginalization_factor.h)

残差维度: 动态
作用: 边缘化先验约束，保留旧帧信息
### 1.6 InitialBiasFactor (initial_bias_factor.h)

残差维度: 6维
优化变量: 速度偏置(9)
作用: 初始化时的偏置先验约束
### 1.7 InitialPoseFactor (initial_pose_factor.h)

残差维度: 6维
优化变量: 位姿(7)
作用: 初始化时的位姿先验约束
### 1.8 ZuptFactor (zupt_factor.h)

残差维度: 6维
优化变量: 速度偏置(9)
作用: 零速更新约束（静止检测）


# VINS-Fusion 滑动窗口优化详解


## 📐 滑动窗口结构

  

```

窗口大小: WINDOW_SIZE (通常为10帧)

状态变量数组: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] (共11帧)

```

  

每一帧包含：

- **位姿**: `Ps[i]`, `Rs[i]` (位置+旋转)

- **速度和偏置**: `Vs[i]`, `Bas[i]`, `Bgs[i]` (速度+加速度偏置+陀螺仪偏置)

- **IMU预积分**: `pre_integrations[i]` (从i-1到i的预积分)

- **特征点**: 由 `FeatureManager` 管理

  

---

  

## 🔄 完整运行流程

  

### 阶段1: 新帧到来 (processImage)

  

```cpp

1. 添加特征点 → 判断是否为关键帧

├─ 关键帧: marginalization_flag = MARGIN_OLD (边缘化最老帧)

└─ 非关键帧: marginalization_flag = MARGIN_SECOND_NEW (边缘化次新帧)

  

2. 关键帧判断标准 (addFeatureCheckParallax):

- 跟踪特征点数量 < 阈值 → 关键帧

- 平均视差 > 阈值 → 关键帧

- 否则 → 非关键帧

```

  

### 阶段2: 构建优化问题 (optimization)

  

```cpp

// 1. 添加参数块

for (int i = 0; i <= frame_count; i++) {

problem.AddParameterBlock(para_Pose[i], 7); // 位姿

problem.AddParameterBlock(para_SpeedBias[i], 9); // 速度+偏置

}

problem.AddParameterBlock(para_Ex_Pose[0], 7); // 外参

problem.AddParameterBlock(para_Td[0], 1); // 时间偏移

  

// 2. 添加边缘化先验 (来自上一次优化)

if (last_marginalization_info) {

MarginalizationFactor *marg_factor =

new MarginalizationFactor(last_marginalization_info);

problem.AddResidualBlock(marg_factor, ...);

}

  

// 3. 添加IMU约束 (连接相邻帧)

for (int i = 0; i < frame_count; i++) {

IMUFactor *imu_factor = new IMUFactor(pre_integrations[i+1]);

problem.AddResidualBlock(imu_factor, NULL,

para_Pose[i], para_SpeedBias[i],

para_Pose[i+1], para_SpeedBias[i+1]);

}

  

// 4. 添加视觉约束 (特征点重投影)

for (auto &feature : f_manager.feature) {

if (feature.used_num < 4) continue; // 至少4次观测

for (auto &observation : feature.observations) {

// 单目约束

ProjectionTwoFrameOneCamFactor *f = ...;

problem.AddResidualBlock(f, loss_function, ...);

// 双目约束 (如果有)

if (stereo) {

ProjectionTwoFrameTwoCamFactor *f = ...;

problem.AddResidualBlock(f, loss_function, ...);

}

}

}

  

// 5. 添加ZUPT约束 (零速更新，可选)

if (enable_zupt && detected_stationary) {

ZuptFactor *zupt = new ZuptFactor(zupt_data);

problem.AddResidualBlock(zupt, NULL, para_SpeedBias[frame_count-1]);

}

```

  

### 阶段3: 求解优化

  

```cpp

ceres::Solver::Options options;

options.linear_solver_type = ceres::DENSE_SCHUR; // 利用稀疏结构

options.trust_region_strategy_type = ceres::DOGLEG;

options.max_num_iterations = 8;

  

ceres::Solve(options, &problem, &summary);

```

  

**优化目标**:

```

min Σ ||r_prior||² + Σ ||r_imu||² + Σ ||r_visual||² + Σ ||r_zupt||²

边缘化先验 IMU约束 视觉约束 零速约束

```

  

### 阶段4: 边缘化 (Marginalization)

  

这是滑动窗口的核心！有两种策略：

  

#### 策略A: MARGIN_OLD (边缘化最老帧) - 关键帧

  

```

优化前窗口: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

↓ 边缘化第0帧

优化后窗口: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, new]

```

  

**步骤**:

```cpp

1. 收集与第0帧相关的所有约束:

- 上一次的边缘化先验 (如果第0帧在其中)

- IMU约束: pre_integrations[1] (连接第0帧和第1帧)

- 视觉约束: 所有从第0帧开始观测的特征点

  

2. 构建舒尔补 (Schur Complement):

将第0帧的状态边缘化掉，保留对其他帧的约束

原始系统: [H_00 H_01] [x_0] [b_0]

[H_10 H_11] [x_1] = [b_1]

边缘化后: H'_11 = H_11 - H_10 * H_00^(-1) * H_01

b'_1 = b_1 - H_10 * H_00^(-1) * b_0

→ 新的先验约束: ||H'_11 * x_1 - b'_1||²

  

3. 滑动窗口:

- 所有状态向前移动一位: Ps[i] = Ps[i+1]

- 删除第0帧的数据

- 在末尾添加新帧

```

  

#### 策略B: MARGIN_SECOND_NEW (边缘化次新帧) - 非关键帧

  

```

优化前窗口: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

↓ 边缘化第9帧

优化后窗口: [0, 1, 2, 3, 4, 5, 6, 7, 8, 10→9, new→10]

```

  

**原因**: 第10帧是非关键帧，信息量少，保留第10帧但删除第9帧

  

**步骤**:

```cpp

1. 如果第9帧在上一次边缘化先验中:

- 边缘化掉第9帧的状态

- 保留对其他帧的约束

  

2. 合并第9帧和第10帧的IMU预积分:

- pre_integrations[9] += pre_integrations[10]

3. 调整窗口:

- 第9帧位置 = 第10帧位置

- 删除第10帧

- 在末尾添加新帧

```

  

---

  

## 📊 约束关系图示


```

时刻: t0    t1     t2     t3     t4    ...   t10

     │      │      │      │      │           │

帧: [0]────[1]────[2]────[3]────[4]───...───[10]

     │      │      │      │      │            │

IMU: └──────┴──────┴──────┴──────┴─────...────┘

(预积分连接相邻帧)

  

视觉: ●──────●──────●──────● (特征点跨多帧观测)

      └──────┴──────┘

  

先验: [边缘化约束] ──→ 来自之前边缘化的帧

```

  

---

  

## 🎯 关键设计思想

  

### 1. 固定窗口大小

保持计算复杂度恒定 O(n³)，n=11

  

### 2. 边缘化保留信息

- 不是简单丢弃旧帧

- 通过舒尔补将旧帧的约束转化为先验

- 保持系统的可观性

  

### 3. 关键帧策略

- 关键帧: 边缘化最老帧 (MARGIN_OLD)

- 非关键帧: 边缘化次新帧 (MARGIN_SECOND_NEW)

- 保证窗口内都是信息丰富的帧

  

### 4. 多约束融合

```

总残差 = 边缘化先验 + IMU + 视觉 + ZUPT

(历史信息) (运动) (观测) (静止)

```

  

### 5. DENSE_SCHUR求解器

- 利用SLAM问题的稀疏结构

- 先消元特征点（大量但独立）

- 再求解位姿（少量但耦合）

  

---

  

## 💡 实际运行示例

  

```

初始化完成后:

帧0: 关键帧 → MARGIN_OLD → 边缘化帧0，窗口=[1,2,...,10,new]

帧1: 非关键帧 → MARGIN_SECOND_NEW → 边缘化帧9，窗口=[1,2,...,8,10,new]

帧2: 关键帧 → MARGIN_OLD → 边缘化帧1，窗口=[2,3,...,10,new]

...

```

  

**性能指标**:

- 每次优化耗时: 20-100ms (取决于特征点数量)

- 边缘化耗时: 5-20ms

- 总处理时间: 通常 < 150ms (满足实时性要求)

  

---

  

## 📝 代码位置

  

- **主优化函数**: `vins/src/estimator/estimator.cpp::optimization()`

- **边缘化实现**: `vins/src/factor/marginalization_factor.cpp`

- **滑动窗口管理**: `vins/src/estimator/estimator.cpp::slideWindow()`

- **关键帧判断**: `vins/src/featureTracker/feature_manager.cpp::addFeatureCheckParallax()`

  

---

  

## 🔍 深入理解边缘化

  

### 为什么需要边缘化？

  

1. **保持固定窗口**: 实时系统需要恒定的计算时间

2. **保留历史信息**: 直接丢弃会损失约束信息

3. **维持可观性**: 某些状态（如尺度）需要长时间观测

  

### 舒尔补的数学原理

  

给定线性系统：

```

[A B] [x] [a]

[C D] [y] = [b]

```

  

消去 x 后得到：

```

(D - C*A^(-1)*B) * y = b - C*A^(-1)*a

```

  

这就是边缘化的核心：将要删除的变量消去，保留其对剩余变量的影响。

  

### 边缘化的代价

  

- **线性化点固定**: 边缘化后的先验在固定线性化点计算，可能引入误差

- **稠密化**: 原本稀疏的Hessian矩阵会变稠密

- **计算开销**: 需要矩阵求逆和乘法运算

  