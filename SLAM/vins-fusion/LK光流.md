
让我查看光流追踪的实现代码：

[7 tools called]

现在我来详细总结 VINS Fusion 的光流追踪流程：

---

# VINS Fusion 光流追踪详解

## 一、整体流程图

> **VINS 不对图像去畸变，只对特征点坐标去畸变**

```
┌─────────────────────────────────────────────────────────────────────┐
│                    trackImage() 完整流程                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Step 1: 帧间光流追踪（畸变图像上）                                  │
│          prev_img (畸变) → cur_img (畸变)                           │
│          prev_pts (畸变) → cur_pts (畸变)                           │
│                              ↓                                      │
│  Step 2: 反向光流验证（畸变图像上）                                  │
│                              ↓                                      │
│  Step 3: Mask + 新特征检测（畸变图像上）                             │
│                              ↓                                      │
│  Step 4: 左目点去畸变                                               │
│          cur_pts (畸变) → cur_un_pts (归一化)   ← Line 436          │
│                              ↓                                      │
│  Step 5: 左目速度计算（用归一化坐标）                                │
│          pts_velocity = (cur_un_pts - prev_un_pts) / dt             │
│                              ↓                                      │
│  Step 6: 双目光流追踪（畸变图像上）                                  │
│          cur_img (畸变) → rightImg (畸变)                           │
│          cur_pts (畸变) → cur_right_pts (畸变)                      │
│                              ↓                                      │
│  Step 7: 右目点去畸变  ← Line 548                                   │
│          cur_right_pts (畸变) → cur_un_right_pts (归一化)           │
│                              ↓                                      │
│  Step 8: 右目速度计算（用归一化坐标）← Line 549-552                  │
│          right_pts_velocity = (cur_un_right_pts - prev) / dt        │
│                              ↓                                      │
│  输出: featureFrame                                                 │
│        左目: (cur_un_pts, cur_pts, pts_velocity)                    │
│        右目: (cur_un_right_pts, cur_right_pts, right_pts_velocity)  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---
代码示例
```
// 左目去畸变
cur_un_pts = undistortedPts(cur_pts, m_camera[0]);  
pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
  
// 左目输出
for (size_t i = 0; i < ids.size(); i++) {

x = cur_un_pts[i].x; // 归一化x（去畸变后）
y = cur_un_pts[i].y; // 归一化y（去畸变后）
p_u = cur_pts[i].x; // 像素u（畸变）
p_v = cur_pts[i].y; // 像素v（畸变）
velocity_x = pts_velocity[i].x; // 归一化速度
velocity_y = pts_velocity[i].y;
}
```

## 二、Step 1: 帧间光流追踪

### 核心代码

```184:219:vins/src/featureTracker/feature_tracker.cpp
      if (hasPrediction) {
        cur_pts = predict_pts;  // 使用IMU预测的位置作为初始值
        cv::calcOpticalFlowPyrLK(
            prev_img,
            cur_img,
            prev_pts,
            cur_pts,
            status,
            err,
            cv::Size(21, 21),   // 窗口大小
            1,                   // 金字塔层数（有预测时用1层）
            cv::TermCriteria(..., 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);  // 使用初始值

        // 如果成功追踪的点太少，重新用3层金字塔追踪
        if (succ_num < 10)
          cv::calcOpticalFlowPyrLK(..., 3);  // 3层金字塔
      } else
        cv::calcOpticalFlowPyrLK(..., 3);    // 无预测，直接3层
```

### 参数说明

| 参数 | 值 | 说明 |
|-----|-----|-----|
| 窗口大小 | 21×21 | LK光流搜索窗口 |
| 金字塔层数 | 1或3 | 有IMU预测用1层，否则3层 |
| 迭代次数 | 30 | 最大迭代次数 |
| 精度阈值 | 0.01 | 收敛阈值 |

---

## 三、Step 2: 反向光流验证 (Flow Back)

```221:244:vins/src/featureTracker/feature_tracker.cpp
      if (params_->flow_back) {
        vector<uchar> reverse_status;
        vector<cv::Point2f> reverse_pts = prev_pts;
        
        // 反向追踪: cur → prev
        cv::calcOpticalFlowPyrLK(
            cur_img,
            prev_img,
            cur_pts,
            reverse_pts,
            reverse_status, ...);
        
        // 验证: 正向+反向都成功，且距离 <= 0.5像素
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && reverse_status[i] &&
              distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
            status[i] = 1;  // 保留
          } else
            status[i] = 0;  // 剔除
        }
      }
```

### 原理图

```
正向追踪:  prev_pts ──────────────────→ cur_pts
                        光流

反向追踪:  reverse_pts ←────────────── cur_pts
                        光流

验证: |prev_pts - reverse_pts| <= 0.5 pixel ?
      YES → 追踪可靠
      NO  → 剔除（可能是遮挡、运动模糊等）
```

---

## 四、Step 3: Mask设置 + 新特征检测

### setMask() - 特征间距控制

```110:136:vins/src/featureTracker/feature_tracker.cpp
  // 按追踪次数排序（长时间追踪的优先保留）
  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;  // 追踪次数多的排前面
       });

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      // 在已选特征周围画圆屏蔽，半径=min_dist
      cv::circle(mask, it.second.first, params_->min_dist, 0, -1);
    }
  }
```

### goodFeaturesToTrack() - 新特征检测

```378:383:vins/src/featureTracker/feature_tracker.cpp
        cv::goodFeaturesToTrack(cur_img,
                                n_pts,
                                params_->max_cnt - cur_pts.size(),  // 需要补充的数量
                                0.01,                               // 质量阈值
                                params_->min_dist,                  // 最小间距
                                mask);                              // 屏蔽已有特征区域
```

### 图解

```
┌─────────────────────────────────────────┐
│              原始图像                    │
│                                         │
│    ●  已追踪特征 (track_cnt=15)          │
│        ╲                                │
│         ╲ 画圆屏蔽 (半径=min_dist)       │
│          ●────────────────●             │
│              已追踪特征 (track_cnt=8)    │
│                                         │
│       ○ 新检测特征 (goodFeaturesToTrack)│
│                                         │
│   优先级: track_cnt大的优先保留          │
│                                         │
└─────────────────────────────────────────┘
```

---

## 五、Step 4: 去畸变

```736:746:vins/src/featureTracker/feature_tracker.cpp
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts,
                                                   camodocal::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);  // 像素坐标
    Eigen::Vector3d b;
    cam->liftProjective(a, b);              // 去畸变 + 反投影到归一化平面
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));  // 归一化
  }
  return un_pts;
}
```

### 坐标转换

```
像素坐标 (u, v)
      ↓ liftProjective()
      ↓ (去畸变 + 反投影)
归一化平面坐标 (x, y, 1)
      ↓ 归一化
输出 (x/z, y/z) = (X/Z, Y/Z) — 相机坐标系下的方向向量
```

---

## 六、Step 5: 计算特征点速度

```748:778:vins/src/featureTracker/feature_tracker.cpp
vector<cv::Point2f>
FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                            map<int, cv::Point2f> &cur_id_pts,
                            map<int, cv::Point2f> &prev_id_pts) {
  vector<cv::Point2f> pts_velocity;
  
  if (!prev_id_pts.empty()) {
    double dt = cur_time - prev_time;  // 帧间时间差

    for (unsigned int i = 0; i < pts.size(); i++) {
      auto it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        // 速度 = 位移 / 时间
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.push_back(cv::Point2f(v_x, v_y));
      } else
        pts_velocity.push_back(cv::Point2f(0, 0));  // 新特征速度为0
    }
  }
  return pts_velocity;
}
```

**用途**：速度用于优化中的[[td优化  | 时间偏移补偿（td矫正）]]。

---

## 七、Step 6: 双目匹配

```454:478:vins/src/featureTracker/feature_tracker.cpp
        // 左 → 右 光流
        cv::calcOpticalFlowPyrLK(cur_img,
                                 rightImg,
                                 cur_pts,
                                 cur_right_pts,
                                 status, err,
                                 cv::Size(21, 21), 3);
        
        // 反向验证: 右 → 左
        if (params_->flow_back) {
          cv::calcOpticalFlowPyrLK(rightImg,
                                   cur_img,
                                   cur_right_pts,
                                   reverseLeftPts,
                                   statusRightLeft, err,
                                   cv::Size(21, 21), 3);
          for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
                distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
              status[i] = 1;
            else
              status[i] = 0;
          }
        }
```

**注意**：双目匹配用**光流**而非传统的**描述子匹配**，因为双目相机基线短，左右图像相似度高，光流更快。

---

## 八、输出数据格式

```570:588:vins/src/featureTracker/feature_tracker.cpp
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  for (size_t i = 0; i < ids.size(); i++) {
    int feature_id = ids[i];
    double x, y, z;
    x = cur_un_pts[i].x;    // 归一化x
    y = cur_un_pts[i].y;    // 归一化y
    z = 1;                  // 归一化z=1
    double p_u = cur_pts[i].x;  // 像素u
    double p_v = cur_pts[i].y;  // 像素v
    double velocity_x = pts_velocity[i].x;
    double velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }
```

### 输出结构

```
featureFrame: map<feature_id, vector<pair<camera_id, data>>>

data (7维向量):
  [0] x     - 归一化平面x (去畸变后)
  [1] y     - 归一化平面y (去畸变后)
  [2] z     - 1 (归一化)
  [3] p_u   - 像素坐标u
  [4] p_v   - 像素坐标v
  [5] vx    - x方向速度 (归一化平面)
  [6] vy    - y方向速度 (归一化平面)

双目时每个feature_id有两个条目:
  - camera_id=0 (左相机)
  - camera_id=1 (右相机)
```

---

## 九、关键参数总结

| 参数 | 配置项 | 说明 |
|-----|-------|-----|
| `max_cnt` | 150~200 | 最大特征点数 |
| `min_dist` | 20~30 | 特征最小间距(像素) |
| `flow_back` | 1 | 是否启用反向光流验证 |
| `use_gpu_acc_flow` | 0/1 | 是否用GPU加速光流 |

---

## 十、一句话总结

> **VINS光流追踪的核心是：LK金字塔光流追踪 + 反向验证剔除外点 + Mask控制特征间距 + Shi-Tomasi检测补充新特征 + 去畸变投影到归一化平面。**


##  十一、第一帧处理流程
```
┌─────────────────────────────────────────────────────────────────┐
│                    第一帧处理流程                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  if (prev_pts.size() > 0)  →  FALSE，跳过光流追踪               │
│                                                                 │
│  setMask()  →  mask全白（没有已有特征需要屏蔽）                  │
│                                                                 │
│  goodFeaturesToTrack()  →  检测 Shi-Tomasi 角点                 │
│     └── 这就是初始特征点！                                       │
│                                                                 │
│  双目匹配: cur_left_pts → cur_right_pts                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```