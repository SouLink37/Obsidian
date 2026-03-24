> Cartographer 中会用到两类 CSM：
> **Real-Time Correlative Scan Matching（局部）**
> **Fast Correlative Scan Matching（全局）**



## catographer中的2种 CSM
```
             Local SLAM (每帧)
IMU/odom prediction
        ↓
Real-Time Correlative Scan Matching  (可选)
        ↓
Ceres Scan Matcher (必须)
        ↓
Insert into Submap


             Global SLAM (低频)
Loop candidate
        ↓
Fast Correlative Scan Matching
        ↓
Ceres refine
        ↓
Add constraint → PoseGraph optimize
```

## fast CSM中的分支定界
暴力搜索时，我们需要遍历三个变量：角度，[横向平移](https://zhida.zhihu.com/search?content_id=162976472&content_type=Article&match_order=1&q=%E6%A8%AA%E5%90%91%E5%B9%B3%E7%A7%BB&zhida_source=entity)，纵向平移。现在我们把角度放在最外层，内层就只有两个平移，**这两个平移正是我们加速的对象**。

![[分支定界分辨率.png]]

>**低分辨率栅格的得分是对应所有高分辨率栅格得分的上界！**

![[分支定界.png]]

## ceres scan match 残差
> Cartographer 的 Ceres Scan Matcher 里主要有 **两类残差（residual）**：
   **(1) Scan-to-map 残差（点云匹配栅格地图）**
   **(2) Pose prior 残差（约束不要偏离预测位姿）**

Cartographer 2D Ceres ScanMatcher 的总代价：

$$

\min_{\mathbf{x}=(x,y,\theta)}

\left(

\sum_i \bigl(1 - P(T(\mathbf{x})p_i)\bigr)^2

- w_t ,|t - t_0|^2
    
- w_r ,|\theta - \theta_0|^2
    
    \right)
    
    $$

Correlative Scan Match VS ceres Scan Match：

|**方法**|**搜索方式**|**是否连续**|**是否梯度优化**|**精度**|
|---|---|---|---|---|
|Correlative SM|枚举候选位姿|❌离散|❌无梯度|粗|
|Ceres SM|非线性最小二乘优化|✅连续|✅梯度|精|
