## 当前条件数计算方法

### 1. 整体流程

```
┌─────────────────────────────────────────────────────────────────┐
│                     ceres::Solve() 完成后                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  1. problem.GetResidualBlocks(&all_residuals)                   │
│     获取 Ceres 内部的 residual block 顺序                        │
│                                                                  │
│  2. 遍历 all_residuals 建立行索引映射：                          │
│     - imu_rows: IMU 因子对应的行号                               │
│     - visual_rows: Visual 因子对应的行号                         │
│     - 其他因子（Marg, ZUPT）自动跳过                             │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  problem.Evaluate(options, jacobian=&J_crs)                     │
│                                                                  │
│  options:                                                        │
│    - apply_loss_function = true  (应用鲁棒核 reweighting)       │
│    - residual_blocks = all_residuals  (显式指定顺序！)          │
│                                                                  │
│  返回 CRS 格式的稀疏 Jacobian：                                  │
│  ✓ 已经是 local parameterization（pose 6维，不是7维）           │
│  ✓ 已经应用 loss function reweighting                           │
│  ✓ 行顺序与 all_residuals 一致（因为显式指定了）                │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│              CRS → Eigen::SparseMatrix<double> J                │
│                      (保持稀疏，不转 dense)                      │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│              构建稀疏 Hessian: H = J^T × J                       │
│                                                                  │
│  J ∈ R^{m×n}  (m=残差数, n=参数维度)                            │
│  H ∈ R^{n×n}  (n×n 比 m×n 小很多，更快)                         │
└─────────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┴───────────────┐
                ▼                               ▼
┌─────────────────────────┐     ┌─────────────────────────┐
│   幂迭代求 λ_max        │     │   逆幂迭代求 λ_min      │
│   (~30 次迭代)          │     │   (~30 次迭代)          │
└─────────────────────────┘     └─────────────────────────┘
                │                               │
                └───────────────┬───────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    cond(H) = λ_max / λ_min                      │
└─────────────────────────────────────────────────────────────────┘
```

---

### 2. 行顺序保证机制（关键！）

**问题**：`problem.Evaluate()` 返回的 Jacobian 行顺序**不保证**按 `AddResidualBlock()` 调用顺序。

**解决方案**：

```cpp
// 1. 获取 Ceres 内部的 residual block 顺序
std::vector<ceres::ResidualBlockId> all_residuals;
problem.GetResidualBlocks(&all_residuals);

// 2. 用 set 快速查找 IMU/Visual ID
std::set<ResidualBlockId> imu_set(imu_ids.begin(), imu_ids.end());
std::set<ResidualBlockId> visual_set(visual_ids.begin(), visual_ids.end());

// 3. 遍历 all_residuals 建立行索引映射（在 Evaluate 之前！）
int row_offset = 0;
for (const auto& rid : all_residuals) {
    int n_res = problem.GetCostFunctionForResidualBlock(rid)->num_residuals();
    if (imu_set.count(rid)) {
        for (int r = 0; r < n_res; ++r) imu_rows.push_back(row_offset + r);
    } else if (visual_set.count(rid)) {
        for (int r = 0; r < n_res; ++r) visual_rows.push_back(row_offset + r);
    }
    // 其他因子（Marg, ZUPT）自动跳过
    row_offset += n_res;
}

// 4. 显式指定 residual_blocks 顺序
eval_options.residual_blocks = all_residuals;  // ← 关键！
problem.Evaluate(eval_options, ..., &jacobian_crs);
```

这样 Evaluate 返回的行顺序就与我们的映射一致。

---

### 3. 数学原理

**条件数定义：**

```
cond(H) = λ_max / λ_min

其中 H = J^T J 是 Hessian 矩阵（正规方程的系数矩阵）
```

**与 Jacobian 条件数的关系：**

```
H 的特征值 λ = J 的奇异值 σ 的平方

所以: cond(H) = cond(J)²
```

---

### 4. 幂迭代法求最大特征值

**原理：** 任意向量经过矩阵多次乘法后，会收敛到最大特征值对应的特征向量方向。

```cpp
double powerIterationMax(const SparseMatrix& H, VectorXd& v, int max_iter) {
    // 1. 随机初始化
    v = VectorXd::Random(n);
    v.normalize();
    
    double lambda = 0;
    for (int i = 0; i < max_iter; ++i) {
        // 2. 矩阵-向量乘法
        VectorXd y = H * v;
        
        // 3. Rayleigh 商 = 特征值估计
        double lambda_new = v.dot(y);
        
        // 4. 归一化
        v = y / y.norm();
        
        // 5. 收敛检查
        if (|lambda_new - lambda| < tol) break;
        lambda = lambda_new;
    }
    return lambda;  // 收敛到 λ_max
}
```

**复杂度：** O(iter × nnz)，其中 nnz 是 H 的非零元素数

---

### 5. 逆幂迭代法求最小特征值

**原理：** 对 H⁻¹ 做幂迭代，收敛到 H 的最小特征值。

```cpp
double inverseIterationMin(const SparseMatrix& H, VectorXd& v, int max_iter) {
    // 1. 预分解 H（只做一次）
    SparseLU<SparseMatrix> solver;
    solver.compute(H);
    
    // 2. 随机初始化
    v = VectorXd::Random(n);
    v.normalize();
    
    double lambda = 0;
    for (int i = 0; i < max_iter; ++i) {
        // 3. 求解 H * y = v（等价于 y = H⁻¹ * v）
        VectorXd y = solver.solve(v);
        
        // 4. 归一化
        v = y / y.norm();
        
        // 5. 计算最小特征值
        lambda = 1.0 / v.dot(H * v);
    }
    return lambda;  // 收敛到 λ_min
}
```

**复杂度：** O(iter × solve)，其中 solve 是稀疏 LU 求解的复杂度

---

### 6. 子系统分析（IMU / Visual）

**方法：根据 ResidualBlockId 从全局 Jacobian 按行提取子矩阵**

```
全局 Jacobian J (m × n):
┌─────────────────────────┐
│  Marg 残差行            │ ← 不在 imu_set 也不在 visual_set（跳过）
├─────────────────────────┤
│  IMU 残差行 (150 行)    │ ← 在 imu_set 中 → imu_rows
├─────────────────────────┤
│  ZUPT 残差行            │ ← 不在任何 set 中（跳过）
├─────────────────────────┤
│  Visual 残差行 (1000行) │ ← 在 visual_set 中 → visual_rows
└─────────────────────────┘

H_imu = J_imu^T × J_imu
H_visual = J_visual^T × J_visual

分别计算 cond(H_imu) 和 cond(H_visual)
```

**注意**：

- 行顺序由 `GetResidualBlocks()` 返回的顺序决定，不是 `AddResidualBlock()` 调用顺序
- ZUPT 因子会被自动跳过（不影响 IMU/Visual 分析）
- 整体条件数（Overall）仍然包含所有因子

**为什么这样做正确：**

- 列维度保持一致（都是 n 维参数空间）
- 不同因子约束不同参数，但在同一参数空间
- 可以看出哪类因子导致条件数恶化

---

### 7. 性能对比

|方法|复杂度|1234×356 矩阵耗时|
|---|---|---|
|Full SVD|O(mn²)|~2500 ms|
|Full 特征值分解|O(n³)|~500 ms|
|**幂迭代 + 逆幂迭代**|O(k × nnz)|**~10 ms**|

---

### 8. 输出指标解释

|指标|含义|健康范围|
|---|---|---|
|`λ_max`|最大特征值，最强约束方向|-|
|`λ_min`|最小特征值，最弱约束方向|> 1e-6|
|`cond(H)`|条件数 = λ_max/λ_min|< 1e10|
|`smallest_vector`|最小特征值对应的特征向量|退化方向|

**状态判断：**

```
cond(H) < 1e6   → ✓ Good     (数值稳定)
cond(H) < 1e8   → ⚠️ OK      (轻微问题)
cond(H) < 1e10  → ⚠️ Fair    (需要关注)
cond(H) ≥ 1e10  → ❌ Poor    (数值不稳定)
```

---

### 9. 退化方向分析

最小特征值对应的特征向量 `v_min` 指示**哪个参数方向不可观**：

```
v_min = [v_pose, v_vel, v_bias, v_features, v_ex, v_td]

如果 v_pose 分量大 → pose 退化（yaw/z/scale 不可观）
如果 v_features 分量大 → 特征点深度退化
```

这对于诊断 VINS 漂移非常有用！

---

### 10. 输出示例

```
========== Condition Number Diagnostics ==========
Frame: 10, Marg: NEW

[Overall] J:1234x356, cond(H)=1.2e+08, λ_max=1.0e+14, λ_min=1.0e+06 ⚠️  Fair
[IMU] 150 residuals, cond(H)=2.3e+05 ✓ Good
[Visual] 1000 residuals, cond(H)=8.7e+09 ❌ Poor
⚠️  High condition number - potential degeneration!
   Degenerate direction (first 6): [0.01, 0.02, 0.98, 0.00, 0.00, 0.00]
==================================================
[Hessian Diagnostics] Total analysis time: 15.2 ms
```