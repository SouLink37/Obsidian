
## 条件数计算方法详解

### 1. 整体流程

```
┌─────────────────────────────────────────────────────────────┐
│                    Ceres Problem                            │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│  │IMU Factor│  │IMU Factor│  │Visual   │  │Visual   │  ...  │
│  │    #1    │  │    #2    │  │Factor #1│  │Factor #2│        │
│  └─────────┘  └─────────┘  └─────────┘  └─────────┘        │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │  对每个因子分别计算：   │
              │  1. 获取雅可比 J        │
              │  2. 计算 H = J^T * J    │
              │  3. 计算条件数 κ        │
              └────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │  统计汇总：             │
              │  - 平均条件数           │
              │  - 最大/最小条件数      │
              │  - Good/Fair/Poor 分布  │
              └────────────────────────┘
```

### 2. 单个因子的条件数计算

以一个 **IMU 因子** 为例：

```cpp
// IMU 因子的残差维度：15 (3 位置 + 3 速度 + 3 旋转 + 3 ba + 3 bg)
// IMU 因子的参数：
//   - para_Pose[i]:      7 维 (位置 3 + 四元数 4) → 切空间 6 维
//   - para_SpeedBias[i]: 9 维 (速度 3 + ba 3 + bg 3)
//   - para_Pose[j]:      7 维 → 切空间 6 维
//   - para_SpeedBias[j]: 9 维
// 总参数维度：6 + 9 + 6 + 9 = 30 维

// 雅可比矩阵 J: 15 × 30
J = [J_pose_i | J_sb_i | J_pose_j | J_sb_j]
    ← 6 →    ← 9 →   ← 6 →    ← 9 →

// 海森矩阵 H = J^T * J: 30 × 30
H = J^T * J

// 条件数 = 最大特征值 / 最小特征值
κ = λ_max / λ_min
```

### 3. 代码实现

```cpp
// 1. 从 Ceres 获取因子的雅可比
const ceres::CostFunction* cost_function = problem.GetCostFunctionForResidualBlock(rid);
cost_function->Evaluate(parameter_blocks.data(), residuals.data(), jacobians.data());

// 2. 构建完整的雅可比矩阵 J
Eigen::MatrixXd J(num_residuals, total_param_dim);
// ... 填充 J ...

// 3. 计算海森矩阵
Eigen::MatrixXd H = J.transpose() * J;

// 4. 计算条件数（使用特征值分解）
Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(H);
Eigen::VectorXd eigenvalues = solver.eigenvalues();

double lambda_max = eigenvalues.maxCoeff();
double lambda_min = eigenvalues.minCoeff();

// 阈值保护
const double eps = 1e-10 * lambda_max;
if (lambda_min < eps) lambda_min = eps;

double kappa = lambda_max / lambda_min;
```

### 4. 条件数的含义

|条件数 κ|状态|含义|
|---|---|---|
|κ < 10⁴|✓ Good|数值稳定，优化收敛良好|
|10⁴ ≤ κ < 10⁶|⚠️ Fair|可能有轻微数值问题|
|κ ≥ 10⁶|❌ Poor|数值不稳定，可能导致优化发散|

### 5. 为什么分别计算 IMU 和 Visual？

**整体 Problem 的条件数** 反映的是所有因子耦合后的情况，但无法定位问题来源。

**分别计算** 可以帮助你：

- 判断是 IMU 参数（如 bias）还是 Visual 参数（如特征点深度）导致的病态
- 针对性地调整参数或添加正则化

### 6. 数学关系

```
Jacobian 条件数:  κ(J) = σ_max / σ_min  (奇异值)
Hessian 条件数:   κ(H) = λ_max / λ_min  (特征值)

关系: κ(H) = κ(J)²

因为: H = J^T * J
      λ(H) = σ(J)²
```