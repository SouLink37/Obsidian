
### 核心概念

```
Problem = 参数块 + 残差块（Factor）
         ↓
      Solve()
         ↓
    优化后的参数
```

### 基本流程

```cpp
// ========== 1. 定义 Factor（CostFunction）==========
class MyFactor : public ceres::SizedCostFunction<2, 3, 4> {
//                                              残差维度, 参数1维度, 参数2维度
    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {
        // 计算残差
        // 计算雅可比（如果 jacobians != nullptr）
    }
};

// ========== 2. 创建 Problem ==========
ceres::Problem problem;

// ========== 3. 添加参数块（可选，通常隐式添加）==========
// 显式添加：需要设置参数化、固定参数、设置边界时
problem.AddParameterBlock(para_Pose, 7, new PoseLocalParameterization());
problem.SetParameterBlockConstant(para_Ex);  // 固定不优化

// ========== 4. 添加残差块 ==========
// 参数会随残差隐式添加到 problem
problem.AddResidualBlock(
    new MyFactor(...),    // cost_function (factor)
    new ceres::HuberLoss(1.0),  // loss_function (鲁棒核，可为 nullptr)
    para_block_1,         // 参数块指针
    para_block_2          // 同一参数可被多个残差引用
);

// 可以添加很多残差
problem.AddResidualBlock(factor1, loss, para_A, para_B);
problem.AddResidualBlock(factor2, loss, para_B, para_C);  // para_B 共享
problem.AddResidualBlock(factor3, nullptr, para_A, para_C);

// ========== 5. 配置求解器 ==========
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_SCHUR;
options.max_num_iterations = 10;
options.num_threads = 4;

// ========== 6. 求解 ==========
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
// 优化后参数直接写回原数组

// ========== 7. 可选：获取 Jacobian（诊断用）==========
ceres::CRSMatrix jacobian;
problem.Evaluate({}, nullptr, nullptr, nullptr, &jacobian);
```

### VINS 中的典型应用

```cpp

// 先显式添加（设置参数化）
for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    problem.AddParameterBlock(para_Pose[i], 7, PoseLocalParameterization());
    problem.AddParameterBlock(para_SpeedBias[i], 9);
}

// 滑动窗口中的多种 Factor
problem.AddResidualBlock(marginalization_factor, ...);  // 边缘化先验
problem.AddResidualBlock(imu_factor, nullptr, ...);     // IMU 约束
problem.AddResidualBlock(visual_factor, loss, ...);     // 视觉重投影
```

### 关键点

|项目|说明|
|---|---|
|Factor|继承 `CostFunction`，实现 `Evaluate()` 计算残差和雅可比|
|参数添加|隐式（随残差）或显式（需要特殊配置时）|
|参数共享|同一指针被多个残差引用，Ceres 自动识别|
|Solve|内部迭代调用 Evaluate，构建 H=J^TJ 求解|
|结果|直接写回参数数组，无需额外获取|



**Ceres 内部做的事：**

```
Solve() {
    while (!converged) {
        // 内部隐式调用 Evaluate
        ComputeResiduals();      // 计算 r
        ComputeJacobians();      // 计算 J
        BuildNormalEquations();  // H = J^T*J, b = J^T*r
        SolveLinearSystem();     // 求解 Δx
        UpdateParameters();      // x += Δx
    }
}
```

**只有特殊需求才显式调用 `Evaluate()`：**

- 获取最终的 Jacobian（你的条件数诊断）
- 获取最终的残差值
- 调试/分析目的
- 自定义优化算法

## **LocalParameterization**

>由于旋转在 SO(3) 上只有 3 个自由度，Ceres 在使用 local parameterization 时会在 3 维切空间里优化增量 δθ。若参数块中旋转用 4 维单位四元数存储，就需要定义 Plus(x, δ) 来把优化得到的 δ（**ceres优化的结果delta** 其中包含 δpose和 δθ）映射为四元数增量并更新四元数。

```cpp
bool PoseLocalParameterization::Plus(const double *x, const double *delta,

double *x_plus_delta) const {

Eigen::Map<const Eigen::Vector3d> _p(x);

Eigen::Map<const Eigen::Quaterniond> _q(x + 3);


Eigen::Map<const Eigen::Vector3d> dp(delta);


Eigen::Quaterniond dq =

Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));


Eigen::Map<Eigen::Vector3d> p(x_plus_delta);

Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  
p = _p + dp;

q = (_q * dq).normalized();

  
return true;

}
```

这个 `Plus` 函数是 Ceres 的 **LocalParameterization**（在新版本叫 Manifold）的核心接口，用于定义如何将优化增量 `delta` 应用到当前参数 `x` 上。

让我解释一下为什么需要这个：

**问题背景：**

- 位姿参数块是 7 维：`[px, py, pz, qx, qy, qz, qw]`（3D位置 + 四元数）
- 但四元数有约束 `||q|| = 1`，实际自由度只有 3（旋转的 SO(3) 流形）
- 所以优化的增量 `delta` 是 6 维：`[dp_x, dp_y, dp_z, dθ_x, dθ_y, dθ_z]`

**Plus 函数做的事情：**

```cpp
// 1. 位置部分：直接加法（欧氏空间）
p = _p + dp;  // 3维 + 3维

// 2. 旋转部分：流形上的更新
// delta+3 是 3 维的旋转增量（轴角/李代数形式）
Eigen::Quaterniond dq = Utility::deltaQ(...);  // 转成四元数
q = (_q * dq).normalized();  // 四元数乘法 + 归一化
```

**关键点：**

- `deltaQ` 把 3 维旋转增量（小角度近似）转成四元数：`dq ≈ [0.5*dθ, 1]`
- 四元数更新用乘法而不是加法，这是 SO(3) 流形的正确操作
- 最后 `normalized()` 确保结果仍然是单位四元数

**维度对应：**

|参数块 x|7 维|`[p(3), q(4)]`|
|---|---|---|
|增量 delta|6 维|`[dp(3), dθ(3)]`|
|更新后 x_plus_delta|7 维|`[p+dp, q⊗dq]`|

这就是所谓的 **over-parameterization**：用 7 个数表示 6 个自由度，通过 LocalParameterization 告诉 Ceres 如何正确更新。