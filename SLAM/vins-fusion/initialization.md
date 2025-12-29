## 📊 双目vs单目初始化对比

| 阶段         | 单目+IMU                 | 双目+IMU                 | 双目                     |
| ---------- | ---------------------- | ---------------------- | ---------------------- |
| IMU激励检查    | ✅ initialStructure()   | ❌ 无                    | ❌ 无                    |
| 相对位姿估计     | ✅ relativePose()       | ❌ 无                    | ❌ 无                    |
| Global SFM | ✅ construct()          | ❌ 无                    | ❌ 无                    |
| PnP定位      | ❌ 无                    | ✅ initFramePoseByPnP() | ✅ initFramePoseByPnP() |
| 三角化        | ✅ SFM中进行               | ✅ triangulate()        | ✅ triangulate()        |
| 陀螺仪偏差      | ✅ solveGyroscopeBias() | ✅ solveGyroscopeBias() | ❌ 无                    |
| 视觉IMU对齐    | ✅ visualInitialAlign() | ❌ 无                    | ❌ 无                    |

https://zhuanlan.zhihu.com/p/412877911

![[vinsfusion-初始化1.png]]
![[vinsfusion-初始化2.png]]