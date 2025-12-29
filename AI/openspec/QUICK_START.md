# OpenSpec 快速入门指南

## ✅ 已完成的工作

### 1. 项目上下文已填充
- ✅ `openspec/project.md` 已完整填写
  - 项目目的：基于ROS2的机器人控制系统
  - 技术栈：C++17, ROS2 Iron, CMake, colcon
  - 代码规范、架构模式、测试策略等

### 2. 第一个变更提案已创建
- ✅ Change ID: `increase-mqtt-keepalive-timeout`
- ✅ 状态：待批准
- ✅ 文件：
  - `proposal.md` - 提案说明
  - `tasks.md` - 实施清单
  - `specs/mqtt-connection/spec.md` - 需求定义
  - `REVIEW.md` - 审查指南

## 📋 OpenSpec 三阶段工作流程

### Stage 1: 创建变更提案 (Creating Changes)

**何时创建提案：**
- 添加新功能
- 做破坏性变更（API、架构）
- 改变架构模式
- 性能优化（改变行为）
- 更新安全模式

**不需要提案的情况：**
- Bug修复（恢复预期行为）
- 拼写错误、格式、注释
- 非破坏性依赖更新
- 配置变更
- 现有行为的测试

**创建流程：**
1. 查看当前状态：`openspec list`, `openspec list --specs`
2. 选择唯一的 `change-id`（kebab-case，动词开头）
3. 创建文件：
   - `proposal.md` - 为什么、改变什么、影响
   - `tasks.md` - 实施清单
   - `design.md` - 技术决策（可选，仅在需要时）
   - `specs/<capability>/spec.md` - 需求定义
4. 验证：`openspec validate <id> --strict`
5. 等待批准

**示例命令：**
```bash
# 查看现有变更
openspec list

# 查看现有规范
openspec list --specs

# 查看变更详情
openspec show increase-mqtt-keepalive-timeout

# 验证变更
openspec validate increase-mqtt-keepalive-timeout --strict
```

### Stage 2: 实施变更 (Implementing Changes)

**重要：必须先获得批准！**

**实施步骤：**
1. 阅读 `proposal.md` - 理解要构建什么
2. 阅读 `design.md`（如果存在）- 审查技术决策
3. 阅读 `tasks.md` - 获取实施清单
4. 按顺序实施任务
5. 完成所有任务后，更新 `tasks.md` 中的复选框为 `- [x]`
6. 验证和测试

**当前提案的实施清单：**
- [ ] 1.1 更新 `MqttBroker.h` 中的 `TIMEOUT` 常量从 30 改为 60
- [ ] 1.2 更新 `MqttConnectionMosquitto.hpp` 中构造函数默认 `keepAlive` 参数从 30 改为 60
- [ ] 1.3 检查所有使用 `MqttConnectionMosquitto` 的地方
- [ ] 2.1-2.4 验证和测试
- [ ] 3.1 更新文档

### Stage 3: 归档变更 (Archiving Changes)

**部署后归档：**
```bash
# 归档变更（会自动移动到 archive/ 目录）
openspec archive increase-mqtt-keepalive-timeout --yes

# 如果只是工具变更，不更新规范
openspec archive <change-id> --skip-specs --yes
```

## 🤝 如何与我协作

### 创建新提案时
你可以说：
- "我想添加 [功能]。请创建一个 OpenSpec 变更提案"
- "帮我创建一个变更提案，用于 [描述]"
- "我需要调整 [功能]，请创建提案"

我会：
1. 探索代码库，了解当前实现
2. 创建提案文件结构
3. 编写需求定义和场景
4. 验证提案格式

### 实施变更时
你可以说：
- "开始实施 increase-mqtt-keepalive-timeout"
- "帮我完成 tasks.md 中的任务 1.1"
- "这个提案已批准，开始实施"

我会：
1. 按照 tasks.md 逐步实施
2. 更新代码
3. 更新任务清单
4. 协助测试和验证

### 查看和审查时
你可以说：
- "显示当前所有变更"
- "查看提案详情"
- "验证这个提案"

我会：
1. 运行相应的 openspec 命令
2. 解释结果
3. 帮助修复问题

## 📚 常用命令速查

```bash
# 列出所有变更
openspec list

# 列出所有规范
openspec list --specs

# 查看变更详情
openspec show <change-id>

# 查看变更的 JSON 格式（用于调试）
openspec show <change-id> --json --deltas-only

# 验证变更
openspec validate <change-id> --strict

# 归档变更（部署后）
openspec archive <change-id> --yes
```

## 🎯 当前状态

**活跃变更：**
- `increase-mqtt-keepalive-timeout` - 待批准

**下一步：**
1. 等待团队审查和批准提案
2. 批准后，按照 `tasks.md` 开始实施
3. 完成实施后归档变更

## 💡 提示

1. **总是先检查现有工作**：使用 `openspec list` 查看是否有相关变更
2. **保持提案范围清晰**：一个提案专注于一个能力或功能
3. **使用描述性的 change-id**：kebab-case，动词开头（如 `add-`, `update-`, `remove-`）
4. **每个需求必须有场景**：使用 `#### Scenario:` 格式
5. **验证很重要**：始终运行 `--strict` 验证

## ❓ 常见问题

**Q: 什么时候需要 design.md？**
A: 当变更涉及多个系统、新架构模式、新外部依赖、安全/性能复杂性，或需要技术决策讨论时。

**Q: 如何知道提案是否已批准？**
A: 通常通过团队审查流程。批准后可以开始实施。

**Q: 可以同时有多个活跃变更吗？**
A: 可以，但要确保它们不冲突。使用 `openspec list` 查看所有活跃变更。

