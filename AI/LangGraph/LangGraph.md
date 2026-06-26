


## 三个核心概念(就这三个)

LangGraph 把 agent 建成一张**状态图(StateGraph)**:

- **State(状态)**:贯穿全图的共享数据,每个节点读它、改它、传给下一个——**记忆就靠它**
- **Node(节点)**:一个 Python 函数,干一件事(调 LLM、调工具、处理数据)
- **Edge(边)**:连接节点;**普通边**=直连,**条件边**=按条件决定走哪条(分支)

一句生活化比喻:做旅游助手 = 问目的地 → 查天气 → 推荐穿搭 → 生成行程,每步结果存进 state,下雨就跳去"提醒带伞"。**流程清晰、状态不丢、能循环能中断恢复**——这就是它比普通脚本强的地方。

## 推荐学习路径

**第 1 步 · 理解概念(中文,30 分钟)**

- [菜鸟教程 · LangGraph 入门](https://www.runoob.com/ai-agent/langgraph-quick-start.html) —— 零基础,从两节点线性流程讲起,把 State 机制讲透(尤其 `Annotated[list, add_messages]` 这个核心 reducer)

**第 2 步 · 官方上手(英文,跑通第一个 agent)**

- [LangGraph 官方 Quickstart](https://docs.langchain.com/oss/python/langgraph/quickstart) —— 手把手建一个 calculator agent,Graph API / Functional API 两种写法都演示

**第 3 步 · 系统实战(二选一)**

- 中文系统教程(基于 1.0):[dive-into-langgraph](https://github.com/luochang212/dive-into-langgraph) —— 14 章,提炼 LangChain+LangGraph 主要功能
- 英文动手项目:[Real Python · Build Stateful AI Agents](https://realpython.com/langgraph-python/) —— 做一个能解析/发邮件、调 API 的有状态 agent

**第 4 步 · 对我们最相关的两块(用到再看)**

- 多工具有状态智能体,**兼容本地模型(Qwen/Ollama,不依赖闭源)**:[Jimmy Song 教程](https://jimmysong.io/zh/book/ai-handbook/agent/langgraph/) —— 正好配你"模型无关、以后可能换"的需求
- 免费结构化课程:[LangChain Academy · Intro to LangGraph](https://academy.langchain.com/courses/intro-to-langgraph)

**提醒一句**:LangGraph 通常和 LangChain 配合用(LangChain 给基础组件/工具,LangGraph 管流程和状态),所以会顺带碰到一点 LangChain。





```
from typing import TypedDict, Annotated
from langgraph.graph import add_messages

class MyState(TypedDict):
    messages: Annotated[list, add_messages]  # 消息列表（自动追加）
    user_name: str                            # 用户名称
    step_count: int                           # 步骤计数
```
>定义一个状态 State。
  State 是一个字典。
  里面有一个 messages 字段。
  messages 用来保存对话消息。
  每次更新 messages 时，不覆盖旧消息，而是用 add_messages 追加合并。