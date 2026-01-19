在 Gin 框架中，**中间件（Middleware）** 是一个非常核心的概念。它本质上是一个在 HTTP 请求被处理（到达业务逻辑）**之前**或**之后**执行的函数。

你可以把中间件想象成一个**关卡**或**滤网**。请求像流水一样经过这些关卡，每个关卡都可以对请求进行检查、拦截或加工。

---

### 1. 中间件的定义

在 Gin 中，中间件的函数签名必须是 `gin.HandlerFunc`：

Go

```
func MyMiddleware() gin.HandlerFunc {
    return func(c *gin.Context) {
        // 【第一部分】：请求到达业务逻辑前执行
        log.Println("等待请求进入...")

        c.Next() // 【核心】：执行后续的中间件或业务逻辑

        // 【第二部分】：业务逻辑执行完毕返回时执行
        log.Println("请求处理完毕，准备返回响应")
    }
}
```

---

### 2. 执行流程：洋葱模型

Gin 的中间件采用的是**洋葱模型**。当你调用 `c.Next()` 时，程序会暂停当前函数，去执行下一个中间件，直到最后的业务逻辑执行完，再按原路返回。

- **`c.Next()`**：去执行后面的逻辑。
    
- **`c.Abort()`**：立即停止。如果认证失败，调用这个方法，请求就不会到达后续的业务逻辑。
    

---

### 3. 如何使用中间件？

#### 3.1 全局中间件

对所有的路由都生效（常用于日志、跨域、异常处理）。

Go

```
r := gin.Default() // Default 默认已经带了 Logger() 和 Recovery() 中间件
r.Use(MyMiddleware()) 
```

#### 3.2 路由组中间件

只对特定的一组路由生效（常用于需要登录的 API 段）。

Go

```
authorized := r.Group("/admin")
authorized.Use(AuthMiddleware()) // 只有 /admin 开头的路由才走这个逻辑
{
    authorized.POST("/login", loginHandler)
    authorized.POST("/submit", submitHandler)
}
```

#### 3.3 单个路由中间件

Go

```
r.GET("/ping", MyMiddleware(), func(c *gin.Context) {
    c.JSON(200, gin.H{"message": "pong"})
})
```

---

### 4. 常见的中间件场景

中间件通常用来处理那些**非业务逻辑但通用**的任务：

|**场景**|**说明**|
|---|---|
|**Authentication**|检查用户是否登录（验证 Token/Session）。|
|**Authorization**|检查用户是否有权限访问当前接口。|
|**Logger**|记录每个请求的路径、耗时、状态码。|
|**Recovery**|捕获代码中的 `panic`，防止程序崩溃，并返回 500。|
|**CORS**|处理跨域请求。|
|**Rate Limiting**|限制同一 IP 的访问频率，防止恶意刷接口。|



## 路由组中间件

简单来说，**路由组中间件**就是给一组具有“共同特征”的 URL 统一套上一层（或多层）关卡。

与其为每个路由单独指定中间件，不如把它们分门别类，统一管理。

---

### 1. 为什么要用路由组中间件？

假设你在开发一个系统，有以下两类接口：

1. **公开接口**（所有人都能看）：首页、注册、登录。
    
2. **私密接口**（必须登录才能看）：个人中心、修改密码、删除评论。
    

如果不使用路由组，你的代码会很冗余：

Go

```
// ❌ 坏习惯：每个路由都手动加中间件
r.GET("/user/profile", AuthMiddleware(), profileHandler)
r.POST("/user/password", AuthMiddleware(), passwordHandler)
r.DELETE("/user/comment", AuthMiddleware(), commentHandler)
```

---

### 2. 使用路由组（Group）后的写法

你可以创建一个路由组，并一次性把中间件应用到这个组上。**这个组里所有的子路径都会自动继承这个中间件。**

Go

```
r := gin.Default()

// 1. 创建一个名为 "user" 的路由组
userGroup := r.Group("/user")

// 2. 为这个组指定中间件
userGroup.Use(AuthMiddleware()) 

// 3. 在这个组下面定义具体的路由
{
    // 这里的路径会自动拼成 /user/profile
    userGroup.GET("/profile", profileHandler) 
    
    // 这里的路径会自动拼成 /user/password
    userGroup.POST("/password", passwordHandler)
}
```

---

### 3. 多层嵌套（进阶用法）

路由组中间件支持嵌套，这就像是**多重门禁**。

- 第一层：`/api` 组（通用中间件：记录日志、限流）。
    
- 第二层：`/api/admin` 组（特殊中间件：必须是管理员身份）。
    

Go

```
api := r.Group("/api").Use(Logger())
{
    // 所有人都能访问 /api/info
    api.GET("/info", infoHandler)

    // 只有管理员能访问 /api/admin/...
    admin := api.Group("/admin").Use(AdminAuth())
    {
        admin.POST("/delete-user", deleteHandler) // 这里会经过 Logger 和 AdminAuth 两个中间件
    }
}
```

---

### 4. 关键点总结

- **路径前缀**：`r.Group("/v1")` 会给组内所有路由自动加上 `/v1` 前缀。
    
- **中间件继承**：父组的中间件会自动作用于子组。
    
- **代码整洁**：通过大括号 `{}`（虽然不是语法强制，但是最佳实践）让代码逻辑结构像树状图一样清晰，一眼就能看出哪些接口是受保护的。