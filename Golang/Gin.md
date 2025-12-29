### 1️⃣ **路由与 Handler**

- **路由**：决定请求路径和方法，触发对应的 handler 函数。
    
- **Handler**：处理请求并返回响应，入参为 `c *gin.Context`。
    

### 2️⃣ **匿名函数**：

`r.GET("/ping", func(c *gin.Context) {     c.JSON(200, gin.H{"message": "pong"}) })`

### 3️⃣ **命名函数**：

`func PingHandler(c *gin.Context) {     c.JSON(200, gin.H{"message": "pong"}) } r.GET("/ping", PingHandler)`

### 4️⃣ **结构体方法**：

`type AuthHandler struct{} func (h *AuthHandler) Register(c *gin.Context) {     // 注册逻辑 } r.POST("/register", authHandler.Register)`

### 5️⃣ **总结**

- **`c *gin.Context`**：所有 handler 函数的标准入参，处理请求与响应。
    
- **匿名函数 / 命名函数**：都可以用作路由的 handler。
    
- **路由定义**：`r.GET()`、`r.POST()` 等关联路径与处理函数。