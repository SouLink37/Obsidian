

## 🔍 DSN 完整解析

`root:12345678@tcp(127.0.0.1:3306)/gorm_homework?charset=utf8mb4&parseTime=True&loc=Local`


### ## 📋 详细说明

### 第一部分：用户认证

root:12345678@

|符号|含义|值|
|---|---|---|
|root|数据库用户名|root（MySQL 默认管理员）|
|:|分隔符|-|
|12345678|数据库密码|你设置的 MySQL 密码|
|@|结束标记|-|

对应配置： DBUser 和 DBPassword

---

### 第二部分：网络连接

tcp(127.0.0.1:3306)

|符号|含义|值|
|---|---|---|
|tcp|网络协议|TCP 协议（最常用）|
|127.0.0.1|主机地址|本地计算机（localhost）|
|:|分隔符|-|
|3306|端口号|MySQL 默认端口|

对应配置： DBHost 和 DBPort

---

### 第三部分：数据库名

/gorm_homework

|符号|含义|值|
|---|---|---|
|/|路径分隔符|-|
|gorm_homework|数据库名|你要连接的数据库|

对应配置： DBName

---

### 第四部分：连接参数

?charset=utf8mb4&parseTime=True&loc=Local

这是 URL 查询参数格式（用 ? 开头，多个参数用 & 分隔）

| 参数        | 值       | 含义                    |
| --------- | ------- | --------------------- |
| charset   | utf8mb4 | 字符编码（支持中文和 emoji）     |
| parseTime | True    | 自动解析时间（重要！否则时间会变成字符串） |
| loc       | Local   | 时区（使用本地时区）            |

