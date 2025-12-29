## 场景

用户登录成功 → 服务器发 JWT  
以后每次请求 → 用 JWT 验证身份

---

## 1️⃣ 安装库

`go get github.com/golang-jwt/jwt/v5`

---

## 2️⃣ 生成 JWT（登录时）

`secret := []byte("my_secret")  token := jwt.NewWithClaims(jwt.SigningMethodHS256, jwt.MapClaims{ 	"user_id": 123, 	"exp":     time.Now().Add(2 * time.Hour).Unix(), })  tokenString, _ := token.SignedString(secret)`

👉 得到的 `tokenString` 就是发给前端的 JWT

---

## 3️⃣ 前端请求时带上 JWT

`Authorization: Bearer <tokenString>`

---

## 4️⃣ 验证 JWT（接口鉴权）

`token, err := jwt.Parse(tokenString, func(t *jwt.Token) (interface{}, error) { 	return secret, nil })  if err != nil || !token.Valid { 	// token 无效 }`

---

## 5️⃣ 取出用户信息

`claims := token.Claims.(jwt.MapClaims) userID := claims["user_id"]`

---

## 一句话记住

- **生成**：登录成功发 token
    
- **验证**：每次请求校验 token
    
- **作用**：不用 session，也知道你是谁