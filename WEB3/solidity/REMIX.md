在 **Remix IDE** 的 **"Deploy & Run Transactions"** 面板中，函数按钮的**颜色**代表以下含义：

|**颜色**|**含义**|
|---|---|
|**红色**|`payable` 函数（需要发送 ETH，如 `deposit`）。|
|**橙色**|会**修改区块链状态**的函数（非 `view`/`pure`，如 `withdraw`、`pause`）。|
|**蓝色**|**只读**函数（`view`/`pure`，如 `getBalance`、`getContractBalance`）。|
|**灰色**|**不可用**（如 `onlyOwner` 函数在非 Owner 调用时）。|
