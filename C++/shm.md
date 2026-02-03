## POSIX 是什么

**POSIX** (Portable Operating System Interface) 是 IEEE 制定的一套操作系统接口标准，目的是让应用程序能在不同的 Unix-like 系统间移植。

### 核心思想

写一次代码，能在 Linux、macOS、FreeBSD、Solaris 等系统上编译运行，不用针对每个系统写不同版本。

### POSIX 定义了什么

|类别|主要 API|用途|
|---|---|---|
|文件操作|`open()`, `read()`, `write()`, `close()`|文件读写|
|进程管理|`fork()`, `exec()`, `wait()`, `kill()`|创建/管理进程|
|线程|`pthread_create()`, `pthread_mutex_*`|多线程编程|
|共享内存|`shm_open()`, `mmap()`, `munmap()`|进程间共享数据|
|信号量|`sem_open()`, `sem_wait()`, `sem_post()`|进程/线程同步|
|信号|`signal()`, `sigaction()`|异步事件处理|
|网络|`socket()`, `bind()`, `listen()`, `accept()`|网络通信|

### 你项目中用到的 POSIX 共享内存 API

```cpp
// 1. 创建/打开共享内存对象
int fd = shm_open("/my_shm", O_CREAT | O_RDWR, 0666);

// 2. 设置大小
ftruncate(fd, size);

// 3. 映射到进程地址空间（核心！）
void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

// 4. 直接读写 ptr 指向的内存，其他进程也能看到

// 5. 清理
munmap(ptr, size);
close(fd);
shm_unlink("/my_shm");  // 删除共享内存对象
```

### 为什么用 POSIX 共享内存而不是其他 IPC

|方式|优点|缺点|
|---|---|---|
|POSIX 共享内存|零拷贝、最快、跨平台|需要自己处理同步|
|管道 (pipe)|简单|需要拷贝数据、单向|
|消息队列|有消息边界|有大小限制、需要拷贝|
|Socket|可跨网络|开销大、需要序列化|
|ROS Topic|生态好、工具多|延迟高、依赖 ROS|

### 共享内存的本质

```
进程A                    物理内存                    进程B
+--------+              +--------+              +--------+
| 虚拟   |  mmap映射    |  共享  |   mmap映射   | 虚拟   |
| 地址   | -----------> |  内存  | <----------- | 地址   |
| 0x1000 |              | 区域   |              | 0x2000 |
+--------+              +--------+              +--------+
```

两个进程的虚拟地址不同，但都映射到同一块物理内存。进程 A 写入的数据，进程 B 立刻就能读到，**没有任何拷贝**。

### 平台差异

|系统|共享内存位置|
|---|---|
|Linux|`/dev/shm/`|
|macOS|`/tmp/` (实际由内核管理)|

你的代码用的是标准 POSIX API，所以在 Linux 和 macOS 上都能跑。