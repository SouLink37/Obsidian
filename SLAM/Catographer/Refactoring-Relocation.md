## 整体介绍：

代码重构后整个系统ros和逻辑代码分离。 整个结构主要分为逻辑端，中间接口，以及ros端。

### 逻辑端（重定位部分）：

relocation.h / relocation.cpp 为逻辑端，主要函数：

- GetStaticResult(); 静态重定位[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-PwXddmoPeog1MNxau1ccInhqnHh)
    
- GetDynamicResult(); 动态重定位[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-NJYAdW0xxokNsnxusv6cJxxVnCg)
    
- CancelRelocation(); 取消重定位
    
- AddXXXData(); 将各种数据加入对应的数据队列
    

### 中间接口：

system.h / system.cpp 为中间接口： 主要负责调用并封装逻辑端的函数，给外部ros或其他提供调用接口。

- 封装GetStaticResult();GetDynamicResult();CancelRelocation(); 函数为HandleStaticReLocation();HandleDynamicReLocation();StopRelocation();
    
- XXXCallback()函数，调用重定位中(也包含其他模块)AddXXXData(); 对内为模块提供数据流。对外提供数据传入接口。
    

### ros端

node_online.h . node_online.cpp 为ros端：

ros 端不直接调用逻辑端，而是调用system提供的接口。

#### RosActionServer：

Ros端提供了一个重定位action服务；

- HandleStartRelocationGoal();
    
      判断是否接受重定位
    
- HandleStartRelocationCancel();
    
      判断是否同意取消重定位，如果同意，调用system提供的StopRelocation();
    
      然后取消移动请求，并结束建小图。
    
- HandleStartRelocationAccepted();
    

接受重定位后执行，调用内部的StartRelocationExecute();

- StartRelocationExecute();
    
      执行重定位的主要逻辑。
    
      执行重定位会根据定义好的ros action中的Goal，来判断重定位的类型，进而做出不同的决策
    
    - relocation_type 1 ：先静后动
        
    - relocation_type 2 ：先静后动，优先在当前地图搜索
        
    - relocation_type 3 ：纯静态
        
    - 基站模式: 缩小搜索范围
        
    
       对于模式1 模式2 ，进入重定位前（无论哪个模式都会先进行静态重定位），先等待一段时间，保证各种数据进入队列。然后通过调用system中的HandleStaticReLocation()完成静态重定位，通过反馈的重定位结果relocation_result.err_code判断静态重定位是否成功，如果成功，返回重定位结果；否则会进入动态重定位，开启动态重定位前会发出建小图和移动请求，然后通过调用sysem中的HandleDynamicReLocation()开启动态重定位，并返回结果。动态重定位结束后会发出取消移动和结束建小图请求。
    
      对于模式3，只通过sysyem调用静态重定位的逻辑。并返回结果。
    

## 动态重定位：

###   流程图：
![[动态重定位.png]]
  

  

###   流程介绍：

```
1. 清空frame数据队列（std::deque<FramePtr> raw_frames_）
    
           防止使用到非当前小图内的数据
    
2. 检查前置条件是否符合 （查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-NH9Pdap27o1ATPxqlqFcr0r5n5d)）
    
          成功后，记录开始时间
    
3. 加载离线地图（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-WPQwd2RtLoAvF2xob68cspm4nHf)）
    
4. 进入主循环 , 两次循环最小间隔 3s
    
    1. 判断是否取消，取消则结束流程
        
    2. 判断是否超时，超时则结束流程
        
    3. 获取融合点云数据
        
        1. 动态重定位中，和地图进行匹配的点云数据为连续几帧数据融合而成
            
        2. 融合前判断是否有足够的数据，如果不够等待最多10次（每次3s）
            
        3. 10次后如果没有frame数据，重定位结束，
            
                      如果有数据但是不足预设的数量，有几帧数据就融合几帧
            
        4. MergeFrames（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-VVA0dM322ofL4kxRzt2csaOBnie)）
            
    4. 用融合点云merged_frame，在每个楼层地图上进行匹配，如果匹配成功，将结果加入黑板（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-BJozdQ86nomcnoxA09tc0lSLnJe)）
        
    5. 将黑板中每个楼层中最新的数据提取出来std::vector<std::pair<int, FramePtr>> candidate_frames;
        
    6. 寻找最佳得分（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-MCfYdVhkToYGkfxpASgcOk0Lnhe)）
        
    7. 判断连续成功次数
        
        1. 没有最佳得分（best_frame == -1）连续次数为0，进入下一次循环；
            
        2. 最佳得分和上一次不在同一楼层，连续次数为1，进入下一次循环；
            
        3. 最佳得分和上一次在同一楼层，连续次数+1，如果连续次数不足，进入下一次循环，连续次数满足，给结果赋值；
            
    8. 重定位结果赋值relocation_result_
        
        1. timestamp_：当前最佳得分时间戳
            
        2. best_candidate_score_： 当前最佳得分分数
            
        3. current_candidate_：当前最佳得分位姿
            
        4. best_candidate_：当前实际机器所在位姿
            
        5. the_relocation_start_pose_：重定位开始时的位姿（建小图开始时）
            
        6. map_id_：最佳得分所在地图楼层
            
        
                如果重定位结果在地图范围内，返回结果，否则重定位失败
        

```

## 静态重定位：

###   流程图：
![[静态重定位.png]]


###   流程介绍：

1. 检查前置条件是否符合 （查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-NH9Pdap27o1ATPxqlqFcr0r5n5d)）
    
2. 加载离线地图（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-WPQwd2RtLoAvF2xob68cspm4nHf)）
    
3. 获取雷达数据
    

  laser_frames_中的最后一帧current_frame。数据由laser数据转换成frame而来，和动态重定位中使用的（raw_frames_）是不同的数据队列，互不干扰。

    进入静态重定位之前，会等待数据进入(由action server执行)，因此静态重定位流程中不会再等待数据

4. 用点云current_frame，在每个楼层地图上进行匹配，如果匹配成功，将结果加入黑板（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-BJozdQ86nomcnoxA09tc0lSLnJe)）
    
5. 将黑板中每个楼层中最新的数据提取出来std::vector<std::pair<int, FramePtr>> candidate_frames;
    

  对于静态重定位，黑板中也只有一次数据

6. 寻找最佳得分（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-MCfYdVhkToYGkfxpASgcOk0Lnhe)）
    
7. 重定位结果赋值relocation_result_
    
         如果有最佳得分（best_frame.first != -1）
    
    2. map_id_：最佳得分所在地图楼层
        
    3. timestamp_：当前最佳得分时间戳
        
    4. best_candidate_score_： 当前最佳得分分数
        
    5. current_candidate_：当前最佳得分位姿
        
    6. best_candidate_：当前最佳得分位姿
        
    7. the_relocation_start_pose_：当前最佳得分位姿
        
    
         由于静态重定位位姿只有一个，所以所有位姿结果都是当前最佳得分位姿
    
        如果重定位结果在地图范围内，返回结果，否则重定位失败
    

  

## 函数介绍：

- SmartErrCode CheckPreconditions(const ReLocationGoal& relocation_goal)；
    
      调取imu数据（如果imu队列为空 最多等待10s）。
    
      检查当前机器是否为倾斜状态, 如果倾斜，重定位失败。
    
- SmartErrCode LoadOfflineMaps(std::string map_path)；
    

加载0 - 4 层离线地图到std::map<int, SubMapPtr> offline_maps_。

key代表楼层。

- bool MergeFrames(FramePtr& merged_frame, const size_t merge_size)；
    
    - 清空 key_frames_。
        
    - 加入待融合的关键帧到清空 key_frames_， 清空raw_frames_。
        
    - 将带融合的关键帧的所有点云，转换到最后一帧的位姿，叠加。
        
    - 体素滤波。
        
    - 得到融合点云merged_frame_，其中位姿信息为最后一帧的位姿信息。
        
- void AddCandidateToBlackboard(int map_id, FramePtr& candidate_frame, const SmartErrCode match_code)
    

std::map<int, std::vector<FramePtr>> map_blackboard_;

黑板中存储每个楼层中的成功匹配的frame，key代表楼层。

加入前判断数据是否符合要求（查看函数介绍：[Refactoring: Relocation](https://lcn1mm23cxn4.feishu.cn/wiki/Lp1ywGZHhiQZsxk05V8c2O27nPg#share-Es3odYIRqoKT1kxqZbscSgolnLb)）。

- bool CheckCandidateSync(const FramePtr& old_frame, const FramePtr& new_frame)
    
       判断位姿是否和odom的位姿同步，如果偏差过大，返回false。
    
       判断和上一次位姿距离或者角度是否变化足够大，若变化过小，2帧过于接近，返回false。
    
- std::pair<int, FramePtr> FindBestScore(std::vector<std::pair<int, FramePtr>>& candidate_frames, const ReLocationGoal& relocation_goal)；
    
    - 寻找到最佳解的条件：该解的得分超过重定位得分阈值，并且该解明显大于其他解。
        
    - 访问过的frame的score标记为负数，保证下次判断时，如果当前楼层没有新加入的数据，不会再次将当前得分重复算作最高分。
        
    - 找到最佳解，在黑板上清除 除了最佳解以外其他所有楼层的候选数据；
        
    
      如果没有最佳解，清除整个黑板。
    
    - 返回最佳解的楼层和frame：std::pair<int, FramePtr> best_frame;
        
    
       如果best_frame.first为-1，代表没有最佳解。