# 移动机器人规划与控制大作业指导书

通过本实验，您将学习机器人规划和控制的相关内容。

## 统一评分细则与报告要求（代码部分）

**评分细则：**

- 依据本文件“任务一”部分补全代码并顺利编译运行，可获得 **30 分**。
- 继续完成任务二、任务三并调好控制器后，将根据轨迹跟踪正确性、稳定性和整体效果再给 **1–40 分**；完成任务三后请自行录制飞行视频 `solutions/video.mp4`，同时运行`code/calculate_results.py`会保存结果文件到`solutions/result.txt`。
- 若完成任务四中的附加题或进行其他有效改进，可依据工作量、效果与新颖性额外获得 **1–10 分**。

**报告要求：**

- 请把 `code/calculate_results.py` 的终端输出（包含最终 RMSE、轨迹运行时间、总长度、碰撞状态与综合得分等指标）贴到报告中，并且进行必要的分析；
- 如有任务四的附加改进，同样在报告中说明并且进行必要的对比实验。报告必须导出为 `report.pdf`，并且放置在`solutions/`路径下。

## 1. 运行环境安装

**注意：如果已经安装过 ROS，可以跳过本节环境安装，直接阅读第 2 节「实验项目介绍」。**

### 1.1 VMware 安装

首先安装运行环境。本项目统一使用 VMware Workstation 虚拟机来运行程序。

**推荐下载渠道：**

1. VMware 官网下载：[Desktop Hypervisor Solutions | VMware](https://www.vmware.com/products/desktop-hypervisor/workstation-and-fusion)

2. 在搜索引擎中搜索"VMware安装"，从网盘中下载（一般来说网速相对会更快一点）

3. 本实验使用的是 VMware WorkStation Pro 16（最新版本为 17，推荐使用 16）。这里提供版本 16 的百度网盘下载链接：
   ```
   https://pan.baidu.com/s/14tM3ApjTBXTA80aqmsbUnw?pwd=adc8
   ```

下载安装之后的界面如图所示：

<img src="./assets/wps1.jpg" width="400">

### 1.2 运行环境安装

为节省同学们的时间，并且规避环境安装中的各种问题，请各位同学下载已经配置好的环境直接使用，环境下载链接如下（**请提前预留至少 30GB 的空间**）：

```
please check in github for the latest link
```

在 VMware 的主界面，点击右上角的「文件」→「打开」：

<img src="./assets/wps2.jpg" width="200">

请选择刚才下载的 `.ovf` 文件，随后设置虚拟机名称和存储路径（至少大于 30GB）：

<img src="./assets/wps4.jpg" width="250">

等待导入完成后，进入环境即可，界面如下：

<img src="./assets/wps5.jpg" width="400">

该虚拟环境为 Ubuntu 20.04 系统，进入右上角的 `stuwork` 文件夹，如下图所示：

<img src="./assets/wps6.jpg" width="400">

其中，`Desktop`、`Documents`、`Downloads`、`Music`、`Public`、`Videos` 为系统文件，`Nlopt`、`osqp`、`osqp-eigen` 为系统环境运行所需的功能包，项目名称为 `MRPC-2025-homework`。**所有操作将在这个项目文件夹内进行。**

### 1.3 ROS 基础知识

下面简单介绍一下 ROS 文件的架构。

机器人操作系统（ROS）是一种用于编写机器人软件的灵活框架。它是工具、库和协议的集合，旨在简化在各种机器人平台上构建复杂而强大的机器人应用。

ROS 的工作目录如下：

<img src="./assets/wps13.jpg" width="300">

`Workspace`、`Package`、`Node` 是工程结构中的几个关键词，也是核心概念。以上视图展示了它们的包含关系。

标准的 Workspace 工作目录结构如下：

<img src="./assets/wps14.jpg" width="200">

ROS 的项目结构类似于树状结构：
- `src` 中为源代码文件
- `build` 和 `devel` 为编译过程中生成的文件
- `pkg1`、`pkg2`、`pkg3` 为项目下的各个功能包

各个功能包同样也是树状结构，里面包含对应的 `include`、`launch`、`src` 等文件夹。`include` 一般为 C++ 的头文件，`launch` 文件一般为配置文件，`src` 文件夹中包含的是各种源代码文件。为了方便同学们学习，注释全部放在了功能包内 `src` 文件夹下的文件中，**同学们只需要查看 `src` 下面的 `.cpp` 文件即可**。

一个完整的项目结构示意图如下：

<img src="./assets/wps15.jpg" width="400">

## 2. 实验项目介绍

在本节中，将对整个项目做详细的介绍，方便各位同学上手项目。本项目是集无人机模型仿真、建图、规划和控制的完整项目，用于考察同学们对于课程内容的学习。这个仓库也可以直接从 GitHub 拉取：

```
git clone https://github.com/Dwl2021/MRPC-2025-homework.git
```

### 项目结构说明

- **`map_server`**：主要负责无人机建图和规划的功能包
  - `map_generator`：用于生成地图的代码
  - `map_render`：将地图信息发送出去用于无人机的感知

- **`quadrotor_simulator`**：主要包含无人机的仿真器
  - `so3_control`：包含位置和速度控制器的部分
  - `so3_quadrotor_simulator`：包含无人机的姿态控制以及四旋翼的控制器仿真

- **`trajectory_generator`**：无人机的轨迹规划部分，其中包含需要填写的 A* 路径搜索方法

- **`Utils`**：包含轨迹的发布、轨迹可视化等一些基础工具

### 与课程内容的对应关系

对应于课程 PPT 的内容，完整的信息传输结构如下：

<img src="./assets/image-20251113201211346.png" width="1000">

- **决策与规划**：`trajectory_generator`
- **位置与速度控制器**：`so3_control`
- **姿态控制器**：`so3_quadrotor_simulator`
- **硬件平台仿真器**：`so3_quadrotor_simulator`

### 信息传输流程

完整的信息传输结构如下：

<img src="./assets/wps17.jpg" width="1000">

**信息传输流程说明：**

- `/sim/imu`：仿真无人机的 IMU 信息
- `/visual_sim/odom`：里程计（odom）信息
- `/random_complex`：用来生成地图信息
- `/random_complex/global_map` 和 `/random_complex/global_ground`：传递全局的地图信息
- `/pcl_render_node`：将地图信息发送出去
- `/trajectory_generator_node`：接收 odom 信息、地图信息以及 `/goal` 传递过来的目标点信息，然后生成 `/trajectory_generator_node/trajectory` 中的轨迹信息
- `/traj_server`：将轨迹信息发送给底层的控制器
- `/so3_control`：进行位置控制
- `/quadrotor_simulator_so3`：控制四旋翼无人机的运动

## 3. 实验任务

### 3.1 任务一：补全四旋翼飞机的动力学模型

在 `MRPC-2025-homework/code/src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp` 中描述了四旋翼的动力学模型，请阅读代码 `Quadrotor.cpp`，补充四旋翼的动力学模型：

```cpp
x_dot = cur_state.v;
//请在这里补充完四旋翼飞机的动力学模型，提示：v_dot应该与重力，总推力，外力和空气阻力相关
// v_dot = //?????

acc_ = v_dot;

R_dot = R * omega_vee;
//请在这里补充完四旋翼飞机的动力学模型，角速度导数的计算涉及到惯性矩阵J_的逆、力矩、科里奥利力（通过角速度与惯性矩阵和角速度的叉积来计算）和外部力矩等因素。
// omega_dot = //??????
```


**补全完成之后进行测试：**

#### 步骤 1：编译代码

首先要对文件进行**编译**。**注意：每次修改完代码之后，都要重新编译，所修改的内容才会生效。**

```bash
cd MRPC-2025-homework
catkin_make
```

如果最后编译到 `100%` 并且没有显示报错，则说明编译成功。

#### 步骤 2：测试无人机悬停

测试一下无人机的悬停，确认代码补全正确：

```bash
cd MRPC-2025-homework
source devel/setup.bash
roslaunch trajectory_generator test_control.launch
```

会弹出可视化界面：

<img src="./assets/image-20251113202022886.png" width="500">

#### 步骤 3：验证悬停效果

目前无人机已经停在空中约 3 米的位置上，在 Rviz 中如下图所示：

<img src="./assets/image-20251113202128649.png" width="400">

#### 步骤 4：测试移动

点击 Rviz 中的 `3D Nav Goal`，在界面中的白色地板上进行拖动。倘若无人机能够沿着 z 轴向上移动 6m，则表明你的修改成功。倘若飞机不能向上移动则表明修改失败。

<img src="./assets/image-20251113202237918.png" width="400">


**提交方式** ：请在报告中稍微介绍一下任务一的补全思路，注意简洁清晰，切勿放大段代码。


### 3.2 任务二：补充无人机的前端规划模块

在 `MRPC-2025-homework/code/src/planner/path_searching/src/kinodynamic_astar.cpp` 中补充完有关于 A* 的相关内容，共有 4 个 `STEP` 需要你补齐。

#### STEP 1.1：补全启发式函数

首先你要补全的是启发式算法的部分，包括填写你所使用的距离表示方式、启发式算法，以及是否使用 `tie_breaker`。

```cpp
double Astarpath::getHeu(MappingNodePtr node1, MappingNodePtr node2) {
  double heu;
  double tie_breaker;
  // ???
  return heu;
}
```

#### STEP 1.2：补全 A* 算法主循环

接下来请在下面的循环中完成对 A* 算法的补全，已对算法进行了提示，共有下面 4 处 `// ???` 的代码需要补充。

```cpp
while (!Openset.empty()) {
    // 1. 弹出 g+h 最小的节点
    // ???
    
    // 2. 判断是否是终点
    // ???
    
    // 3. 拓展当前节点
    // ???
    
    for(unsigned int i=0; i<neighborPtrSets.size(); i++){
      if(neighborPtrSets[i]->id == -1)
      {
         continue;
      }
      tentative_g_score = currentPtr->g_score + edgeCostSets[i];
      neighborPtr = neighborPtrSets[i];
      if(isOccupied(neighborPtr->index))
        continue;
      if(neighborPtr->id == 0)
      {
        // 4. 填写信息，完成更新
        // ???
        continue;
      }
      else if(neighborPtr->id == 1)
      {
        if(neighborPtr->g_score > tentative_g_score){
          neighborPtr->g_score = tentative_g_score;
          neighborPtr->Father = currentPtr;
          neighborPtr->f_score = tentative_g_score + getHeu(neighborPtr, endPtr);
          Openset.insert(make_pair(neighborPtr->f_score, neighborPtr));
        }
        continue;
      }
    }
  }

  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}
```

#### STEP 1.3：追溯找到的路径

补充完"返回 A* 寻找到的路径"的部分：

```cpp
do{
  terminatePtr->coord=gridIndex2coord(terminatePtr->index);
  front_path.push_back(terminatePtr);
  terminatePtr=terminatePtr->Father;
}while(terminatePtr->Father!=NULL);
  /**
   *
   * STEP 1.3:  追溯找到的路径
   *
  * **/

// ???
return path;
```

#### 补全完成后进行测试

**步骤 1：编译代码**

首先要对文件进行编译。每次修改完代码之后，都要重新编译，所修改的内容才会生效。

```bash
cd MRPC-2025-homework
catkin_make
```

**步骤 2：启动仿真**

编译成功之后，启动 launch 文件：

```bash
cd MRPC-2025-homework
source devel/setup.bash
roslaunch trajectory_generator demo.launch
```

**步骤 3：测试路径规划**

然后会弹出可视化界面：

<img src="./assets/image-20251113203635286.png" width="400">

粉色的为无人机模型，红色的为无人机的局部感知范围，灰色的为点云地图。现在点击 `3D Nav Goal`，在界面上拖动一下。

<img src="./assets/image-20251113203706849.png" width="400">

运行成功时，飞机会寻找路径，到达目标点。


**提交方式** ：在报告中稍微介绍一下任务二的补全思路，注意简洁清晰，切勿放大段代码。


### 3.3 任务三：控制器参数调试

在上面所有的代码都补齐之后，你应该可以编译成功并且进行测试了，但是你会发现飞机对于轨迹的跟踪并不是很好，那么你需要对控制器的参数进行调整。

请阅读 `MRPC-2025-homework/code/src/uav_simulator/so3_control/src/so3_control_nodelet.cpp` 中的控制器代码，并且对 `position_cmd_callback` 函数中的 `kx_` 和 `kv_` 参数进行调整。

```cpp
void SO3ControlNodelet::position_cmd_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  kx_ = Eigen::Vector3d(15.7, 8.7, 6.0);
  kv_ = Eigen::Vector3d(6.4, 13.4, 4.0);

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}
```

#### 补全完成后进行测试

进入工作空间，然后运行评估脚本：

```bash
cd MRPC-2025-homework/
python3 calculate_results.py
```

输出示例：

```
计算得到的均方根误差(RMSE)值为: 0.11586971639650995
轨迹运行总时间为: 47.12802791595459
总轨迹长度为: 33.97356465040666
是否发生了碰撞: 1
综合评价得分为(综合分数越低越好): 1.3102676466588432
```

**提交方式** ：任务三运行成功之后，录制一个运行视频命名为`video.mp4`并且放置到`solutions/`路径下，同时运行`code/calculate_results.py`后会生成`solutions/result.txt`结果文件，然后把终端得到的RMSE，轨迹运行时间等指标贴到报告中，并且进行必要的分析，包括但不限于调参思路和技巧，简洁为主。

### 3.4 任务四：附加题（Optional）

如果学有余力的同学，可以尝试以下改进方向：

1. **前端规划改进**：使用 JPS3D、Hybrid A*、RRT 等方法进行改进
2. **后端轨迹优化**：如果对优化有所了解，可以修改后端的轨迹优化算法
3. **控制器改进**：编写新的控制器，将姿态环和角速度环从 `so3_quadrotor_simulator` 移植到 `so3_control` 当中去

**提交方式** ：如果选做了该附加题，需要在报告中清晰地介绍你的改进思路和改进方法，以及进行必要的对比实验。报告力求简洁为主，最影响得分的主要还是附加题的新颖性和最终的飞行效果。


## 4. 提交要求

1. 在 `solutions` 目录下保留三类最终材料：
   - `report.pdf`：需要包含上述所提到的必要的思路分析以及实验结果，以及如果选做了附加题则需要简洁清晰地描述附加题的改进方案。
   - `video.mp4`：任务三完成后的无人机飞行视频（录屏）。
   - `result.txt`：任务三完成以后的结果文件。
2. 确保 `code/` 目录内的 ROS 包均可按本 README 的步骤编译、运行。
