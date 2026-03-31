# 无人驾驶机器人仿真项目 – 路径跟踪与自动调参

由于计算机GPU算力不足，难以支持 Isaac Sim 的运行，因此选用 PyBullet 作为替代仿真环境。
本项目用于仿真实习生笔试任务，基于 PyBullet 仿真环境实现机器人的路径跟踪控制、数据采集、性能评估以及自动调参功能。

## 环境要求

- Python 3.8 或更高版本
- pip 包管理器
- 推荐操作系统：Ubuntu 20.04+ / Windows WSL2

## 快速开始

### 1. 创建虚拟环境

打开终端，进入项目根目录，执行以下命令创建并激活虚拟环境：

```bash
# 创建虚拟环境（Python 3）
python3 -m venv venv

# 激活虚拟环境
# Linux / macOS:
source venv/bin/activate
# Windows:
venv\Scripts\activate

# 安装依赖 
pip install -r requirements.txt
```
### 2. 运行方式

### 1）任务一
```bash
# 进入folder
cd 任务一
# 运行命令
python3 simulation1.py
```

运行命令后，会看到以下提示：

```text
# 这里可以调参数，前进与转弯速度
=== 机器人运动控制（真实汽车）===
============= 前进 ============
前进线速度 v (默认 2.0): *
============= 转弯 ============
转弯线速度 v (默认 1.0): *
转弯角速度 ω (默认 1.0 rad/s): *
```

> `*` 表示此处需要输入数值（直接按回车则使用默认值）。```

### 2）任务二
```bash
# 进入folder
cd 任务二
# 运行命令
python3 simulation2.py
```
在``simulation.py``里，可以改变 waypoints 来控制机器人跟的路径和参数

![alt text](Simulation_CONFIG.png)

运行命令后，会看到以下提示：
```text
路径点 (回车使用 CONFIG 默认 [(0.0, 0.0), (5.0, 0.0), (5, 10), (3, 8)]) 
```
回车就可以了。

### 3）任务三
```bash
# 进入folder
cd 任务三
# 运行命令
python3 simulation3.py
```
与任务二同样的，只是现在跑完程序后，会多一个 ``task3_result.json``文件。里面就会有实验的结果
```json
{
    "trajectory": [...],
    "error": [...],
    "control": [...]
}
```
### 3）任务四


### 实验结果：

![alt text](Task_Json.png)


### Comparison.py 结果：

![alt text](Summary_Output.png)