# 无人驾驶机器人仿真项目 – 路径跟踪与自动调参

由于计算机GPU算力不足，难以支持 Isaac Sim 的运行，因此选用 PyBullet 作为替代仿真环境。
本项目用于仿真实习生笔试任务，基于 PyBullet 仿真环境实现机器人的路径跟踪控制、数据采集、性能评估以及自动调参功能。

## 环境要求

- Python 3.8 或更高版本
- pip 包管理器
- 推荐操作系统：Ubuntu 20.04+ / Windows WSL2

## 快速开始

### 创建虚拟环境

打开终端，进入项目根目录，执行以下命令创建并激活虚拟环境：

```bash
# 创建虚拟环境（Python 3）
python3 -m venv myvenv

# 激活虚拟环境
# Linux / macOS:
source myenv/bin/activate


# 安装依赖 
pip install -r requirements.txt
```
# 运行方式

# 1）任务一
任务要求

1) 支持控制机器人运动（线速度 v + 角速度 ω)
2) 能成功启动仿真环境
3) 机器人可正常运动

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

# 2）任务二
```bash
# 进入folder
cd 任务二
# 运行命令
python3 simulation2.py
```
在``simulation.py``里，可以改变 waypoints 来控制机器人跟的路径和参数

![alt text](Simulation_CONFIG.png)


# 3）任务三
```bash
# 进入folder
cd 任务三
# 运行命令
python3 simulation3.py
```
与任务二同样的，只是现在跑完程序后，会多一个 ``task_result.json``文件。里面就会有实验的结果
```json
{
    "trajectory": [...],
    "error": [...],
    "control": [...]
}
```
# 3）任务四


## Compute_score.py 说明
该函数通过两项惩罚指标评估机器人的路径跟踪质量，分数越高（越接近 0）表示表现越好。
评分逻辑
1. 均方根误差（RMSE）- 衡量机器人实际轨迹与目标路径之间的平均偏差。偏差越大，惩罚越重。
2. 晃动率（Oscillation）- 统计角速度 ω 正负号切换的频率（即左转→右转的次数）。
切换越频繁，说明机器人在来回摆动、过度修正，而非平稳跟踪路径。
除以总步数以消除路径长度的影响，使指标具有可比性。
```python 
score = -(rmse + 0.3 * oscillation)
```


### 实验结果都会存在 data/ 里：

![alt text](Task_Json.png)


# Comparison.py 说明：
运行方式 ：
```bash
python3 Comparison.py
```
它会把所做过的仿真结果数据都做对比，把最好的成绩呈现出来

![alt text](Summary_Output.png)