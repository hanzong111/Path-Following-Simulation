import json
import numpy as np

def compute_score(data):
    """
    根据仿真数据计算评分
    """
    errors = data['Error']
    
    # 如果没有数据，直接返回负无穷（表示极差表现）
    if len(errors) == 0:
        return -float('inf')
    
    # 1. 均方根误差（RMSE）
    # 用来衡量整体误差大小，对大误差惩罚更严重
    rmse = np.sqrt(np.mean(np.square(errors)))
    
    # 2️. 误差总变化量（Total Variation）
    # 衡量误差是否“抖动”或震荡（越大说明越不稳定）
    
    # 计算相邻误差之间的变化量
    total_variation = np.sum(np.abs(np.diff(errors)))
    normalized_variation = total_variation / len(errors)
    
    # 3️. 最终评分（Score）
    # 评分 = -(误差 + 振荡惩罚)
    # 权重 0.3 控制对“震荡”的惩罚程度
    score = - (rmse + 0.5 * normalized_variation)
    
    return score