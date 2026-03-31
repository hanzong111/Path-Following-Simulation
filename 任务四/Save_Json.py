from Compute_Score import compute_score

def save_simulation_result(trajectory, errors, controls,lookahead, max_speed, min_speed,  folder="data"):
    """
    保存仿真结果到 JSON 文件（自动生成时间戳文件名）

    参数:
        trajectory: 轨迹数据 (list)
        errors: 误差数据 (list)
        controls: 控制输入 (list)
        folder: 保存目录（默认 data)

    返回:
        file_path: 保存的文件路径
        score: 计算得到的评分
    """

    import json
    import os
    from datetime import datetime

    # === 构建数据 ===
    data = {
        "Trajectory": trajectory,
        "Error": errors,
        "Control": controls,
        "Lookahead": lookahead,
        "Max_Speed": max_speed,
        "Min_Speed": min_speed
    }

    # === 计算 Score ===
    score = compute_score(data)
    data["Score"] = score

    print(f"Score: {score:.4f}")

    # === 创建文件夹 ===
    os.makedirs(folder, exist_ok=True)

    # === 生成唯一文件名 ===
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = f"{folder}/task_result_{timestamp}.json"

    # === 写入文件 ===
    with open(file_path, "w") as f:
        json.dump(data, f, indent=4)

    print(f"数据已保存到 {file_path}")

    return file_path, score