import os
import json

def compare_scores(folder="data"):
    """
    读取指定文件夹中所有 JSON 文件，
    提取每个文件的 Score 和调参参数，
    并按 Score 从高到低排序。

    返回:
        results: 列表，每个元素是一个字典
    """
    results = []

    # === 1️⃣ 检查文件夹是否存在 ===
    if not os.path.exists(folder):
        print(f"文件夹不存在: {folder}")
        return results

    # === 2️⃣ 遍历文件夹中所有文件 ===
    for filename in os.listdir(folder):
        if filename.endswith(".json"):
            file_path = os.path.join(folder, filename)

            try:
                # === 3️⃣ 打开并读取 JSON 文件 ===
                with open(file_path, "r") as f:
                    data = json.load(f)

                # === 4️⃣ 检查是否有 Score 字段 ===
                if "Score" not in data:
                    print(f"文件 {filename} 中没有 Score 字段，已跳过。")
                    continue

                # === 5️⃣ 提取需要的信息 ===
                result = {
                    "filename": filename,
                    "score": data["Score"],
                    "lookahead": data.get("Lookahead", "无"),
                    "max_speed": data.get("Max_Speed", "无"),
                    "min_speed": data.get("Min_Speed", "无")
                }

                results.append(result)

            except Exception as e:
                print(f"读取文件 {filename} 失败: {e}")

    # === 按 Score 从大到小排序（越大越好）===
    results.sort(key=lambda x: x["score"], reverse=True)

    return results


def main():
    # === 读取并比较所有结果 ===
    results = compare_scores("data")

    # === 如果没有可用结果，直接结束 ===
    if not results:
        print("没有找到可比较的 Score 数据。")
        return

    # === 打印所有文件的 Score 排名 ===
    print("\n=== Score 排名 ===")
    for i, result in enumerate(results, start=1):
        print(f"{i}. {result['filename']} -> Score: {result['score']:.4f}")

    # === 取出最优和最差结果 ===
    best = results[0]
    worst = results[-1]

    print("\n=== 总结 ===")
    print(f"最优结果 : {best['filename']} -> Score: {best['score']:.4f}")
    print(f"最差结果 : {worst['filename']} -> Score: {worst['score']:.4f}")

    # === 打印最优结果对应的调参参数 ===
    print("\n=== 最优结果对应的参数 ===")
    print(f"Lookahead : {best['lookahead']}")
    print(f"Max_Speed : {best['max_speed']}")
    print(f"Min_Speed : {best['min_speed']}")

    # === 计算并打印平均分 ===
    avg_score = sum(item["score"] for item in results) / len(results)
    print(f"\n平均 Score: {avg_score:.4f}")


if __name__ == "__main__":
    main()