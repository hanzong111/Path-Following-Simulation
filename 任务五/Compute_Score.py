import json
import numpy as np

def compute_score(data):
    errors = data['Error']
    controls = data.get('controls', [])

    if len(errors) == 0:
        return -float('inf')

    # 1. Root-Mean_Square Error
    rmse = np.sqrt(np.mean(np.square(errors)))

    # 2. 晃动: ω +/- 的改变频率
    if len(controls) > 1:
        omegas = [c[1] for c in controls]
        sign_changes = sum(
            1 for i in range(1, len(omegas))
            if omegas[i] * omegas[i-1] < 0  # left turn → right turn = oscillation
        )
        oscillation = sign_changes / len(omegas)  # normalize by steps
    else:
        oscillation = 0.0

    # 分数
    score = -(rmse + 0.3 * oscillation)

    return score