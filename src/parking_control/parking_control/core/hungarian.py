"""헝가리안 알고리즘 기반 전역 최적 할당. 순수 Python (ROS import 금지).

NearestAllocator는 작업을 하나씩 처리하는 탐욕적 방식이라 전체 합이
최적이 아닐 수 있다. 여기서는 로봇×작업 비용행렬을 만들어
scipy.optimize.linear_sum_assignment(헝가리안)으로 총비용 최소 매칭을 구한다.
로봇 2~3대 규모에선 체감 차이가 작지만, 학습 목적으로 직접 구성했다.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

from .allocator import Allocator, Assignment

# 도달 불가(cost=None) 표시용. inf는 linear_sum_assignment가 feasibility
# 에러를 낼 수 있어 매우 큰 유한값을 쓰고, 결과에서 걸러낸다.
UNREACHABLE = 1e9


class HungarianAllocator(Allocator):

    def assign(self, robots, tasks, cost_fn) -> list:
        if not robots or not tasks:
            return []

        cost_matrix = np.full((len(robots), len(tasks)), UNREACHABLE)
        for i, robot in enumerate(robots):
            for j, task in enumerate(tasks):
                cost = cost_fn(robot, task)
                if cost is not None:
                    cost_matrix[i, j] = cost

        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        return [
            Assignment(robots[i].robot_id, tasks[j].task_id,
                       float(cost_matrix[i, j]))
            for i, j in zip(row_idx, col_idx)
            if cost_matrix[i, j] < UNREACHABLE
        ]
