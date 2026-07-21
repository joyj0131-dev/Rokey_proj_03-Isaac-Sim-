"""그래프 최단 경로 탐색 + 엣지 차단/복구. 순수 Python (ROS import 금지).

safety_monitor가 장애물을 보고하면 block_edge()로 해당 통로를 막고,
해제되면 unblock_edge()로 복구한다. 차단은 엣지를 그래프에서 제거하는
방식이라 탐색 알고리즘 쪽에는 아무 분기가 없다.
"""

from dataclasses import dataclass, field

import networkx as nx

from .graph import ParkingMap


@dataclass
class PathResult:
    nodes: list          # 노드 id 순서
    length: float        # 총 거리 (m)
    waypoints: list      # (x, y) 좌표 순서 — NavigateToPose goal 시퀀스
    zones: list = field(default_factory=list)  # 통과하는 존 순서


class PathFinder:

    def __init__(self, parking_map: ParkingMap):
        self.map = parking_map
        self._blocked = {}  # frozenset({u, v}) -> 보관한 엣지 속성

    def block_edge(self, u, v):
        key = frozenset((u, v))
        if key in self._blocked:
            return
        self._blocked[key] = dict(self.map.graph.edges[u, v])
        self.map.graph.remove_edge(u, v)

    def unblock_edge(self, u, v):
        attrs = self._blocked.pop(frozenset((u, v)), None)
        if attrs is not None:
            self.map.graph.add_edge(u, v, **attrs)

    @property
    def blocked_edges(self) -> list:
        return [tuple(sorted(key)) for key in self._blocked]

    def find_path(self, start, goal) -> PathResult | None:
        """최단 경로. 도달 불가면 None (호출자가 명시적으로 처리)."""
        try:
            nodes = nx.shortest_path(self.map.graph, start, goal, weight="dist")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None
        length = sum(self.map.graph.edges[u, v]["dist"]
                     for u, v in zip(nodes, nodes[1:]))
        return PathResult(
            nodes=nodes,
            length=length,
            waypoints=[self.map.node_pos(n) for n in nodes],
            zones=self.map.edge_zones(nodes),
        )
