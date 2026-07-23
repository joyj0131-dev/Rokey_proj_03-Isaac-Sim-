"""parking_map.yaml → networkx 그래프 로더. 순수 Python (ROS import 금지).

엣지 가중치(거리)는 노드 좌표에서 자동 계산한다 — YAML에 거리를 따로
적지 않으므로 좌표와 거리가 어긋날 수 없다.
"""

import math
from pathlib import Path

import networkx as nx
import yaml


class ParkingMap:

    def __init__(self, graph: nx.Graph, meta: dict, zones: list):
        self.graph = graph
        self.meta = meta
        self.zones = zones

    @classmethod
    def load(cls, yaml_path) -> "ParkingMap":
        with open(Path(yaml_path)) as f:
            data = yaml.safe_load(f)

        graph = nx.Graph()
        for node_id, attrs in data["nodes"].items():
            graph.add_node(node_id, **attrs)
        for edge in data["edges"]:
            u, v = edge["u"], edge["v"]
            for n in (u, v):
                if n not in graph:
                    raise ValueError(f"엣지 {u}-{v}가 미정의 노드 {n}를 참조합니다")
            dist = math.hypot(
                graph.nodes[u]["x"] - graph.nodes[v]["x"],
                graph.nodes[u]["y"] - graph.nodes[v]["y"],
            )
            graph.add_edge(u, v, dist=dist, zone=edge.get("zone"))
        return cls(graph, data["meta"], data["zones"])

    def node_pos(self, node_id) -> tuple:
        n = self.graph.nodes[node_id]
        return (n["x"], n["y"])

    def nodes_of_kind(self, kind) -> list:
        return [n for n, a in self.graph.nodes(data=True) if a.get("kind") == kind]

    def nearest_node(self, x, y, kind=None) -> str:
        """좌표에서 가장 가까운 노드. kind 지정 시 해당 종류만 대상."""
        candidates = self.nodes_of_kind(kind) if kind else list(self.graph.nodes)
        return min(candidates,
                   key=lambda n: math.hypot(self.graph.nodes[n]["x"] - x,
                                            self.graph.nodes[n]["y"] - y))

    def edge_zones(self, path: list) -> list:
        """노드 경로가 통과하는 존 목록 (순서 유지, 중복 제거)."""
        seen = []
        for u, v in zip(path, path[1:]):
            zone = self.graph.edges[u, v].get("zone")
            if zone and zone not in seen:
                seen.append(zone)
        return seen
