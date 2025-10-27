from queue import Queue
from typing import Generic, Protocol, TypeVar

from freecad.assembly2mujoco.constants import (
    WORKBENCH_NAME,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["GraphNode", "GraphEdge", "Graph"]


class GraphNode(Protocol):
    @property
    def name(self) -> str: ...

    def __hash__(self) -> int: ...

    def __eq__(self, other: object) -> bool: ...

    def __lt__(self, other: object) -> bool: ...

    def __repr__(self) -> str: ...


NodeType = TypeVar("NodeType", bound=GraphNode)


class GraphEdge(Protocol[NodeType]):
    parent_node: NodeType
    child_node: NodeType
    weight: float

    @property
    def name(self) -> str: ...

    def __repr__(self) -> str: ...

    def __hash__(self) -> int: ...


EdgeType = TypeVar("EdgeType", bound=GraphEdge)


class Graph(Generic[NodeType, EdgeType]):
    def __init__(
        self,
        *,
        is_directed: bool = False,
    ) -> None:
        self.is_directed = is_directed
        self.adjacency_list: dict[NodeType, dict[NodeType, EdgeType]] = {}

    def add_node(
        self,
        node: NodeType,
    ) -> NodeType:
        if node not in self.adjacency_list:
            self.adjacency_list[node] = {}
        return node

    def add_edge(self, edge: EdgeType) -> None:
        self.add_node(edge.parent_node)
        self.add_node(edge.child_node)
        self.adjacency_list[edge.parent_node][edge.child_node] = edge
        if not self.is_directed:
            # Since undirected, add both directions
            self.adjacency_list[edge.child_node][edge.parent_node] = edge

    def get_nodes(self) -> list[NodeType]:
        """Return a list of all unique nodes."""
        return list(self.adjacency_list.keys())

    def get_neighbors(self, node: NodeType) -> list[NodeType]:
        return list(self.adjacency_list.get(node, []))

    def get_edge(self, u: NodeType, v: NodeType) -> EdgeType:
        try:
            return self.adjacency_list[u][v]
        except KeyError:
            raise RuntimeError(f"Did not find edge between node '{u}' and node '{v}'")

    def get_edges(
        self,
    ) -> list[tuple[NodeType, NodeType, EdgeType]]:
        """Return a list of all unique edges as (u, v, edge)."""
        seen: set[tuple[NodeType, NodeType]] = set()
        edge_list = []
        for u in self.adjacency_list:
            for v in self.adjacency_list[u]:
                if self.is_directed or u < v:
                    edge_key = (u, v)
                else:
                    edge_key = (v, u)
                if edge_key not in seen:
                    edge = self.get_edge(edge_key[0], edge_key[1])
                    edge_list.append((edge_key[0], edge_key[1], edge))
                    seen.add(edge_key)
        return edge_list

    def get_possible_root_nodes(self) -> list[NodeType]:
        """Dummy implementation that returns all nodes"""
        return self.get_nodes()

    def get_disconnected_subgraphs(self) -> list["Graph[NodeType, EdgeType]"]:
        """Splits graph into disconnected subgraphs"""
        # Get list of nodes to use
        # as possible root nodes for the subgraphs
        possible_root_nodes = self.get_possible_root_nodes()
        log_message(f"Possible root nodes: {[x.label for x in possible_root_nodes]}")
        # Get all of the graph's nodes
        remaining_nodes = [
            node for node in self.get_nodes() if node not in possible_root_nodes
        ]
        # Add the possible root nodes at the end so that we can pop them out first
        remaining_nodes += possible_root_nodes
        subgraphs = []

        while len(remaining_nodes) > 0:
            # Breadth first traversal
            subgraph = Graph[NodeType, EdgeType]()
            seen_nodes: set[NodeType] = set()
            queue: Queue[NodeType] = Queue()
            queue.put(remaining_nodes.pop())

            while not queue.empty():
                current_node = queue.get()
                if current_node in seen_nodes:
                    continue

                subgraph.add_node(current_node)
                seen_nodes.add(current_node)

                if current_node in remaining_nodes:
                    remaining_nodes.remove(current_node)

                log_message(f"Current Node: {current_node.label}")
                log_message(
                    f"Neighbors of current Node: {[x.label for x in self.get_neighbors(current_node)]}"
                )
                for next_node in self.get_neighbors(current_node):
                    edge = self.get_edge(current_node, next_node)
                    if edge is None:
                        raise RuntimeError("Expected edge to be not be None")

                    subgraph.add_edge(
                        edge,
                    )
                    queue.put(next_node)

                    if next_node in remaining_nodes:
                        remaining_nodes.remove(next_node)

            subgraphs.append(subgraph)

        return subgraphs

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} directed={self.is_directed} n_nodes={len(self.get_nodes())} n_edges={len(self.get_edges())}>"


####################################################################
# Minimum Spanning Tree
####################################################################


class UnionFind(Generic[NodeType, EdgeType]):
    def __init__(self, graph: Graph[NodeType, EdgeType]) -> None:
        # Initialize disjoint set for Kruskal's algorithm
        self.parent: dict[NodeType, NodeType] = {
            node: node for node in graph.get_nodes()
        }
        self.rank: dict[NodeType, int] = {node: 0 for node in graph.get_nodes()}

    def find_root(self, node: NodeType) -> NodeType:
        if self.parent[node] != node:
            # Path compression
            self.parent[node] = self.find_root(self.parent[node])
        return self.parent[node]

    def union(self, node1: NodeType, node2: NodeType) -> bool:
        """Union the sets containing node1 and node2 using union by rank."""

        root1 = self.find_root(node1)
        root2 = self.find_root(node2)

        if root1 == root2:
            # Cycle detected
            return False

        # Union by rank
        # Attach smaller rank tree under root of
        # high rank tree (Union by Rank)
        if self.rank[root1] < self.rank[root2]:
            self.parent[root1] = root2
        elif self.rank[root1] > self.rank[root2]:
            self.parent[root2] = root1
        # If ranks are same, then mark first one as root
        # and increment its rank by one
        else:
            self.parent[root2] = root1
            self.rank[root1] += 1

        return True


def convert_to_directed_tree(
    graph: Graph[NodeType, EdgeType], root_node: NodeType
) -> Graph[NodeType, EdgeType]:
    """Converts an undirected graph to a directed graph.

    Args:
        graph: Undirected graph.
        root_node: Starting node for directed graph.

    Returns:
        Directed graph.
    """
    if root_node not in graph.get_nodes():
        raise RuntimeError(
            f"{WORKBENCH_NAME}: Provided root_node, {root_node}, is not part of graph"
        )

    visited: set[NodeType] = set()
    directed_tree = Graph[NodeType, EdgeType](is_directed=True)

    def dfs(node: NodeType) -> None:
        visited.add(node)
        for neighbor in graph.get_neighbors(node):
            if neighbor not in visited:
                # Get edge from undirected graph
                edge = graph.get_edge(node, neighbor)
                # Add only one direction
                directed_tree.add_edge(edge)
                dfs(neighbor)

    dfs(root_node)
    return directed_tree


def find_minimum_spanning_tree(
    graph: Graph[NodeType, EdgeType],
) -> tuple[Graph[NodeType, EdgeType], list[tuple[NodeType, NodeType, EdgeType]]]:
    """Builds minimum spanning tree using Kruskal's algorithm.

    Returns:
        tree_edges: List of edges representing the minimum spanning tree.
        unused_edges: List of unused edges that would form loops.
    """
    uf = UnionFind(graph)

    # sort edges in non-decreasing order of weights
    sorted_edges = sorted(graph.get_edges(), key=lambda e: e[2].weight)

    # Track which edges are used in the tree
    # and which ones are not
    tree = Graph[NodeType, EdgeType]()
    unused_edges: list[tuple[NodeType, NodeType, EdgeType]] = []
    for u, v, edge in sorted_edges:
        if uf.union(u, v):
            tree.add_edge(edge)
        else:
            unused_edges.append((u, v, edge))

    return tree, unused_edges
