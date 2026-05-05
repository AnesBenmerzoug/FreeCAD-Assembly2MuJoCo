from queue import Queue
from typing import Generator

from freecad.assembly2mujoco.constants import (
    WORKBENCH_NAME,
)
from freecad.assembly2mujoco.core.assembly import (
    AssemblyGraph,
    AssemblyGraphNode,
    AssemblyGraphEdge,
)

__all__ = ["depth_first_traversal", "get_disconnected_subgraphs"]


def depth_first_traversal(
    graph: AssemblyGraph, *, root_node: AssemblyGraphNode
) -> Generator[
    tuple[AssemblyGraphNode, AssemblyGraphNode | None, AssemblyGraphEdge | None],
    None,
    None,
]:
    seen_edges: set[AssemblyGraphEdge] = set()
    queue: Queue[AssemblyGraphNode] = Queue()
    queue.put(root_node)

    while not queue.empty():
        current_node = queue.get()

        current_node_neighbors = graph.get_neighbors(current_node)
        if len(current_node_neighbors) == 0:
            yield current_node, None, None
            continue

        for next_node in current_node_neighbors:
            edge = graph.get_edge(current_node, next_node)
            if edge in seen_edges:
                continue
            seen_edges.add(edge)
            yield current_node, next_node, edge
            queue.put(next_node)


def get_disconnected_subgraphs(graph: AssemblyGraph) -> list["AssemblyGraph"]:
    """Splits graph into disconnected subgraphs"""
    # Get list of nodes to use
    # as possible root nodes for the subgraphs
    possible_root_nodes = graph.get_possible_root_nodes()
    # Get all of the graph's nodes
    remaining_nodes = [
        node for node in graph.get_nodes() if node not in possible_root_nodes
    ]
    # Add the possible root nodes at the end so that we can pop them out first
    remaining_nodes += possible_root_nodes
    subgraphs = []

    while len(remaining_nodes) > 0:
        # Breadth first traversal
        subgraph = AssemblyGraph()

        for current_node, next_node, edge in depth_first_traversal(
            graph, root_node=remaining_nodes.pop()
        ):
            if current_node in remaining_nodes:
                remaining_nodes.remove(current_node)
            subgraph.add_node(current_node)

            if next_node is None or edge is None:
                continue

            if next_node in remaining_nodes:
                remaining_nodes.remove(next_node)
            subgraph.add_node(next_node)

            subgraph.add_edge(edge=edge, parent_node=current_node, child_node=next_node)

        subgraphs.append(subgraph)

    return subgraphs


####################################################################
# Minimum Spanning Tree
####################################################################


class UnionFind:
    def __init__(self, graph: AssemblyGraph) -> None:
        # Initialize disjoint set for Kruskal's algorithm
        self.parent: dict[AssemblyGraphNode, AssemblyGraphNode] = {
            node: node for node in graph.get_nodes()
        }
        self.rank: dict[AssemblyGraphNode, int] = {
            node: 0 for node in graph.get_nodes()
        }

    def find_root(self, node: AssemblyGraphNode) -> AssemblyGraphNode:
        if self.parent[node] != node:
            # Path compression
            self.parent[node] = self.find_root(self.parent[node])
        return self.parent[node]

    def union(self, node1: AssemblyGraphNode, node2: AssemblyGraphNode) -> bool:
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
    graph: AssemblyGraph, *, root_node: AssemblyGraphNode
) -> tuple[
    AssemblyGraph, list[tuple[AssemblyGraphNode, AssemblyGraphNode, AssemblyGraphEdge]]
]:
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

    # Find minimum spanning tree representing kinematic tree
    # As well as unused edges (joints) that will be converted to equality constraints
    tree, unused_edges = find_minimum_spanning_tree(graph)

    # Convert resulting tree to a directed tree
    directed_tree = AssemblyGraph(is_directed=True)
    for parent_node, child_node, edge in depth_first_traversal(
        tree, root_node=root_node
    ):
        directed_tree.add_node(parent_node)
        if child_node is None or edge is None:
            continue

        directed_tree.add_node(child_node)
        directed_tree.add_edge(edge, parent_node=parent_node, child_node=child_node)

    return directed_tree, unused_edges


def find_minimum_spanning_tree(
    graph: AssemblyGraph,
) -> tuple[
    AssemblyGraph, list[tuple[AssemblyGraphNode, AssemblyGraphNode, AssemblyGraphEdge]]
]:
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
    tree = AssemblyGraph()
    unused_edges: list[
        tuple[AssemblyGraphNode, AssemblyGraphNode, AssemblyGraphEdge]
    ] = []
    for u, v, edge in sorted_edges:
        if uf.union(u, v):
            tree.add_edge(edge, parent_node=u, child_node=v)
        else:
            unused_edges.append((u, v, edge))

    return tree, unused_edges
