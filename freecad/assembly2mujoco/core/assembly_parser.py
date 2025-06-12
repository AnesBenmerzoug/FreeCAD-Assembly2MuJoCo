import math
from typing import Any

import FreeCAD as App
import UtilsAssembly

from freecad.assembly2mujoco.constants import (
    WORKBENCH_NAME,
    MUJOCO_JOINT_TYPE,
    JOINT_TYPE_MAPPING,
)
from freecad.assembly2mujoco.utils.helpers import log_message
from freecad.assembly2mujoco.utils.types import AppearanceDict, MaterialProperties


__all__ = ["Graph"]


class GraphNode:
    def __init__(
        self,
        part: App.DocumentObject,
    ) -> None:
        self.part = part

    def get_body_position_and_orientation(self) -> tuple[str, str]:
        """Get position and orientation for FreeCAD part in MuJoCo."""
        pos = "0 0 0"
        quat = "1.0 0.0 0.0 0.0"

        log_message(f"Part: Name={self.part.Name}, Pos={pos}, Quat={quat}")
        return pos, quat

    def get_body_material(
        self,
    ) -> MaterialProperties:
        material_properties: MaterialProperties = self.part.ShapeMaterial.Properties
        return material_properties

    def get_body_appearance(
        self,
    ) -> AppearanceDict:
        name = self.part.Name
        rgb = self.part.ViewObject.ShapeAppearance[0].DiffuseColor[:3]
        rgba = rgb + (1.0,)
        rgba = " ".join(str(x) for x in rgba)
        shininess = str(self.part.ViewObject.ShapeAppearance[0].Shininess)
        appearance_dict = AppearanceDict(name=name, rgba=rgba, shininess=shininess)
        return appearance_dict

    def __repr__(self) -> str:
        return f"<AssemblyGraphNode part={self.part.Name}>"

    def __hash__(self):
        return hash(repr(self))

    def __eq__(self, other: "GraphNode") -> bool:
        return self.part.Name == other.part.Name


class GraphEdge:
    def __init__(
        self,
        joint: App.DocumentObject,
        *,
        parent_node: GraphNode,
        child_node: GraphNode,
        weight: float,
    ) -> None:
        self.joint = joint
        self.parent_node = parent_node
        self.child_node = child_node
        self.weight = weight

    def get_mujoco_joint_type(self) -> MUJOCO_JOINT_TYPE | None:
        if not (
            hasattr(self.joint, "ObjectToGround") or hasattr(self.joint, "JointType")
        ):
            raise RuntimeError(f"Object {self.joint.Name} is not a joint")

        if hasattr(self.joint, "ObjectToGround"):
            return None

        if self.joint.JointType not in JOINT_TYPE_MAPPING:
            raise NotImplementedError(
                f"Getting MuJoCo joint type not implemented for joint '{self.joint.Name}' of type '{self.joint.JointType}'"
            )

        mujoco_joint_type = JOINT_TYPE_MAPPING[self.joint.JointType]
        return mujoco_joint_type

    def get_joint_position_and_axis(self) -> tuple[App.Vector, App.Vector]:
        """Extract joint position and axis from FreeCAD joint"""
        assembly = self.parent_node.part.Parents[0][0]
        if assembly.Type != "Assembly":
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Unexpected error trying to get root assembly from part"
            )

        # Get global placement of joint
        global_plc = UtilsAssembly.getJcsGlobalPlc(
            self.joint.Placement1, self.joint.Reference1
        )

        if self.joint.JointType == "Revolute":
            pos_vector = global_plc.Base
            # For a Revolute joint, the Z-axis of the placement is the rotation axis
            # Transform the Z-axis (0,0,1) by the rotation part of the placement
            axis_vector = global_plc.Rotation.multVec(App.Vector(0, 0, 1))
            # axis_vector = UtilsAssembly.round_vector(axis_vector)

        elif self.joint.JointType == "Slider":
            pos_vector = global_plc.Base
            # For a Slider joint, the Z-axis of the placement is typically the sliding direction
            # Transform the Z-axis (0,0,1) by the rotation part of the placement
            axis_vector = global_plc.Rotation.multVec(App.Vector(0, 0, 1))
            # Note: Some FreeCAD assemblies might use X-axis (1,0,0) for sliding direction
            # You may need to adjust this based on your FreeCAD assembly convention:
            # axis_vector = global_plc.Rotation.multVec(App.Vector(1, 0, 0))

        else:
            raise NotImplementedError(
                f"Getting joint axis not implemented for joint type: {self.joint.JointType}"
            )

        # Convert mm to m
        pos_vector = pos_vector / 1000
        # Normalize axis
        axis_vector = axis_vector.normalize()

        log_message(
            f"Joint: Name={self.joint.Name}, Pos={pos_vector}, Axis={axis_vector}"
        )
        return pos_vector, axis_vector

    def get_joint_range(self) -> str | None:
        """Extract joint range from a FreeCAD joint limits, if there are any."""
        limits = {"lower": None, "upper": None}

        # Try to get limits from joint
        if self.joint.EnableAngleMin:
            # Convert degrees to radian
            limits["lower"] = self.joint.AngleMin * math.pi / 180
        elif self.joint.EnableLengthMin:
            limits["lower"] = self.joint.LengthMin

        if self.joint.EnableAngleMax:
            limits["upper"] = self.joint.AngleMax * math.pi / 180
        elif self.joint.EnableLengthMax:
            limits["upper"] = self.joint.LengthMax

        # Calculate range if both limits are defined
        range: str | None = None
        if limits["lower"] is not None and limits["upper"] is not None:
            range = f"{limits['lower']} {limits['upper']}"

        return range

    def __eq__(self, other: "GraphEdge") -> bool:
        return self.joint == other.joint

    def __repr__(self) -> str:
        return f"<AssemblyGraphEdge joint={self.joint.JointType} weight={self.weight}>"

    def __hash__(self):
        return hash((self.joint.Name, self.joint.JointType))


class Graph:
    def __init__(
        self,
        *,
        is_directed: bool = False,
    ) -> None:
        self.is_directed = is_directed
        self.adjacency_list: dict[GraphNode, dict[GraphNode, GraphEdge]] = {}

    @classmethod
    def from_assembly(
        cls, assembly: App.DocumentObject, joint_type_weights: dict[str, float]
    ) -> None:
        """Construct graph from FreeCAD assembly"""
        graph = cls()
        joint_group = UtilsAssembly.getJointGroup(assembly)
        for joint in joint_group.Group:
            # Grounded Joint will be set as the root of the graph
            if hasattr(joint, "ObjectToGround"):
                graph.add_node(joint.ObjectToGround)
            else:
                part1 = UtilsAssembly.getMovingPart(assembly, joint.Reference1)
                part2 = UtilsAssembly.getMovingPart(assembly, joint.Reference2)
                # Assign weights to prioritize which joints to keep in the tree                 ..
                # Higher weight are more likely to be excluded from tree                        ..
                weight = joint_type_weights.get(joint.JointType, 100.0)
                graph.add_edge(part1, part2, joint, weight=weight)
        return graph

    def add_node(self, part: App.DocumentObject) -> GraphNode:
        node = GraphNode(part)
        if node not in self.adjacency_list:
            self.adjacency_list[node] = {}
        return node

    def add_edge(
        self,
        part1: App.DocumentObject,
        part2: App.DocumentObject,
        joint: App.DocumentObject,
        weight: float,
    ) -> None:
        node1 = self.add_node(part1)
        node2 = self.add_node(part2)
        edge = GraphEdge(joint, parent_node=node1, child_node=node2, weight=weight)

        self.adjacency_list[node1][node2] = edge
        if not self.is_directed:
            # Since undirected, add both directions
            self.adjacency_list[node2][node1] = edge

    def get_nodes(self) -> list[GraphNode]:
        """Return a list of all unique nodes."""
        return list(self.adjacency_list.keys())

    def get_neighbors(self, node: GraphNode) -> list[GraphNode]:
        return list(self.adjacency_list.get(node, []))

    def get_edge(self, u: GraphNode, v: GraphNode) -> GraphEdge | None:
        return self.adjacency_list.get(u, {}).get(v, None)

    def get_edges(
        self,
    ) -> list[tuple[GraphNode, GraphNode, dict[str, GraphEdge | Any]]]:
        """Return a list of all unique edges as (u, v, edge)."""
        seen: set[tuple[GraphNode, GraphNode]] = set()
        edge_list = []
        for u in self.adjacency_list:
            for v in self.adjacency_list[u]:
                if self.is_directed:
                    edge_key = (u, v)
                else:
                    edge_key = tuple(sorted((u, v), key=lambda x: x.part.Name))
                if edge_key not in seen:
                    edge = self.get_edge(edge_key[0], edge_key[1])
                    if edge is not None:
                        edge_list.append((edge_key[0], edge_key[1], edge))
                        seen.add(edge_key)
        return edge_list

    def __repr__(self) -> str:
        return f"<Graph directed={self.is_directed} n_nodes={len(self.get_nodes())} n_edges={len(self.get_edges())}>"


####################################################################
# Minimum Spanning Tree
####################################################################


class UnionFind:
    def __init__(self, graph: Graph) -> None:
        # Initialize disjoint set for Kruskal's algorithm
        self.parent: dict[GraphNode, GraphNode] = {
            node: node for node in graph.get_nodes()
        }
        self.rank: dict[GraphNode, int] = {node: 0 for node in graph.get_nodes()}

    def find_root(self, node: GraphNode) -> GraphNode:
        if self.parent[node] != node:
            # Path compression
            self.parent[node] = self.find_root(self.parent[node])
        return self.parent[node]

    def union(self, node1: GraphNode, node2: GraphNode) -> None:
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


def convert_to_directed_tree(graph: Graph, root_node: GraphNode | None = None) -> Graph:
    """Converts an undirected graph to a directed graph.

    If root_node is provided, the directed graph will start from that node.
    Otherwise, it will attempt to select as root_node the first node it finds with a single neighbor.

    Args:
        graph: Undirected graph.
        root_node: Optional starting node for directed graph.

    Returns:
        Directed graph.
    """
    if root_node is None:
        # Find root(s)
        root_nodes = [
            node for node in graph.get_nodes() if len(graph.get_neighbors(node)) == 1
        ]
        if not root_nodes:
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Could not find root node for assembly"
            )

        # Select first one as main root node
        root_node = root_nodes[0]
    else:
        if root_node not in graph.get_nodes():
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Provided root_node, {root_node.part.Name}, is not part of graph"
            )

    visited: set[GraphNode] = set()
    directed_tree = Graph(is_directed=True)

    def dfs(node: GraphNode) -> None:
        visited.add(node)
        for neighbor in graph.get_neighbors(node):
            if neighbor not in visited:
                # Get edge from undirected graph
                edge = graph.get_edge(node, neighbor)
                # Add only one direction
                directed_tree.add_edge(node.part, neighbor.part, edge.joint, weight=1.0)
                dfs(neighbor)

    dfs(root_node)
    return directed_tree


def find_minimum_spanning_tree(
    graph: Graph,
) -> tuple[Graph, list[tuple[GraphNode, GraphNode, GraphEdge]]]:
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
    tree = Graph()
    unused_edges: list[tuple[GraphNode, GraphNode, GraphEdge]] = []
    for u, v, edge in sorted_edges:
        if uf.union(u, v):
            tree.add_edge(u.part, v.part, edge.joint, weight=1.0)
        else:
            unused_edges.append((u, v, edge))

    return tree, unused_edges
