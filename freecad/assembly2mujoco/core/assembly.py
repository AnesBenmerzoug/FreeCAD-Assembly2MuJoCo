import math
from ast import literal_eval
from collections import defaultdict

import FreeCAD as App
import UtilsAssembly

from freecad.assembly2mujoco.constants import (
    WORKBENCH_NAME,
    MUJOCO_JOINT_TYPE,
    JOINT_TYPE_MAPPING,
    DEFAULT_JOINT_TYPE_WEIGHTS,
)
from freecad.assembly2mujoco.utils.helpers import log_message
from freecad.assembly2mujoco.utils.types import AppearanceDict


__all__ = ["AssemblyGraph", "AssemblyGraphNode", "AssemblyGraphEdge"]


class AssemblyGraphNode:
    def __init__(self, part: App.DocumentObject, *, is_grounded: bool = False) -> None:
        if not isinstance(part, App.DocumentObject):
            raise RuntimeError(
                f"{WORKBENCH_NAME}: part must be an instance of 'App.DocumentObject' instead of '{type(part)}'"
            )
        super().__init__()
        self.part = part
        self.is_grounded = is_grounded
        self.pos = "0 0 0"
        self.quat = "1.0 0.0 0.0 0.0"

    @property
    def body_appearance(self) -> AppearanceDict:
        if App.GuiUp:
            diffuse_color = self.part.ViewObject.ShapeAppearance[0].DiffuseColor
            shininess = str(self.part.ViewObject.ShapeAppearance[0].Shininess)
            rgba = f"{diffuse_color[0]} {diffuse_color[1]} {diffuse_color[2]} 1.0"
        else:
            log_message(
                "Gui is not running. Using shape material appearance properties.",
                level="warning",
            )
            rgb: tuple[float, float, float] = literal_eval(
                self.part.ShapeMaterial.AppearanceProperties["DiffuseColor"]
            )[:3]
            rgba = " ".join(str(x) for x in rgb + (1.0,))
            shininess = self.part.ShapeMaterial.AppearanceProperties["Shininess"]
        appearance_dict = AppearanceDict(
            name=self.label, rgba=rgba, shininess=shininess
        )
        return appearance_dict

    @property
    def label(self) -> str:
        return self.part.Label

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} part={self.label}>"

    def __hash__(self):
        return hash((self.part, self.is_grounded))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AssemblyGraphNode):
            return False
        return self.label == other.label

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, AssemblyGraphNode):
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Can't compare object of type '{type(self)}' with object of type '{type(other)}'"
            )
        return self.label < other.label


class AssemblyGraphEdge:
    def __init__(
        self,
        joint: App.DocumentObject,
        *,
        weight: float,
    ) -> None:
        if not isinstance(joint, App.DocumentObject):
            raise RuntimeError(
                f"{WORKBENCH_NAME}: joint must be an instance of 'App.DocumentObject' instead of '{type(joint)}'"
            )

        self.weight = weight
        self.joint = joint
        self.is_joint = hasattr(self.joint, "JointType")

        if not self.is_joint:
            raise RuntimeError(f"{WORKBENCH_NAME}: Object {self.label} is not a joint")

    @property
    def mujoco_joint_type(self) -> MUJOCO_JOINT_TYPE | None:
        # Grounded joint are handled differently from other joints
        if self.is_joint and self.joint.JointType == "Fixed":
            return None

        if self.joint.JointType not in JOINT_TYPE_MAPPING:
            raise NotImplementedError(
                f"{WORKBENCH_NAME}: Getting MuJoCo joint type not implemented for joint '{self.label}' of type '{self.joint.JointType}'"
            )

        mujoco_joint_type = JOINT_TYPE_MAPPING[self.joint.JointType]
        return mujoco_joint_type

    @property
    def joint_position_and_axis(self) -> tuple[App.Vector, App.Vector]:
        """Extract joint position and axis from FreeCAD joint"""
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

        elif self.joint.JointType == "Cylindrical":
            # Cylindrical joint combines rotation and translation along the same axis
            # The Z-axis of the placement is the axis for both rotation and translation
            pos_vector = global_plc.Base
            axis_vector = global_plc.Rotation.multVec(App.Vector(0, 0, 1))

        else:
            raise NotImplementedError(
                f"{WORKBENCH_NAME}: Getting joint axis not implemented for joint type: {self.joint.JointType}"
            )

        # Convert mm to m
        pos_vector = pos_vector / 1000
        # Normalize axis
        axis_vector = axis_vector.normalize()
        return pos_vector, axis_vector

    @property
    def is_cylindrical(self) -> bool:
        """Check if this is a cylindrical joint"""
        return self.joint.JointType == "Cylindrical"

    @property
    def joint_range(self) -> str | None:
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

    @property
    def label(self) -> str:
        return self.joint.Label

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AssemblyGraphEdge):
            return False
        return self.label == other.label

    def __hash__(self):
        return hash((self.label, self.joint.JointType))

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} joint={self.label}>"


class AssemblyGraph:
    def __init__(
        self,
        *,
        is_directed: bool = False,
    ) -> None:
        self.is_directed = is_directed
        self.adjacency_list: dict[
            AssemblyGraphNode, dict[AssemblyGraphNode, AssemblyGraphEdge]
        ] = defaultdict(dict)

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} directed={self.is_directed} n_nodes={len(self.get_nodes())} n_edges={len(self.get_edges())}>"

    @classmethod
    def from_assembly(
        cls,
        assembly: App.DocumentObject,
        joint_type_weights: dict[str, float] = DEFAULT_JOINT_TYPE_WEIGHTS,
    ) -> "AssemblyGraph":
        """Construct graph from FreeCAD assembly"""
        graph = cls()
        # First add all parts connected by joints
        joint_group = UtilsAssembly.getJointGroup(assembly)
        for joint in joint_group.Group:
            # Grounded Joint will be set as the root of the graph
            # and we don't create a graph joint for it
            if hasattr(joint, "ObjectToGround"):
                node = AssemblyGraphNode(joint.ObjectToGround, is_grounded=True)
                graph.add_node(node)
                continue

            part1 = UtilsAssembly.getMovingPart(assembly, joint.Reference1)
            part2 = UtilsAssembly.getMovingPart(assembly, joint.Reference2)
            node1 = AssemblyGraphNode(part1, is_grounded=assembly.isPartGrounded(part1))
            node2 = AssemblyGraphNode(part2, is_grounded=assembly.isPartGrounded(part2))
            # Assign weights to prioritize which joints to keep in the tree                 ..
            # Higher weight are more likely to be excluded from tree                        ..
            weight = joint_type_weights.get(joint.JointType, 100.0)
            edge = AssemblyGraphEdge(joint=joint, weight=weight)
            graph.add_edge(edge=edge, parent_node=node1, child_node=node2)

        # Then get all disconnected parts that are still part of the assembly
        for object in assembly.OutList:
            if object.TypeId == "PartDesign::Body" or UtilsAssembly.isLink(object):
                node = AssemblyGraphNode(
                    object, is_grounded=assembly.isPartGrounded(object)
                )
                graph.add_node(node)

        # Sanity checks
        graph_nodes = graph.get_nodes()
        unique_part_names = set(x.label for x in graph_nodes)
        if len(graph_nodes) != len(unique_part_names):
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Sanity check failed. Number of created graph nodes, '{len(graph_nodes)}', "
                f"is different from number of unique part names, '{len(unique_part_names)}'"
            )

        return graph

    def add_node(self, node: AssemblyGraphNode) -> AssemblyGraphNode:
        if node not in self.adjacency_list:
            self.adjacency_list[node] = {}
        return node

    def add_edge(
        self,
        edge: AssemblyGraphEdge,
        *,
        parent_node: AssemblyGraphNode,
        child_node: AssemblyGraphNode,
    ) -> None:
        self.adjacency_list[parent_node][child_node] = edge
        if not self.is_directed:
            # Since undirected, add both directions
            self.adjacency_list[child_node][parent_node] = edge

    def get_nodes(self) -> list[AssemblyGraphNode]:
        """Return a list of all unique nodes."""
        return list(self.adjacency_list.keys())

    def get_neighbors(self, node: AssemblyGraphNode) -> list[AssemblyGraphNode]:
        return list(self.adjacency_list.get(node, []))

    def get_edge(
        self, parent: AssemblyGraphNode, child: AssemblyGraphNode
    ) -> AssemblyGraphEdge:
        try:
            return self.adjacency_list[parent][child]
        except KeyError:
            raise RuntimeError(
                f"{WORKBENCH_NAME}: Did not find edge between node '{parent}' and node '{child}'"
            )

    def get_edges(
        self,
    ) -> list[tuple[AssemblyGraphNode, AssemblyGraphNode, AssemblyGraphEdge]]:
        """Return a list of all unique edges as (parent, child, edge)."""
        seen: set[tuple[AssemblyGraphNode, AssemblyGraphNode]] = set()
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

    def update_edge_weights(self, joint_type_weights: dict[str, float]) -> None:
        """Update edge weights using provided joint type weights"""
        for *_, edge in self.get_edges():
            edge = edge
            edge.weight = joint_type_weights.get(edge.joint.JointType, 100.0)

    def get_possible_root_nodes(self) -> list[AssemblyGraphNode]:
        """Get list of grounded parts to use as possible root nodes for the subgraphs"""
        possible_root_nodes = [u for u in self.get_nodes() if u.is_grounded]
        return possible_root_nodes
