import math

import FreeCAD as App
import UtilsAssembly

from freecad.assembly2mujoco.constants import (
    WORKBENCH_NAME,
    MUJOCO_JOINT_TYPE,
    JOINT_TYPE_MAPPING,
    DEFAULT_JOINT_TYPE_WEIGHTS,
)
from freecad.assembly2mujoco.core.graph import Graph
from freecad.assembly2mujoco.utils.helpers import log_message
from freecad.assembly2mujoco.utils.types import AppearanceDict, MaterialProperties


__all__ = ["AssemblyGraph", "AssemblyGraphNode", "AssemblyGraphEdge"]


class AssemblyGraphNode:
    def __init__(self, part: App.DocumentObject, *, is_grounded: bool = False) -> None:
        if not isinstance(part, App.DocumentObject):
            raise RuntimeError(
                f"part must be an instance of 'App.DocumentObject' instead of '{type(part)}'"
            )
        super().__init__()
        self.part = part
        self.is_grounded = is_grounded

    def get_body_position_and_orientation(self) -> tuple[str, str]:
        """Get position and orientation for FreeCAD part in MuJoCo."""
        pos = "0 0 0"
        quat = "1.0 0.0 0.0 0.0"

        log_message(f"Part: Name={self.label}, Pos={pos}, Quat={quat}")
        return pos, quat

    def get_body_material(
        self,
    ) -> MaterialProperties:
        material_properties: MaterialProperties = self.part.ShapeMaterial.Properties
        return material_properties

    def get_body_appearance(
        self,
    ) -> AppearanceDict:
        name = self.label
        rgb = self.part.ViewObject.ShapeAppearance[0].DiffuseColor[:3]
        rgba = rgb + (1.0,)
        rgba = " ".join(str(x) for x in rgba)
        shininess = str(self.part.ViewObject.ShapeAppearance[0].Shininess)
        appearance_dict = AppearanceDict(name=name, rgba=rgba, shininess=shininess)
        return appearance_dict

    @property
    def name(self) -> str:
        return self.part.Name

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
        return self.part == other.part

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, AssemblyGraphNode):
            raise RuntimeError(
                f"Can't compare object of type '{type(self)}' with object of type '{type(other)}'"
            )
        return self.name < other.name


class AssemblyGraphEdge:
    def __init__(
        self,
        joint: App.DocumentObject,
        *,
        parent_node: AssemblyGraphNode,
        child_node: AssemblyGraphNode,
        weight: float,
    ) -> None:
        if not isinstance(joint, App.DocumentObject):
            raise RuntimeError(
                f"joint must be an instance of 'App.DocumentObject' instead of '{type(joint)}'"
            )

        self.parent_node = parent_node
        self.child_node = child_node
        self.weight = weight
        self.joint = joint
        self.is_joint = hasattr(self.joint, "JointType")

        if not self.is_joint:
            raise RuntimeError(f"Object {self.label} is not a joint")

    def get_mujoco_joint_type(self) -> MUJOCO_JOINT_TYPE | None:
        # Grounded joint are handled differently from other joints
        if self.is_joint and self.joint.JointType == "Fixed":
            return None

        if self.joint.JointType not in JOINT_TYPE_MAPPING:
            raise NotImplementedError(
                f"Getting MuJoCo joint type not implemented for joint '{self.label}' of type '{self.joint.JointType}'"
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

        log_message(f"Joint: Name={self.label}, Pos={pos_vector}, Axis={axis_vector}")
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

    @property
    def name(self) -> str:
        return self.joint.Name

    @property
    def label(self) -> str:
        return self.joint.Label

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AssemblyGraphEdge):
            return False
        return self.joint == other.joint

    def __hash__(self):
        return hash((self.name, self.joint.JointType))


class AssemblyGraph(Graph[AssemblyGraphNode, AssemblyGraphEdge]):
    def __init__(
        self,
        *,
        is_directed: bool = False,
    ) -> None:
        super().__init__(is_directed=is_directed)

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
            edge = AssemblyGraphEdge(
                joint, parent_node=node1, child_node=node2, weight=weight
            )
            graph.add_edge(edge)

        # Then get all disconnected parts that are still part of the assembly
        for object in assembly.OutList:
            if object.TypeId == "PartDesign::Body" or UtilsAssembly.isLink(object):
                node = AssemblyGraphNode(
                    object, is_grounded=assembly.isPartGrounded(object)
                )
                graph.add_node(node)

        # Sanity checks
        graph_nodes = graph.get_nodes()
        unique_part_names = set(x.name for x in graph_nodes)
        if len(graph_nodes) != len(unique_part_names):
            raise RuntimeError(
                f"Sanity check failed. Number of created graph nodes, '{len(graph_nodes)}', "
                f"is different from number of unique part names, '{len(unique_part_names)}'"
            )

        return graph

    def update_edge_weights(self, joint_type_weights: dict[str, float]) -> None:
        """Update edge weights using provided joint type weights"""
        for *_, edge in self.get_edges():
            edge = edge
            edge.weight = joint_type_weights.get(edge.joint.JointType, 100.0)

    def get_possible_root_nodes(self) -> list[AssemblyGraphNode]:
        """Get list of grounded parts to use as possible root nodes for the subgraphs"""
        possible_root_nodes = [u for u in self.get_nodes() if u.is_grounded]
        return possible_root_nodes
