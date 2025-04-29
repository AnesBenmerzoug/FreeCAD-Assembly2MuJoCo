import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Literal, Optional

import FreeCAD as App
import FreeCADGui as Gui
import MeshPart
import UtilsAssembly


MACRO_NAME = "ExportToMujoco"

MUJOCO_JOINT_TYPE = Literal["hinge", "slide", "ball", "free"]

####################################################################
# Utilities
####################################################################


def log_message(
    message: str,
    *,
    level: Literal["info", "warning", "error"] = "info",
) -> None:
    formatted_message = MACRO_NAME + ": " + message + "\n"
    if level == "info":
        App.Console.PrintMessage(formatted_message)
    elif level == "warning":
        App.Console.PrintWarning(formatted_message)
    elif level == "error":
        App.Console.PrintError(formatted_message)
    else:
        raise ValueError(f"Unrecognized log level '{level}'")


####################################################################
# Assembly Graph Classes
####################################################################


class GraphNode:
    def __init__(
        self,
        part: App.DocumentObject,
    ) -> None:
        self.part = part

    def get_body_position_and_orientation(
        self, parent: Optional["GraphNode"] = None
    ) -> tuple[str, str]:
        """Extract position and orientation from FreeCAD part."""
        if parent is not None:
            # plc is self.placement relative to parent.placement
            plc = parent.part.Placement.inverse() * self.part.Placement
            pos = plc.Base
            quat = plc.Rotation.Q
        else:
            # No parent, use global position and orientation
            pos = self.part.Placement.Base
            quat = self.part.Placement.Rotation.Q

        # Convert mm to m
        pos = f"{pos.x / 1000} {pos.y / 1000} {pos.z / 1000}"
        quat = f"{quat[0]} {quat[1]} {quat[2]} {quat[3]}"
        print(f"Name = {self.part.Name}")
        print(f"{pos=}")
        print(f"{quat=}")
        print()
        return pos, quat

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
    ) -> None:
        self.joint = joint
        self.parent_node = parent_node
        self.child_node = child_node
        self.weight = self.compute_weight(self.joint)

    @staticmethod
    def compute_weight(joint) -> float:
        # Assign weights to prioritize which joints to keep in the tree
        # Higher weight are more likely to be excluded from tree
        # Base weight by joint type
        type_weights = {
            "Fixed": 10.0,
            "Revolute": 1.0,
            "Prismatic": 2.0,
            "Cylindrical": 3.0,
            "Ball": 5.0,
            "Planar": 8.0,
        }

        weight = type_weights.get(joint.JointType, 20.0)
        return weight

    def get_mujoco_joint_type(self) -> MUJOCO_JOINT_TYPE | None:
        joint_type_mapping: dict[str, MUJOCO_JOINT_TYPE] = {
            "Revolute": "hinge",
            "Prismatic": "slide",
            "Cylindrical": "hinge",
            "Ball": "ball",
            "Planar": "free",
        }
        mujoco_joint_type = joint_type_mapping.get(self.joint.JointType)
        return mujoco_joint_type

    def get_joint_position_and_axis(
        self,
    ) -> tuple[str, str]:
        """Extract joint position and axis from FreeCAD joint"""
        assembly = self.parent_node.part.Parents[0][0]
        if assembly.Type != "Assembly":
            raise RuntimeError(
                f"{MACRO_NAME}: Unexpected error trying to get root assembly from part"
            )

        # Get global placement of joint
        global_plc = UtilsAssembly.getJcsGlobalPlc(
            self.joint.Placement1, self.joint.Reference1
        )

        # Determine which joint reference is this edge's actual parent
        if (
            UtilsAssembly.getMovingPart(assembly, self.joint.Reference1)
            == self.parent_node.part
        ):
            parent_part = self.parent_node.part
        else:
            parent_part = self.child_node.part

        if self.joint.JointType == "Revolute":
            # relative_plc is joint_placement relative to parent_part.placement
            relative_plc = parent_part.Placement.inverse() * global_plc

            pos_vector = relative_plc.Base
            # For a Revolute joint, the Z-axis of the placement is the rotation axis
            # Transform the Z-axis (0,0,1) by the rotation part of the placement
            axis_vector = relative_plc.Rotation.multVec(App.Vector(0, 0, 1))
            # axis_vector = UtilsAssembly.round_vector(axis_vector)

        else:
            raise NotImplementedError(
                f"Getting joint axis not implemented for joint type: {self.joint.JointType}"
            )

        # Convert mm to m
        pos = f"{pos_vector[0] / 1000} {pos_vector[1] / 1000} {pos_vector[2] / 1000}"
        axis = f"{axis_vector[0]} {axis_vector[1]} {axis_vector[2]}"
        return pos, axis

    def __eq__(self, other: "GraphEdge") -> bool:
        return self.joint == other.joint

    def __repr__(self) -> str:
        return f"<AssemblyGraphEdge joint={self.joint.JointType} weight={self.weight}>"

    def __hash__(self):
        return hash((self.joint.Name, self.joint.JointType))


class Graph:
    def __init__(self, *, is_directed: bool = False) -> None:
        self.is_directed = is_directed
        self.adjacency_list: dict[GraphNode, dict[GraphNode, GraphEdge]] = {}

    @classmethod
    def from_assembly(cls, assembly: App.DocumentObject) -> None:
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
                graph.add_edge(part1, part2, joint)
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
    ) -> None:
        node1 = self.add_node(part1)
        node2 = self.add_node(part2)
        edge = GraphEdge(joint, parent_node=node1, child_node=node2)
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
            raise RuntimeError(f"{MACRO_NAME}: Could not find root node for assembly")

        # Select first one as main root node
        root_node = root_nodes[0]
    else:
        if root_node not in graph.get_nodes():
            raise RuntimeError(
                f"{MACRO_NAME}: Provided root_node, {root_node.part.Name}, is not part of graph"
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
                directed_tree.add_edge(node.part, neighbor.part, edge.joint)
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
            tree.add_edge(u.part, v.part, edge.joint)
        else:
            unused_edges.append((u, v, edge))

    return tree, unused_edges


####################################################################
# MuJuCo Exporter Class
####################################################################


class MuJuCoExporter:
    """Class for exporting a kinematic tree as a MuJoCo MJCF (XML) file and STL files.

    Args:
        integrator: The numerical integrator to be used in MuJoCo.
            The available integrators are the semi-implicit Euler method,
            the fixed-step 4-th order Runge Kutta method,
            the Implicit-in-velocity Euler method, and implicitfast.
        time_step: Simulation time step in seconds.
    """

    def __init__(
        self,
        integrator: Literal[
            "Euler", "implicit", "implicitfast", "RK4"
        ] = "implicitfast",
        timestep: float = 0.0001,
    ) -> None:
        self.mujoco = ET.Element("mujoco")
        self.option = ET.SubElement(
            self.mujoco, "option", integrator=integrator, timestep=str(timestep)
        )
        self.compiler = ET.SubElement(
            self.mujoco, "compiler", meshdir="meshes", autolimits="true"
        )
        self.asset = ET.SubElement(self.mujoco, "asset")
        ET.SubElement(
            self.asset,
            "texture",
            type="skybox",
            builtin="gradient",
            rgb1="0.3 0.5 0.7",
            rgb2="0 0 0",
            width="512",
            height="3072",
        )
        ET.SubElement(
            self.asset,
            "texture",
            type="2d",
            name="groundplane",
            builtin="checker",
            mark="edge",
            rgb1="0.2 0.3 0.4",
            rgb2="0.1 0.2 0.3",
            markrgb="0.8 0.8 0.8",
            width="300",
            height="300",
        )
        ET.SubElement(
            self.asset,
            "material",
            name="groundplane",
            texture="groundplane",
            texuniform="true",
            texrepeat="5 5",
            reflectance="0.1",
        )

        self.worldbody = ET.SubElement(self.mujoco, "worldbody")
        # Add multiple light sources for better visualization
        ET.SubElement(
            self.worldbody, "light", diffuse=".8 .8 .8", pos="0 0 4", dir="0 0 -1"
        )
        ET.SubElement(
            self.worldbody, "light", diffuse=".6 .6 .6", pos="4 4 4", dir="-1 -1 -1"
        )
        self.part_bodies: dict[str, Any] = {}
        # Placeholder for elements that will be added when exporting
        self.equality = ET.SubElement(self.mujoco, "equality")
        self.sensor = ET.SubElement(self.mujoco, "sensor")
        self.actuator = ET.SubElement(self.mujoco, "actuator")

    def export_assembly(
        self, assembly: App.DocumentObject, output_dir: str | os.PathLike
    ) -> None:
        """Main export method"""

        # Create graph connecting parts with joints
        assembly_graph = Graph.from_assembly(assembly)

        # Export assembly parts as binary stl meshes
        meshes_dir = Path(output_dir).joinpath("meshes")
        meshes_dir.mkdir(exist_ok=True, parents=True)
        self.export_parts_as_meshes_and_add_to_assets(assembly_graph, meshes_dir)

        # Add floorplane (cosmetic)
        self.add_floorplane(assembly_graph)

        # Find minimum spanning tree representing kinematic tree
        # As well as unused edges (joints) that will be converted to equality constraints
        tree, unused_edges = find_minimum_spanning_tree(assembly_graph)

        # Use grounded part as root node
        root_node: GraphNode | None = None
        grounded_joints = [
            joint
            for joint in UtilsAssembly.getJointGroup(assembly).Group
            if hasattr(joint, "ObjectToGround")
        ]
        if grounded_joints:
            grounded_joint = grounded_joints[0]
            grounded_part = grounded_joint.ObjectToGround
            # Find corresponding node
            root_node = [
                node for node in tree.get_nodes() if node.part == grounded_part
            ][0]

        # Build tree
        tree = convert_to_directed_tree(tree, root_node)
        if root_node is None:
            root_node = tree.get_nodes()[0]
        self.process_tree(root_node, tree)

        # Save MJCF file
        xml_file = (
            Path(output_dir).joinpath(App.activeDocument().Name).with_suffix(".xml")
        )
        self.write_xml(xml_file)

    def export_parts_as_meshes_and_add_to_assets(
        self, assembly_graph: Graph, meshes_dir: str | os.PathLike
    ) -> None:
        for node in assembly_graph.get_nodes():
            part = node.part
            shape = part.Shape.copy(False)
            # Reset position by creating a temporary placement at origin
            reset_placement = App.Placement()
            reset_placement.Base = App.Vector(0, 0, 0)
            reset_placement.Rotation = App.Rotation(
                0, 0, 0, 1
            )  # Identity rotation (w=1)
            shape.Placement = reset_placement

            mesh = MeshPart.meshFromShape(
                Shape=shape,
                LinearDeflection=0.1,
                AngularDeflection=5.0,
                Relative=False,
            )
            mesh_file = Path(meshes_dir).joinpath(part.Name).with_suffix(".stl")
            mesh.write(os.fspath(mesh_file))
            # Add new mesh
            ET.SubElement(
                self.asset,
                "mesh",
                name=part.Name,
                file=mesh_file.name,
                # Convert mm to m
                scale="0.001 0.001 0.001",
            )

    def add_floorplane(self, assembly_graph: Graph) -> None:
        minimum_z_placement: float | None = None
        for node in assembly_graph.get_nodes():
            part = node.part
            if minimum_z_placement is None:
                minimum_z_placement = part.Placement.Base[2]
            else:
                minimum_z_placement = min(minimum_z_placement, part.Placement.Base[2])
        if minimum_z_placement is None:
            raise RuntimeError(f"{MACRO_NAME}: This should not happen")
        # convert from mm to m
        minimum_z_placement *= 0.001
        # shift placement to account for floorplane size
        floor_z_pos = minimum_z_placement - 0.001
        # Add floorplane under lowest part
        ET.SubElement(
            self.worldbody,
            "geom",
            name="floor",
            pos=f"0 0 {floor_z_pos}",
            size="0 0 1",
            type="plane",
            material="groundplane",
        )

    def process_tree(
        self,
        current_node: GraphNode,
        tree: Graph,
        *,
        parent_node: ET.Element | None = None,
        body_elements: dict[GraphNode, ET.Element] | None = None,
        added_joint: int = 0,
    ) -> ET.Element:
        if body_elements is None:
            body_elements = {}
        parent_body = body_elements.get(parent_node, self.worldbody)

        if (body := body_elements.get(current_node)) is None:
            body = self.add_body(current_node, parent_node, parent_body)
            body_elements[current_node] = body

        for child_node in tree.get_neighbors(current_node):
            child_body = self.process_tree(
                child_node,
                tree,
                parent_node=current_node,
                body_elements=body_elements,
                added_joint=added_joint,
            )
            edge = tree.get_edge(current_node, child_node)
            if added_joint < 1:
                if self.add_joint_to_body(child_body, edge):
                    added_joint += 1
        return body

    def add_body(
        self, node: GraphNode, parent_node: GraphNode, parent_body: ET.Element
    ) -> ET.Element:
        # Create body element for this part
        pos, quat = node.get_body_position_and_orientation(parent_node)
        body = ET.SubElement(
            parent_body,
            "body",
            name=node.part.Name,
            pos=pos,
            quat=quat,
        )
        # Add mesh for visualization
        ET.SubElement(
            body,
            "geom",
            type="mesh",
            name=f"{node.part.Name} geom",
            mesh=node.part.Name,
            contype="1",
            conaffinity="1",
        )
        return body

    def add_joint_to_body(
        self,
        body: ET.Element,
        edge: GraphEdge,
    ) -> None:
        """Add a joint to a body element"""
        joint_type = edge.get_mujoco_joint_type()

        if joint_type is None or joint_type == "fixed":
            return False

        joint_pos, joint_axis = edge.get_joint_position_and_axis()

        # Create the joint element
        joint_element = ET.SubElement(
            body,
            "joint",
            type=joint_type,
            name=edge.joint.Label,
            pos=joint_pos,
            axis=joint_axis,
            damping="0.1",
        )

        # Create actuator element
        actuator_element = ET.SubElement(
            self.actuator,
            "position",
            name=f"position_{joint_element.get('name')}",
            joint=joint_element.get("name"),
            kp="100",
        )
        return True

    def write_xml(self, xml_file: str | os.PathLike) -> None:
        """Writes final XML structure to a file.

        Args:
            xml_file: Output path to XML file.
        """
        ET.indent(self.mujoco)
        tree = ET.ElementTree(self.mujoco)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
        App.Console.PrintMessage(f"{MACRO_NAME}: Successfully exported to {xml_file}\n")


####################################################################
# Main Macro Function
####################################################################


def main() -> None:
    """Macro entrypoint."""
    doc = App.activeDocument()
    if not doc:
        App.Console.PrintError(f"{MACRO_NAME}: No active document\n")
        return

    selection = Gui.Selection.getSelectionEx()
    if not selection:
        App.Console.PrintError("Please select an assembly\n")
        return

    if len(selection) != 1:
        App.Console.PrintError(
            f"{MACRO_NAME}: Expected selection to contain 1 object instead of {len(selection)}. Please only select an assembly\n"
        )
        return

    if selection[0].Object.Type != "Assembly":
        App.Console.PrintError(
            f"{MACRO_NAME}: Expected selection to be an object of type Assembly instead of {selection[0].Object.Type}\n"
        )
        return

    assembly = selection[0].Object

    doc_file = Path(doc.FileName)
    doc_dir = doc_file.parent
    mujoco_export_dir = doc_dir / "mujoco"

    # Export to MuJoCo Format
    exporter = MuJuCoExporter()
    exporter.export_assembly(assembly, output_dir=mujoco_export_dir)


if __name__ == "__main__":
    import PySide
    from PySide import QtGui

    mw = Gui.getMainWindow()
    try:
        r = mw.findChild(QtGui.QTextEdit, "Report view")
        r.clear()
    except Exception:
        None
    try:
        main()
    except Exception as e:
        App.Console.PrintError(f"{MACRO_NAME}: Error: {e}\n")
        raise
