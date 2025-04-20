import bisect
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from queue import SimpleQueue
from typing import Any

import FreeCAD as App
import FreeCADGui as Gui
import MeshPart
import UtilsAssembly

MACRO_NAME = "ExportToMujoco"


class AssemblyGraphNode:
    def __init__(
        self,
        part: App.DocumentObject,
        *,
        is_dummy: bool = False,
        is_root: bool = False,
        mass: float = 1.0,
    ) -> None:
        self.part = part
        self.is_dummy = is_dummy
        self.is_root = is_root
        self.mass = mass

    def __repr__(self) -> str:
        return f"<AssemblyGraphNode part={self.part.Name} mass={self.mass} is_dummy={self.is_dummy} is_root={self.is_root}>"


class AssemblyGraphEdge:
    def __init__(
        self, joint, *, node1: AssemblyGraphNode, node2: AssemblyGraphNode
    ) -> None:
        self.joint = joint
        self.node1 = node1
        self.node2 = node2
        self.weight = self.compute_weight(self.joint)
        self.mujoco_joint_type = self.determine_mujoco_joint_type(joint)

    @staticmethod
    def compute_weight(joint) -> float:
        # Assign weights to prioritize which joints to keep in the tree
        # Higher weight are more likely to be excluded from tree
        if joint.JointType == "Revolute":
            weight = 1.0
        elif joint.JointType == "Prismatic":
            weight = 2.0
        elif joint.JointType == "Ball":
            weight = 3.0
        elif joint.JointType == "Cylindrical":
            weight = 4.0
        elif joint.JointType == "Planar":
            weight = 5.0
        elif joint.JointType == "Fixed":
            weight = 10.0
        else:
            weight = 20.0
        return weight

    @staticmethod
    def determine_mujoco_joint_type(joint) -> str | None:
        mujoco_joint_type: str | None = None
        if joint.JointType == "Revolute":
            mujoco_joint_type = "hinge"
        elif joint.JointType == "Prismatic":
            mujoco_joint_type = "slide"
        elif joint.JointType == "Ball":
            mujoco_joint_type = "ball"
        elif joint.JointType == "Cylindrical":
            mujoco_joint_type = "hinge"
        elif joint.JointType == "Planar":
            mujoco_joint_type = "free"
        elif joint.JointType == "Fixed":
            mujoco_joint_type = None
        return mujoco_joint_type

    def __eq__(self, other: "AssemblyGraphEdge") -> bool:
        return self.joint == other.joint


class AssemblyGraph:
    def __init__(self, assembly) -> None:
        self.nodes: dict[str, AssemblyGraphNode] = {}
        self.list_of_edges = []
        self.build_graph(assembly)

    def build_graph(self, assembly) -> None:
        """Construct graph from FreeCAD assembly"""
        joint_group = UtilsAssembly.getJointGroup(assembly)
        for joint in joint_group.Group:
            # Grounded Joint will be set as the root of the graph
            if hasattr(joint, "ObjectToGround"):
                self.add_node(joint.ObjectToGround)
            else:
                part1 = UtilsAssembly.getMovingPart(assembly, joint.Reference1)
                part2 = UtilsAssembly.getMovingPart(assembly, joint.Reference2)
                self.add_edge(part1, part2, joint)

    def get_nodes(self) -> list[AssemblyGraphNode]:
        return list(self.nodes.values())

    def add_node(
        self,
        part: App.DocumentObject,
        *,
        density: float = 0.001,  # g/cm³ to kg/mm³
    ) -> AssemblyGraphNode:
        if (node := self.nodes.get(part.Name)) is not None:
            return node
        # Store additional metadata useful for processing
        mass = 1.0
        volume = 1.0
        if hasattr(part.Shape, "Volume"):
            volume = part.Shape.Volume
            if volume > 1e-9:  # Valid volume
                # Basic estimate - can be improved with material properties
                mass = volume * density
        node = AssemblyGraphNode(part, mass=mass)
        self.nodes[part.Name] = node
        return node

    def add_edge(self, part1, part2, joint) -> None:
        node1 = self.add_node(part1)
        node2 = self.add_node(part2)
        edge1 = AssemblyGraphEdge(joint, node1=node1, node2=node2)
        edge2 = AssemblyGraphEdge(joint, node1=node2, node2=node1)
        bisect.insort(self.list_of_edges, edge1, key=lambda x: x.weight)
        bisect.insort(self.list_of_edges, edge2, key=lambda x: x.weight)

    def simplify_graph(self) -> None:
        return


class MujocoExporter:
    def __init__(self) -> None:
        self.mujoco = ET.Element("mujoco")
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
        # Placeholder for elements that will be added when exporting
        self.equality = ET.SubElement(self.mujoco, "equality")
        self.sensor = ET.SubElement(self.mujoco, "sensor")
        self.actuator = ET.SubElement(self.mujoco, "actuator")

    def export_assembly(
        self, assembly_graph: AssemblyGraph, output_dir: str | os.PathLike
    ) -> None:
        """Main export method"""
        # Export assembly parts as binary stl meshes
        meshes_dir = Path(output_dir).joinpath("meshes")
        meshes_dir.mkdir(exist_ok=True, parents=True)
        self.export_parts_as_meshes_and_add_to_assets(assembly_graph, meshes_dir)

        # Add parts
        self.add_parts_as_independent_bodies(assembly_graph)

        # Add floorplane (cosmetic)
        self.add_floorplane(assembly_graph)

        # Save MJCF file
        xml_file = (
            Path(output_dir).joinpath(App.activeDocument().Name).with_suffix(".xml")
        )
        self.write_xml(xml_file)

    def export_parts_as_meshes_and_add_to_assets(
        self, assembly_graph: AssemblyGraph, meshes_dir: str | os.PathLike
    ) -> None:
        for node in assembly_graph.get_nodes():
            part = node.part
            shape = part.Shape.copy(False)
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

    def add_parts_as_independent_bodies(self, assembly_graph: AssemblyGraph) -> None:
        for node in assembly_graph.get_nodes():
            part = node.part
            part_body = ET.SubElement(
                self.worldbody,
                "body",
                attrib={
                    "name": part.Name,
                    "pos": "0.0 0.0 0.0",
                    "quat": "1.0 0.0 0.0 0.0",
                },
            )
            ET.SubElement(
                part_body,
                "geom",
                attrib={
                    "type": "mesh",
                    "name": f"{part.Name} geom",
                    "mesh": part.Name,
                },
            )

    def add_floorplane(self, assembly_graph: AssemblyGraph) -> None:
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

    def write_xml(self, xml_file: str | os.PathLike) -> None:
        ET.indent(self.mujoco)
        tree = ET.ElementTree(self.mujoco)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
        App.Console.PrintMessage(f"{MACRO_NAME}: Successfully exported to {xml_file}\n")


def main() -> None:
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
    exporter = MujocoExporter()

    # Create graph connecting parts with joints
    assembly_graph = AssemblyGraph(assembly)

    # Simplify graph and convert it to an acyclic one by adding dummy nodes
    assembly_graph.simplify_graph()

    exporter.export_assembly(assembly_graph, output_dir=mujoco_export_dir)


try:
    main()
except Exception as e:
    App.Console.PrintError("ExportToMujoco: ERROR: {}\n".format(e))
    raise
