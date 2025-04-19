import os
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from queue import SimpleQueue

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
    ) -> None:
        self.part = part
        self.is_dummy = is_dummy
        self.is_root = is_root

    def __repr__(self) -> str:
        return f"<AssemblyGraphNode part={self.part.Name} is_dummy={self.is_dummy} is_root={self.is_root}>"


class AssemblyGraphEdge:
    def __init__(self, joint) -> None:
        self.joint = joint


class AssemblyGraph:
    def __init__(self):
        self.nodes: dict[str, AssemblyGraphNode] = {}
        self.list_of_edges = defaultdict(list)
        self.root_node = None

    def get_nodes(self) -> list[AssemblyGraphNode]:
        return list(self.nodes.values())

    def add_node(
        self,
        part: App.DocumentObject,
        *,
        is_dummy: bool = False,
        is_root: bool = False,
    ) -> AssemblyGraphNode:
        if (node := self.nodes.get(part.Name)) is not None:
            return node
        node = AssemblyGraphNode(part, is_dummy=is_dummy, is_root=is_root)
        self.nodes[part.Name] = node
        if is_root:
            self.root_node = node
        return node

    def add_edge(self, part1, part2, joint) -> None:
        node1 = self.add_node(part1)
        node2 = self.add_node(part2)
        self.list_of_edges[node1].append((node2, joint))
        self.list_of_edges[node2].append((node1, joint))

    def simplify_graph(self) -> None:
        if not self._is_graph_cyclic():
            return

    def _is_graph_cyclic(self) -> bool:
        return True

    def __iter__(self):
        if self.root_node is None:
            raise RuntimeError("Root node was not set")

        already_traversed_edges = set()
        nodes_to_traverse = SimpleQueue()
        nodes_to_traverse.put(self.root_node)

        while not nodes_to_traverse.empty():
            node = nodes_to_traverse.get()
            edges = self.list_of_edges[node]
            for node2, edge in edges:
                if edge in already_traversed_edges:
                    continue
                already_traversed_edges.add(edge)
                nodes_to_traverse.put(node2)
                yield node, node2, edge


class MujocoExporter:
    def __init__(self, export_dir: str | os.PathLike):
        self.export_dir = Path(export_dir)
        self.export_dir.mkdir(exist_ok=True)
        self.xml_file = self.export_dir.joinpath("assembly.xml")
        self.meshes_dir = self.export_dir / "meshes"
        self.meshes_dir.mkdir(exist_ok=True)

    def export_parts_as_meshes(self, assembly_graph: AssemblyGraph) -> None:
        for node in assembly_graph.get_nodes():
            if node.is_dummy:
                continue
            part = node.part
            shape = part.Shape.copy(False)
            mesh = MeshPart.meshFromShape(
                Shape=shape,
                LinearDeflection=0.1,
                AngularDeflection=5.0,
                Relative=False,
            )
            mesh_file = self.meshes_dir.joinpath(part.Name).with_suffix(".stl")
            mesh.write(os.fspath(mesh_file))

    def write_xml_file(self, assembly_graph: AssemblyGraph) -> None:
        xml_root = ET.Element("mujoco")

        # Skybox, Textures and Meshes
        ET.SubElement(xml_root, "compiler", attrib={"meshdir": "./meshes"})
        assets = ET.SubElement(xml_root, "asset")
        ET.SubElement(
            assets,
            "texture",
            attrib={
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "0.3 0.5 0.7",
                "rgb2": "0 0 0",
                "width": "512",
                "height": "3072",
            },
        )
        ET.SubElement(
            assets,
            "texture",
            attrib={
                "type": "2d",
                "name": "groundplane",
                "builtin": "checker",
                "mark": "edge",
                "rgb1": "0.2 0.3 0.4",
                "rgb2": "0.1 0.2 0.3",
                "markrgb": "0.8 0.8 0.8",
                "width": "300",
                "height": "300",
            },
        )
        ET.SubElement(
            assets,
            "material",
            attrib={
                "name": "groundplane",
                "texture": "groundplane",
                "texuniform": "true",
                "texrepeat": "5 5",
                "reflectance": "0.1",
            },
        )
        for node in assembly_graph.get_nodes():
            if node.is_dummy:
                continue
            part = node.part
            mesh_file = f"{part.Name}.stl"
            ET.SubElement(assets, "mesh", attrib={"name": part.Name, "file": mesh_file})

        xml_worldbody = ET.SubElement(xml_root, "worldbody")

        # Add light sources and floor plane
        ET.SubElement(xml_worldbody, "light", attrib={"pos": "0 0 1"})
        ET.SubElement(
            xml_worldbody,
            "light",
            attrib={
                "pos": "0 -0.2 1",
                "dir": "0 0.2 -0.8",
                "directional": "true",
            },
        )
        ET.SubElement(
            xml_worldbody,
            "geom",
            attrib={
                "name": "floor",
                "pos": "0 0 -3",
                "size": "0 0 3",
                "type": "plane",
                "material": "groundplane",
            },
        )

        part_to_xml_elem_mapping = {}
        for node1, node2, joint in assembly_graph:
            part1 = node1.part
            part2 = node2.part
            if (part1_xml_body := part_to_xml_elem_mapping.get(part1)) is None:
                part1_xml_body = ET.SubElement(
                    xml_worldbody,
                    "body",
                    attrib={
                        "name": part1.Name,
                        "pos": "0.0 0.0 0.0",
                        "quat": "1.0 0.0 0.0 0.0",
                    },
                )

                ET.SubElement(
                    part1_xml_body,
                    "geom",
                    attrib={
                        "type": "mesh",
                        "name": f"{part1.Name} geom",
                        "mesh": part1.Name,
                    },
                )
                part_to_xml_elem_mapping[part1] = part1_xml_body
            if part2 not in part_to_xml_elem_mapping:
                part2_xml_body = ET.SubElement(
                    part1_xml_body,
                    "body",
                    attrib={
                        "name": part2.Name,
                        "pos": "0.0 0.0 0.0",
                        "quat": "1.0 0.0 0.0 0.0",
                    },
                )
                ET.SubElement(
                    part2_xml_body,
                    "geom",
                    attrib={
                        "type": "mesh",
                        "name": f"{part2.Name} geom",
                        "mesh": part2.Name,
                    },
                )
                part_to_xml_elem_mapping[part2] = part2_xml_body

        xml_tree = ET.ElementTree(xml_root)
        ET.indent(xml_tree)
        xml_tree.write(self.xml_file, method="xml")


def main() -> None:
    doc = App.activeDocument()
    if not doc:
        raise RuntimeError(f"{MACRO_NAME}: No active document")

    doc_file = Path(doc.FileName)
    doc_dir = doc_file.parent
    mujoco_export_dir = doc_dir / "mujoco"
    exporter = MujocoExporter(mujoco_export_dir)

    selection = Gui.Selection.getSelectionEx()
    if len(selection) != 1:
        raise RuntimeError(
            f"{MACRO_NAME}: Expected selection to contain 1 object instead of {len(selection)}. Please select the Assembly only"
        )

    if selection[0].Object.Type != "Assembly":
        raise RuntimeError(
            f"{MACRO_NAME}: Expected selection to be an object of type Assembly instead of {selection[0].Object.Type}"
        )

    assembly = UtilsAssembly.activeAssembly()

    # Get joint group
    joint_group = UtilsAssembly.getJointGroup(assembly)

    # Create graph connecting parts with joints
    assembly_graph = AssemblyGraph()

    for joint in joint_group.Group:
        # Grounded Joint will be set as the root of the graph
        if hasattr(joint, "ObjectToGround"):
            assembly_graph.add_node(joint.ObjectToGround, is_root=True)
        else:
            part1 = UtilsAssembly.getMovingPart(assembly, joint.Reference1)
            part2 = UtilsAssembly.getMovingPart(assembly, joint.Reference2)
            assembly_graph.add_edge(part1, part2, joint)

    # Simplify graph and convert it to an acyclic one by adding dummy nodes
    assembly_graph.simplify_graph()

    # export parts as stl meshes
    exporter.export_parts_as_meshes(assembly_graph)

    # write mujoco xml file
    exporter.write_xml_file(assembly_graph)


try:
    main()
except Exception as e:
    App.Console.PrintError("ExportToMujoco: ERROR: {}\n".format(e))
    raise
