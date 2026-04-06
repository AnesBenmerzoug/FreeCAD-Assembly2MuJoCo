import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Literal

import Mesh
import MeshPart

from freecad.assembly2mujoco.constants import (
    DEFAULT_STL_MESH_ANGULAR_DEFLECTION,
    DEFAULT_STL_MESH_LINEAR_DEFLECTION,
    DEFAULT_MESH_EXPORT_FORMAT,
    DEFAULT_JOINT_TYPE_WEIGHTS,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_SOLVER_MAX_ITERATIONS,
    DEFAULT_MJCF_SOLVER_TOLERANCE,
    DEFAULT_MJCF_TIMESTEP,
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    WORKBENCH_NAME,
)
from freecad.assembly2mujoco.core.assembly import (
    AssemblyGraph,
    AssemblyGraphNode,
    AssemblyGraphEdge,
)
from freecad.assembly2mujoco.core.graph_utils import (
    get_disconnected_subgraphs,
    convert_to_directed_tree,
    depth_first_traversal,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["MuJoCoExporter"]


class MuJoCoExporter:
    """Class for exporting a kinematic tree as a MuJoCo MJCF (XML) file and STL files.

    Args:
        mjcf_integrator: The numerical integrator to be used in MuJoCo.
            The available integrators are the semi-implicit Euler method,
            the fixed-step 4-th order Runge Kutta method,
            the Implicit-in-velocity Euler method, and implicitfast.
        mjcf_timestep: Simulation time step in seconds.
    """

    def __init__(
        self,
        *,
        mesh_export_format: Literal["STL", "OBJ"] = DEFAULT_MESH_EXPORT_FORMAT,
        stl_mesh_linear_deflection: float = DEFAULT_STL_MESH_LINEAR_DEFLECTION,
        stl_mesh_angular_deflection: float = DEFAULT_STL_MESH_ANGULAR_DEFLECTION,
        joint_type_weights: dict = DEFAULT_JOINT_TYPE_WEIGHTS,
        mjcf_integrator: Literal[
            "Euler", "implicit", "implicitfast", "RK4"
        ] = DEFAULT_MJCF_INTEGRATOR,
        mjcf_solver: Literal["PGS", "CG", "Newton"] = DEFAULT_MJCF_SOLVER,
        mjcf_solver_max_iterations: float = DEFAULT_MJCF_SOLVER_MAX_ITERATIONS,
        mjcf_solver_tolerance: float = DEFAULT_MJCF_SOLVER_TOLERANCE,
        mjcf_timestep: float = DEFAULT_MJCF_TIMESTEP,
        mjcf_damping: float = DEFAULT_MJCF_DAMPING,
        mjcf_armature: float = DEFAULT_MJCF_ARMATURE,
    ) -> None:
        self.mesh_export_format = mesh_export_format
        self.stl_mesh_linear_deflection = stl_mesh_linear_deflection
        self.stl_mesh_angular_deflection = stl_mesh_angular_deflection
        self.joint_type_weights = joint_type_weights
        self.mjcf_integrator = mjcf_integrator
        self.mjcf_solver = mjcf_solver
        self.mjcf_solver_max_iterations = mjcf_solver_max_iterations
        self.mjcf_solver_tolerance = mjcf_solver_tolerance
        self.mjcf_timestep = mjcf_timestep
        self.mjcf_damping = mjcf_damping
        self.mjcf_armature = mjcf_armature

        self.mujoco = ET.Element("mujoco")
        self.option = ET.SubElement(
            self.mujoco,
            "option",
            integrator=self.mjcf_integrator,
            timestep=str(self.mjcf_timestep),
            solver=self.mjcf_solver,
            iterations=str(self.mjcf_solver_max_iterations),
            tolerance=str(self.mjcf_solver_tolerance),
        )
        self.compiler = ET.SubElement(
            self.mujoco,
            "compiler",
            meshdir="meshes",
            autolimits="true",
            inertiafromgeom="true",
            angle="radian",
        )
        # Add defaults
        self.default = ET.SubElement(self.mujoco, "default")
        ET.SubElement(
            self.default,
            "joint",
            damping=f"{self.mjcf_damping}",
            armature=f"{self.mjcf_armature}",
        )
        ET.SubElement(
            self.default,
            "geom",
            conaffinity="0",
            condim="3",
            friction="1 0.5 0.5",
            margin="0",
        )
        # Add assets
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
        self.worldbody.append(ET.Comment("Lighting and floor"))
        ET.SubElement(
            self.worldbody, "light", diffuse=".8 .8 .8", pos="0 0 4", dir="0 0 -1"
        )
        ET.SubElement(
            self.worldbody, "light", diffuse=".6 .6 .6", pos="4 4 4", dir="-1 -1 -1"
        )
        # Placeholder for elements that will be added when exporting
        self.contact = ET.SubElement(self.mujoco, "contact")
        self.equality = ET.SubElement(self.mujoco, "equality")
        self.tendon = ET.SubElement(self.mujoco, "tendon")
        self.actuator = ET.SubElement(self.mujoco, "actuator")
        self.sensor = ET.SubElement(self.mujoco, "sensor")

    def export_assembly(
        self, assembly_graph: AssemblyGraph, *, export_dir: Path | None = None
    ) -> ET.Element:
        """Main export method"""

        # Apply edge weights from configuration
        assembly_graph.update_edge_weights(self.joint_type_weights)

        if export_dir is not None:
            # Export assembly parts as binary stl or obj meshes
            self.export_parts_as_meshes_and_add_to_assets(
                assembly_graph,
                parent=self.asset,
                export_dir=export_dir,
                mesh_export_format=self.mesh_export_format,
                stl_mesh_angular_deflection=self.stl_mesh_angular_deflection,
                stl_mesh_linear_deflection=self.stl_mesh_linear_deflection,
            )

        # Add floorplane (cosmetic)
        self.add_floorplane(assembly_graph)

        # See if graph can be split into disconnected graphs
        assembly_subgraphs = get_disconnected_subgraphs(assembly_graph)
        log_message(f"Number of disconnected subgraphs: {len(assembly_subgraphs)}")

        self.worldbody.append(ET.Comment("Assembly"))
        for graph in assembly_subgraphs:
            if len(graph.get_nodes()) == 0:
                log_message("Subgraph is empty. This should not happen")
                raise RuntimeError("Subgraph is empty")

            # Handle graphs with a single node
            if len(graph.get_nodes()) == 1:
                self.process_single_node_tree(graph, self.worldbody)
                continue

            grounded_part_nodes = [
                node for node in graph.get_nodes() if node.is_grounded
            ]
            if grounded_part_nodes:
                root_node = grounded_part_nodes[0]
            else:
                log_message(
                    "Could not find grounded node in graph. Using a non-grounded node as root",
                    level="warning",
                )
                root_node = graph.get_nodes()[0]

            # Convert directed tree
            tree, unused_edges = convert_to_directed_tree(graph, root_node=root_node)

            self.process_tree_no_recursion(
                tree, root_node=root_node, worldbody=self.worldbody
            )  # type: ignore

            # Handle kinematic loops
            if unused_edges:
                self.process_kinematic_loops(unused_edges)

        return self.mujoco

    @staticmethod
    def export_parts_as_meshes_and_add_to_assets(
        assembly_graph: AssemblyGraph,
        *,
        parent: ET.Element,
        export_dir: Path,
        mesh_export_format: Literal["STL", "OBJ"],
        stl_mesh_linear_deflection: float,
        stl_mesh_angular_deflection: float,
    ) -> None:
        meshes_dir = Path(export_dir).joinpath("meshes")
        meshes_dir.mkdir(exist_ok=True, parents=True)

        for node in assembly_graph.get_nodes():
            mesh_file = Path(meshes_dir).joinpath(node.label)

            if mesh_export_format == "STL":
                shape = node.part.Shape.copy(False)
                mesh = MeshPart.meshFromShape(
                    Shape=shape,
                    LinearDeflection=stl_mesh_linear_deflection,
                    AngularDeflection=stl_mesh_angular_deflection,
                    Relative=False,
                )
                mesh_file = mesh_file.with_suffix(".stl")
                mesh.write(os.fspath(mesh_file))
            elif mesh_export_format == "OBJ":
                mesh_file = mesh_file.with_suffix(".obj")
                Mesh.export([node.part], os.fspath(mesh_file))
            else:
                raise ValueError(
                    f"{WORKBENCH_NAME}: Unexpected mesh export format '{mesh_export_format}'"
                )

            parent.append(ET.Comment("Part Meshes"))
            # Add new mesh to assets
            ET.SubElement(
                parent,
                "mesh",
                name=node.label,
                file=mesh_file.name,
                # Convert mm to m
                scale="0.001 0.001 0.001",
            )
            # Add material for appearance to assets
            found_existing_materials = parent.findall(
                f"./material[@name='{node.body_appearance['name']}']"
            )
            # Add material only if it wasn't added already
            # We do the check by name
            # TODO: Consider using a dictionary to keep track of added materials
            if len(found_existing_materials) == 0:
                ET.SubElement(parent, "material", **node.body_appearance)

    def add_floorplane(self, assembly_graph: AssemblyGraph) -> None:
        minimum_z_placement: float | None = None
        for node in assembly_graph.get_nodes():
            part = node.part
            if minimum_z_placement is None:
                minimum_z_placement = part.Placement.Base[2]
            else:
                minimum_z_placement = min(minimum_z_placement, part.Placement.Base[2])
        if minimum_z_placement is None:
            raise RuntimeError(f"{WORKBENCH_NAME}: This should not happen")
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

    def process_single_node_tree(
        self,
        tree: AssemblyGraph,
        worldbody: ET.Element,
    ) -> None:
        if len(tree.get_nodes()) != 1:
            raise RuntimeError(
                "This method should only be called for trees with a single node"
            )

        node = tree.get_nodes()[0]
        body = self.add_body(node, worldbody)
        ET.SubElement(
            body,
            "freejoint",
        )

    def process_tree_no_recursion(
        self,
        tree: AssemblyGraph,
        root_node: AssemblyGraphNode,
        worldbody: ET.Element,
    ) -> None:
        body_elements: dict[AssemblyGraphNode, ET.Element] = {}

        for parent_node, child_node, edge in depth_first_traversal(
            tree, root_node=root_node
        ):
            if (parent_body := body_elements.get(parent_node)) is None:
                parent_body = self.add_body(parent_node, worldbody)
                body_elements[parent_node] = parent_body

            if child_node is None or edge is None:
                continue

            if (child_body := body_elements.get(child_node)) is None:
                child_body = self.add_body(child_node, parent_body)
                body_elements[child_node] = child_body

            self.add_joint_to_body(child_body, edge)

    def process_tree(
        self,
        current_node: AssemblyGraphNode,
        tree: AssemblyGraph,
        *,
        parent_node: AssemblyGraphNode | None = None,
        body_elements: dict[AssemblyGraphNode, ET.Element] | None = None,
    ) -> ET.Element:
        if body_elements is None:
            body_elements = {}

        if parent_node is None:
            parent_body = self.worldbody
        else:
            parent_body = body_elements.get(parent_node) or self.worldbody

        if (current_body := body_elements.get(current_node)) is None:
            current_body = self.add_body(current_node, parent_body)
            body_elements[current_node] = current_body

        children_nodes = tree.get_neighbors(current_node)
        if not children_nodes and parent_node is None:
            # Handle case when there is a single node in the graph
            self.add_free_joint_to_body(current_body)
        else:
            for child_node in tree.get_neighbors(current_node):
                child_body = self.process_tree(
                    child_node,
                    tree,
                    parent_node=current_node,
                    body_elements=body_elements,
                )
                edge = tree.get_edge(current_node, child_node)
                self.add_joint_to_body(child_body, edge)
        return current_body

    def add_body(self, node: AssemblyGraphNode, parent_body: ET.Element) -> ET.Element:
        # Create body element for this part
        body = ET.SubElement(
            parent_body,
            "body",
            name=node.label,
            pos=node.pos,
            quat=node.quat,
        )
        # Add mesh for visualization
        ET.SubElement(
            body,
            "geom",
            type="mesh",
            name=f"{node.label} geom",
            mesh=node.label,
            material=node.body_appearance["name"],
            contype="0",
            conaffinity="0",
        )
        return body

    def add_free_joint_to_body(
        self,
        body: ET.Element,
    ) -> ET.Element:
        joint_element = ET.SubElement(
            body,
            "freejoint",
        )
        return joint_element

    def add_joint_to_body(
        self,
        body: ET.Element,
        edge: AssemblyGraphEdge,
    ) -> ET.Element | None:
        """Add a joint to a body element"""
        joint_type = edge.mujoco_joint_type
        if joint_type is None:
            return None

        joint_pos_vector, joint_axis_vector = edge.joint_position_and_axis
        joint_pos = " ".join(str(x) for x in joint_pos_vector)
        joint_axis = " ".join(str(x) for x in joint_axis_vector)
        joint_range = edge.joint_range

        # Create the joint element
        joint_element = ET.SubElement(
            body,
            "joint",
            type=joint_type,
            name=edge.label,
            pos=joint_pos,
            axis=joint_axis,
        )
        if joint_range is not None:
            joint_element.set("range", joint_range)

        # Create actuator element
        actuator_element = ET.SubElement(
            self.actuator,
            "position",
            name=joint_element.get("name"),  # type: ignore
            joint=joint_element.get("name"),  # type: ignore
            kp="100",
        )
        if joint_range is not None:
            actuator_element.set("ctrlrange", joint_range)

        # Create sensor element
        ET.SubElement(
            self.sensor,
            "jointpos",
            name=joint_element.get("name") + "_pos",  # type: ignore
            joint=joint_element.get("name"),  # type: ignore
        )
        return joint_element

    def process_kinematic_loops(
        self,
        unused_edges: list[
            tuple[AssemblyGraphNode, AssemblyGraphNode, AssemblyGraphEdge]
        ],
    ) -> None:
        log_message(f"Found {len(unused_edges)} kinematic loops in the assembly")
        for u, v, edge in unused_edges:
            if edge.mujoco_joint_type is None:
                # For fixed joints, use weld constraint
                ET.SubElement(
                    self.equality,
                    "weld",
                    name=f"loop_weld_{edge.label}",
                    body1=u.label,
                    body2=v.label,
                    solref="0.01 1",
                    solimp="0.9 0.95 0.001",
                )
            elif edge.mujoco_joint_type == "hinge":
                joint_position, joint_axis = edge.joint_position_and_axis
                offset_joint_position = joint_position + joint_axis.scale(
                    0.01, 0.01, 0.01
                )
                anchor = f"{joint_position.x} {joint_position.y} {joint_position.z}"
                offset_anchor = f"{offset_joint_position.x} {offset_joint_position.y} {offset_joint_position.z}"
                ET.SubElement(
                    self.equality,
                    "connect",
                    body1=u.label,
                    body2=v.label,
                    name=f"connect_hinge_{edge.label}_1",
                    anchor=anchor,
                )
                ET.SubElement(
                    self.equality,
                    "connect",
                    body1=u.label,
                    body2=v.label,
                    name=f"connect_hinge_{edge.label}_2",
                    anchor=offset_anchor,
                )
            else:
                joint_pos_vector, joint_axis_vector = edge.joint_position_and_axis
                joint_pos = " ".join(str(x) for x in joint_pos_vector)

                # For other joint types we insert dummy bodies and add weld constraints
                found_bodies = self.worldbody.findall(f".//body[@name='{u.label}']")
                if not found_bodies:
                    raise ValueError(
                        f"Could not find body with name '{u.label}' in MJCF"
                    )

                parent_body = found_bodies[0]

                # Insert dummy body
                dummy_body = ET.SubElement(
                    parent_body,
                    "body",
                    name=f"dummy_{u.label}_{v.label}",
                    pos=joint_pos,
                )
                ET.SubElement(
                    dummy_body,
                    "inertial",
                    pos=joint_pos,
                    mass="1e-6",  # Much larger than mjMINVAL (1e-15)
                    diaginertia="1e-9 1e-9 1e-9",  # Much larger than mjMINVAL
                )
                # Insert joint between parent and dummy body
                self.add_joint_to_body(dummy_body, edge)

                # Insert weld constraint between dummy body and child body
                ET.SubElement(
                    self.equality,
                    "weld",
                    name=f"loop_weld_{edge.label}",
                    body1=dummy_body.get("name"),  # type: ignore
                    body2=v.label,
                    solref="0.01 1",
                    solimp="0.9 0.95 0.001",
                )

    def write_xml(
        self,
        xml: ET.Element,
        xml_file: Path,
    ) -> None:
        """Writes final XML structure to a file.

        Args:
            xml_file: Output path to XML file.
        """
        # Save MJCF file
        ET.indent(xml)
        tree = ET.ElementTree(xml)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
        log_message(f"Successfully exported to {xml_file}")
