import JointObject
import pytest
from freecad import app  # type: ignore
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
from pytest import FixtureRequest


@pytest.mark.parametrize(
    "assembly_fixture_name",
    ["universal_joint_assembly", "crank_and_slider_assembly", "pan_tilt_assembly"],
)
def test_assembly_graph(request: FixtureRequest, assembly_fixture_name: str):
    assembly: app.DocumentObject = request.getfixturevalue(assembly_fixture_name)
    graph = AssemblyGraph.from_assembly(assembly)
    assert len(graph.get_nodes()) > 0
    assert len(graph.get_edges()) > 0


def test_disconnected_subgraphs(new_document_with_assembly: app.Document):
    graph = AssemblyGraph()
    nodes = []
    for _ in range(4):
        node = AssemblyGraphNode(
            new_document_with_assembly.addObject("PartDesign::Body")
        )
        graph.add_node(node)
        nodes.append(node)

    assembly = new_document_with_assembly.Objects[0]
    edges = []
    for _ in range(3):
        joint = assembly.newObject("App::FeaturePython", "Temporary joint")
        JointObject.Joint(joint, 0)
        edge = AssemblyGraphEdge(joint, weight=1.0)
        edges.append(edge)

    for edge, (parent_node, child_node) in zip(edges, zip(nodes[:-2], nodes[2:])):
        graph.add_edge(edge, parent_node=parent_node, child_node=child_node)

    subgraphs = get_disconnected_subgraphs(graph)
    assert len(subgraphs) == 2


def test_depth_first_traversal(new_document_with_assembly: app.Document):
    graph = AssemblyGraph()

    nodes = []
    for i in range(3):
        node = AssemblyGraphNode(
            new_document_with_assembly.addObject("PartDesign::Body", f"Body-{i}")
        )
        graph.add_node(node)
        nodes.append(node)

    assembly = new_document_with_assembly.Objects[0]
    edges = []
    for i in range(3):
        joint = assembly.newObject("App::FeaturePython", f"Joint-{i}")
        JointObject.Joint(joint, 0)
        edge = AssemblyGraphEdge(joint, weight=1.0)
        edges.append(edge)

    for edge, (parent_node, child_node) in zip(
        edges, zip(nodes, nodes[1:] + [nodes[0]])
    ):
        graph.add_edge(edge, parent_node=parent_node, child_node=child_node)

    traversed_nodes = set()
    traversed_edges = set()
    for current_node, next_node, edge in depth_first_traversal(
        graph, root_node=nodes[1]
    ):
        traversed_nodes.add(current_node)
        traversed_nodes.add(next_node)
        traversed_edges.add(edge)

    assert traversed_nodes == set(nodes)
    assert traversed_edges == set(edges)


def test_depth_first_traversal_single_node(new_document_with_assembly: app.Document):
    graph = AssemblyGraph()

    node = AssemblyGraphNode(
        new_document_with_assembly.addObject("PartDesign::Body", "Body")
    )
    graph.add_node(node)
    nodes = [node]

    traversed_nodes = set()
    traversed_edges = set()
    for current_node, next_node, edge in depth_first_traversal(
        graph, root_node=nodes[0]
    ):
        traversed_nodes.add(current_node)
        if next_node is None:
            continue
        traversed_nodes.add(next_node)
        traversed_edges.add(edge)

    assert traversed_nodes == set(nodes)
    assert traversed_edges == set()


def test_converting_graph_to_directed_tree(new_document_with_assembly: app.Document):
    graph = AssemblyGraph()

    nodes = []
    for i in range(3):
        node = AssemblyGraphNode(
            new_document_with_assembly.addObject("PartDesign::Body", f"Body-{i}")
        )
        graph.add_node(node)
        nodes.append(node)

    assembly = new_document_with_assembly.Objects[0]
    edges = []
    for i in range(3):
        joint = assembly.newObject("App::FeaturePython", f"Joint-{i}")
        JointObject.Joint(joint, 0)
        edge = AssemblyGraphEdge(joint, weight=1.0)
        edges.append(edge)

    for edge, (parent_node, child_node) in zip(
        edges, zip(nodes, nodes[1:] + [nodes[0]])
    ):
        graph.add_edge(edge, parent_node=parent_node, child_node=child_node)

    tree, unused_edges = convert_to_directed_tree(graph, root_node=nodes[1])
    assert len(tree.get_nodes()) == len(graph.get_nodes())
    assert len(tree.get_edges()) == len(graph.get_edges()) - 1
    assert len(unused_edges) == 1


def test_cylindrical_joint_is_cylindrical_property(
    new_document_with_assembly: app.Document,
):
    """Test that cylindrical joints are correctly identified."""
    assembly = new_document_with_assembly.Objects[0]

    # Create a cylindrical joint
    cylinder_joint = assembly.newObject("App::FeaturePython", "CylindricalJoint")
    JointObject.Joint(cylinder_joint, 0)
    cylinder_joint.JointType = "Cylindrical"

    edge = AssemblyGraphEdge(cylinder_joint, weight=1.0)
    assert edge.is_cylindrical

    # Create a revolute joint for comparison
    rev_joint = assembly.newObject("App::FeaturePython", "RevoluteJoint")
    JointObject.Joint(rev_joint, 0)
    rev_joint.JointType = "Revolute"

    rev_edge = AssemblyGraphEdge(rev_joint, weight=1.0)
    assert not rev_edge.is_cylindrical


def test_cylindrical_joint_position_and_axis(new_document_with_assembly: app.Document):
    """Test that cylindrical joints extract position and axis correctly."""
    assembly = new_document_with_assembly.Objects[0]

    # Create a cylindrical joint
    cylinder_joint = assembly.newObject("App::FeaturePython", "CylindricalJoint")
    JointObject.Joint(cylinder_joint, 0)
    cylinder_joint.JointType = "Cylindrical"

    # Set up basic placement
    cylinder_joint.Placement1 = app.Placement()
    cylinder_joint.Reference1 = None

    edge = AssemblyGraphEdge(cylinder_joint, weight=1.0)

    # Should not raise NotImplementedError
    pos, axis = edge.joint_position_and_axis

    assert isinstance(pos, app.Vector)
    assert isinstance(axis, app.Vector)
    assert abs(axis.Length - 1.0) < 1e-6  # Axis should be normalized


def test_ball_joint_is_ball_property(new_document_with_assembly: app.Document):
    """Test that ball joints are correctly identified."""
    assembly = new_document_with_assembly.Objects[0]

    # Create a ball joint
    ball_joint = assembly.newObject("App::FeaturePython", "BallJoint")
    JointObject.Joint(ball_joint, 0)
    ball_joint.JointType = "Ball"

    edge = AssemblyGraphEdge(ball_joint, weight=1.0)
    assert edge.is_ball

    # Create a revolute joint for comparison
    rev_joint = assembly.newObject("App::FeaturePython", "RevoluteJoint")
    JointObject.Joint(rev_joint, 0)
    rev_joint.JointType = "Revolute"

    rev_edge = AssemblyGraphEdge(rev_joint, weight=1.0)
    assert not rev_edge.is_ball


def test_ball_joint_position_and_axis(new_document_with_assembly: app.Document):
    """Test that ball joints extract position correctly (axis is zero vector)."""
    assembly = new_document_with_assembly.Objects[0]

    # Create a ball joint
    ball_joint = assembly.newObject("App::FeaturePython", "BallJoint")
    JointObject.Joint(ball_joint, 0)
    ball_joint.JointType = "Ball"

    # Set up basic placement
    ball_joint.Placement1 = app.Placement()
    ball_joint.Reference1 = None

    edge = AssemblyGraphEdge(ball_joint, weight=1.0)

    # Should not raise NotImplementedError
    pos, axis = edge.joint_position_and_axis

    assert isinstance(pos, app.Vector)
    assert isinstance(axis, app.Vector)
    # Ball joints return zero vector for axis (not needed for ball joints)
    assert abs(axis.Length) < 1e-6
