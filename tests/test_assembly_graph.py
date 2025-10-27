import JointObject
import pytest
from freecad import app  # type: ignore
from freecad.assembly2mujoco.core.assembly_parser import (
    AssemblyGraph,
    AssemblyGraphNode,
    AssemblyGraphEdge,
)
from pytest import FixtureRequest


@pytest.mark.parametrize(
    "assembly_fixture_name", ["universal_joint_assembly", "crank_and_slider_assembly"]
)
def test_assembly_graph(request: FixtureRequest, assembly_fixture_name: str):
    assembly: app.DocumentObject = request.getfixturevalue(assembly_fixture_name)
    graph = AssemblyGraph.from_assembly(assembly)
    assert len(graph.get_nodes()) > 0
    assert len(graph.get_edges()) > 0


def test_disconnected_subgraphs(new_document_with_assembly: app.Document):
    graph = AssemblyGraph()
    node1 = AssemblyGraphNode(new_document_with_assembly.addObject("PartDesign::Body"))
    node2 = AssemblyGraphNode(new_document_with_assembly.addObject("PartDesign::Body"))
    node3 = AssemblyGraphNode(new_document_with_assembly.addObject("PartDesign::Body"))
    node4 = AssemblyGraphNode(new_document_with_assembly.addObject("PartDesign::Body"))
    assembly = new_document_with_assembly.Objects[0]
    joint1 = assembly.newObject("App::FeaturePython", "Temporary joint")
    JointObject.Joint(joint1, 0)
    joint2 = assembly.newObject("App::FeaturePython", "Temporary joint")
    JointObject.Joint(joint2, 0)
    joint3 = assembly.newObject("App::FeaturePython", "Temporary joint")
    JointObject.Joint(joint3, 0)
    edge1 = AssemblyGraphEdge(joint1, parent_node=node1, child_node=node2, weight=1.0)
    edge2 = AssemblyGraphEdge(joint2, parent_node=node2, child_node=node3, weight=1.0)
    edge3 = AssemblyGraphEdge(joint3, parent_node=node3, child_node=node1, weight=1.0)
    graph.add_edge(edge1)
    graph.add_edge(edge2)
    graph.add_edge(edge3)
    graph.add_node(node4)
    subgraphs = graph.get_disconnected_subgraphs()
    assert len(subgraphs) == 2
