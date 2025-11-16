from pathlib import Path

import pytest
from freecad import app  # type: ignore
from pytest import FixtureRequest

from freecad.assembly2mujoco.core.assembly import AssemblyGraph
from freecad.assembly2mujoco.core.mujoco import MuJoCoExporter


@pytest.mark.parametrize(
    "assembly_fixture_name", ["universal_joint_assembly", "crank_and_slider_assembly"]
)
def test_mujoco_export(
    request: FixtureRequest, tmp_path: Path, assembly_fixture_name: str
):
    assembly: app.DocumentObject = request.getfixturevalue(assembly_fixture_name)
    graph = AssemblyGraph.from_assembly(assembly)
    exporter = MuJoCoExporter()
    mujoco_xml = exporter.export_assembly(graph)
    assert len(mujoco_xml.findall("*")) > 0
    worldbody = mujoco_xml.find("worldbody")
    assert worldbody is not None
    assert len(worldbody.findall("*")) > 0
