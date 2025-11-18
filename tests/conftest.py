import os
from pathlib import Path

import pytest
from freecad import app  # type: ignore


@pytest.fixture(scope="session")
def examples_dir() -> Path:
    examples_dir = Path(__file__).resolve().parents[1] / "examples"
    assert examples_dir.is_dir()
    return examples_dir


@pytest.fixture(scope="session")
def universal_joint_assembly(examples_dir: Path) -> app.DocumentObject:
    assembly_file = examples_dir / "universal_joint" / "universal_joint.FCStd"
    assert assembly_file.is_file()
    document = app.openDocument(os.fspath(assembly_file))
    assemblies = list(
        filter(lambda x: x.TypeId == "Assembly::AssemblyObject", document.Objects)
    )
    assert len(assemblies) == 1
    yield assemblies[0]


@pytest.fixture(scope="session")
def crank_and_slider_assembly(examples_dir: Path) -> app.DocumentObject:
    assembly_file = examples_dir / "crank_and_slider" / "crank_and_slider.FCStd"
    assert assembly_file.is_file()
    document = app.openDocument(os.fspath(assembly_file))
    assemblies = list(
        filter(lambda x: x.TypeId == "Assembly::AssemblyObject", document.Objects)
    )
    assert len(assemblies) == 1
    yield assemblies[0]


@pytest.fixture
def new_document() -> app.Document:
    return app.newDocument()


@pytest.fixture
def new_document_with_assembly(new_document: app.Document) -> app.Document:
    new_document.addObject("Assembly::AssemblyObject")
    return new_document
