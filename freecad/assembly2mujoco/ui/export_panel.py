from pathlib import Path
from typing import Callable, Literal, TypedDict

import FreeCAD as App
from PySide import QtWidgets

from freecad.assembly2mujoco.core.assembly import AssemblyGraph
from freecad.assembly2mujoco.ui.export_dir_editor import ExportDirEditor
from freecad.assembly2mujoco.ui.debug_options_editor import DebugOptionsEditor
from freecad.assembly2mujoco.ui.joint_type_weights_editor import (
    JointTypeWeightEditor,
    JointTypeWeightsDict,
)
from freecad.assembly2mujoco.ui.mjcf_options_editor import MJCFOptionsEditor
from freecad.assembly2mujoco.ui.mesh_export_options_editor import (
    MeshExportOptionsEditor,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["ExportTaskPanel", "ExportParamsDict"]


class ExportParamsDict(TypedDict):
    export_dir: Path
    mesh_export_format: Literal["STL", "OBJ"]
    stl_mesh_linear_deflection: float
    stl_mesh_angular_deflection: float
    joint_type_weights: JointTypeWeightsDict
    mjcf_timestep: float
    mjcf_damping: float
    mjcf_armature: float
    mjcf_integrator: Literal["implicitfast", "Euler", "implicit", "RK4"]
    mjcf_solver: Literal["PGS", "CG", "Newton"]


class ExportTaskPanel:
    def __init__(
        self,
        assembly_graph: AssemblyGraph,
        on_accept_callback: Callable[[ExportParamsDict], bool],
    ):
        self.assembly_graph = assembly_graph
        self.on_accept_callback = on_accept_callback

        # Get current document and its path
        self.doc = App.ActiveDocument
        if not self.doc:
            log_message("No active document", level="error")
            raise RuntimeError("No active document")

        self.setup_ui()
        self.set_default_values()

    def setup_ui(self) -> None:
        # Create the form widget
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Export FreeCAD Assembly to MuJoCo")

        # Create main layout
        main_layout = QtWidgets.QVBoxLayout(self.form)

        # Toolbox
        toolbox = QtWidgets.QToolBox()
        main_layout.addWidget(toolbox)

        ###############################################
        # Directory selection section
        ###############################################
        self.export_dir_editor = ExportDirEditor(doc=self.doc)
        tool_index = toolbox.addItem(self.export_dir_editor, "Export Directory")
        toolbox.setItemToolTip(
            tool_index,
            "Select the path to the directory in which the export files will be saved.",
        )

        ###############################################
        # Mesh Export options
        ###############################################
        self.mesh_export_options_editor = MeshExportOptionsEditor()
        tool_index = toolbox.addItem(
            self.mesh_export_options_editor, "Mesh Export Options"
        )
        toolbox.setItemToolTip(
            tool_index,
            "Adjust options for exporting parts as meshes.",
        )

        ###############################################
        # Joint Type Weights
        ###############################################
        self.joint_type_weight_editor = JointTypeWeightEditor()
        tool_index = toolbox.addItem(
            self.joint_type_weight_editor, "Joint Type Weights"
        )
        toolbox.setItemToolTip(
            tool_index,
            "Adjust the relative importance of different joint types when exporting the model to MuJoCo. These weights influence splitting of kinematic loops and simplification.",
        )

        ###############################################
        # MuJoCo MJCF parameters
        ###############################################
        self.mjcf_options_editor = MJCFOptionsEditor()
        tool_index = toolbox.addItem(self.mjcf_options_editor, "MuJoCo MJCF Parameters")
        toolbox.setItemToolTip(
            tool_index,
            "Adjust MuJoCo simulation parameters.",
        )

        ###############################################
        # Debug
        ###############################################
        self.debug_options_editor = DebugOptionsEditor()
        tool_index = toolbox.addItem(self.debug_options_editor, "Debug Options")
        toolbox.setItemToolTip(
            tool_index,
            "Debug parameters.",
        )

    def accept(self) -> bool:
        """Called when user clicks the export button in the task panel

        Returns:
            True, if the task panel should be closed. False otherwise.
        """
        # Get all parameters from UI
        export_dir_options = self.export_dir_editor.get_options()
        mesh_export_options = self.mesh_export_options_editor.get_options()
        joint_type_weights_options = self.joint_type_weight_editor.get_options()
        mjcf_options = self.mjcf_options_editor.get_options()

        export_params = ExportParamsDict(
            **export_dir_options,  # type: ignore
            **mesh_export_options,
            **joint_type_weights_options,
            **mjcf_options,
        )
        # Trigger callback
        return self.on_accept_callback(export_params)

    def set_default_values(self) -> None:
        self.export_dir_editor.reset_to_defaults()
        self.joint_type_weight_editor.reset_to_defaults()
        self.mesh_export_options_editor.reset_to_defaults()
        self.mjcf_options_editor.reset_to_defaults()
