import os
from pathlib import Path
from typing import Any, Callable, Literal, TypedDict

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtCore, QtGui, QtWidgets

from freecad.assembly2mujoco.constants import (
    DEFAULT_STL_MESH_ANGULAR_DEFLECTION,
    DEFAULT_STL_MESH_LINEAR_DEFLECTION,
    DEFAULT_MESH_EXPORT_FORMAT,
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_TIMESTEP,
    DEFAULT_JOINT_TYPE_WEIGHTS,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["ExportTaskPanel", "ExportParamsDict"]


class JointTypeWeightsDict(TypedDict):
    Slider: float
    Revolute: float
    Cylindrical: float
    Ball: float
    Planar: float
    Fixed: float


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


class ExportDirEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)
        self.doc = App.ActiveDocument
        self.default_dir = Path(self.doc.FileName).parent / "mujoco"

        # Use QVBoxLayout as top-level layout
        layout = QtWidgets.QVBoxLayout(self)

        # Use QHBoxLayout for input fields
        input_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(input_layout)

        self.dir_edit = QtWidgets.QLineEdit(os.fspath(self.default_dir))
        input_layout.addWidget(self.dir_edit)

        browse_button = QtWidgets.QPushButton("Browse...")
        browse_button.clicked.connect(self.browse_export_directory)
        input_layout.addWidget(browse_button)

        # Reset button
        reset_button = QtWidgets.QPushButton("Reset to Defaults")
        reset_button.clicked.connect(self.reset_to_defaults)
        layout.addWidget(reset_button, alignment=QtCore.Qt.AlignRight)

    def browse_export_directory(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Export Directory", self.dir_edit.text()
        )
        if directory:
            self.dir_edit.setText(directory)

    def get_options(self) -> dict[str, Path]:
        export_dir = Path(self.dir_edit.text())
        export_dir.mkdir(exist_ok=True)

        # Validate export directory
        if not os.path.isdir(export_dir):
            QtWidgets.QMessageBox.warning(
                self,
                "Directory Error",
                f"The directory '{export_dir}' does not exist. Please select a valid directory.",
            )
            return False

        options = {"export_dir": export_dir}
        return options

    def reset_to_defaults(self) -> None:
        self.dir_edit.setText(os.fspath(self.default_dir))


class MeshExportOptionsEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)

        # Use QVBoxLayout as top-level layout
        layout = QtWidgets.QVBoxLayout(self)

        # Use QFormLayout for input fields
        form_layout = QtWidgets.QFormLayout()
        layout.addLayout(form_layout)

        ### Mesh Format
        self.mesh_export_format_combo = QtWidgets.QComboBox()
        mesh_export_format_values = ["STL", "OBJ"]
        self.mesh_export_format_combo.addItems(mesh_export_format_values)
        self.mesh_export_format_combo.currentTextChanged.connect(
            self.mesh_export_format_changed_callback
        )
        form_layout.addRow("Format:", self.mesh_export_format_combo)

        ## STL Mesh quality
        self.stl_mesh_widget_container = QtWidgets.QWidget()
        form_layout.addRow("", self.stl_mesh_widget_container)
        mesh_stl_layout = QtWidgets.QFormLayout()
        mesh_stl_layout.setContentsMargins(0, 0, 0, 0)
        self.stl_mesh_widget_container.setLayout(mesh_stl_layout)

        self.stl_mesh_linear_deflection_spin = QtWidgets.QDoubleSpinBox()
        self.stl_mesh_linear_deflection_spin.setRange(0.01, 1)
        self.stl_mesh_linear_deflection_spin.setSingleStep(0.01)
        self.stl_mesh_linear_deflection_spin.setDecimals(2)
        mesh_stl_layout.addRow(
            "Linear Deflection:", self.stl_mesh_linear_deflection_spin
        )

        self.stl_mesh_angular_deflection_spin = QtWidgets.QDoubleSpinBox()
        self.stl_mesh_angular_deflection_spin.setRange(0.5, 5.0)
        self.stl_mesh_angular_deflection_spin.setSingleStep(0.1)
        self.stl_mesh_angular_deflection_spin.setDecimals(1)
        mesh_stl_layout.addRow(
            "Angular Deflection:", self.stl_mesh_angular_deflection_spin
        )

        # Reset button
        reset_button = QtWidgets.QPushButton("Reset to Defaults")
        reset_button.clicked.connect(self.reset_to_defaults)
        layout.addWidget(reset_button, alignment=QtCore.Qt.AlignRight)

    def get_options(self) -> dict[str, Any]:
        return dict(
            mesh_export_format=self.mesh_export_format_combo.currentText(),
            stl_mesh_linear_deflection=self.stl_mesh_linear_deflection_spin.value(),
            stl_mesh_angular_deflection=self.stl_mesh_angular_deflection_spin.value(),
        )

    def mesh_export_format_changed_callback(self, value: Literal["STL", "OBJ"]) -> None:
        if value == "STL":
            self.stl_mesh_widget_container.show()
        elif value == "OBJ":
            self.stl_mesh_widget_container.hide()
        else:
            raise ValueError(f"Unknown mesh format '{value}'")

    def reset_to_defaults(self) -> None:
        # Mesh
        self.mesh_export_format_combo.setCurrentText(DEFAULT_MESH_EXPORT_FORMAT)
        # STL
        self.stl_mesh_linear_deflection_spin.setValue(
            DEFAULT_STL_MESH_LINEAR_DEFLECTION
        )
        self.stl_mesh_angular_deflection_spin.setValue(
            DEFAULT_STL_MESH_ANGULAR_DEFLECTION
        )


class MJCFOptionsEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)

        # Use QVBoxLayout as top-level layout
        layout = QtWidgets.QVBoxLayout(self)

        # Use QFormLayout for input fields
        form_layout = QtWidgets.QFormLayout()
        layout.addLayout(form_layout)

        ## Timestep
        self.timestep_spin = QtWidgets.QDoubleSpinBox()
        self.timestep_spin.setRange(0.0001, 0.05)
        self.timestep_spin.setSingleStep(0.0001)
        self.timestep_spin.setDecimals(4)
        form_layout.addRow("Timestep:", self.timestep_spin)

        ## Damping
        self.damping_spin = QtWidgets.QDoubleSpinBox()
        self.damping_spin.setRange(0.0, 10.0)
        self.damping_spin.setSingleStep(0.1)
        self.damping_spin.setDecimals(3)
        form_layout.addRow("Default Damping:", self.damping_spin)

        ## Armature
        self.armature_spin = QtWidgets.QDoubleSpinBox()
        self.armature_spin.setRange(0.0, 1.0)
        self.armature_spin.setSingleStep(0.01)
        self.armature_spin.setDecimals(3)
        form_layout.addRow("Default Armature:", self.armature_spin)

        # Additional common parameters
        self.integrator_combo = QtWidgets.QComboBox()
        integrator_values = ["implicitfast", "Euler", "implicit", "RK4"]
        self.integrator_combo.addItems(integrator_values)
        form_layout.addRow("Integrator:", self.integrator_combo)

        self.solver_combo = QtWidgets.QComboBox()
        solver_values = ["PGS", "CG", "Newton"]
        self.solver_combo.addItems(solver_values)
        form_layout.addRow("Solver:", self.solver_combo)

        # Reset button
        reset_button = QtWidgets.QPushButton("Reset to Defaults")
        reset_button.clicked.connect(self.reset_to_defaults)
        layout.addWidget(reset_button, alignment=QtCore.Qt.AlignRight)

    def get_options(self) -> dict[str, Any]:
        options = dict(
            mjcf_timestep=self.timestep_spin.value(),
            mjcf_damping=self.damping_spin.value(),
            mjcf_armature=self.armature_spin.value(),
            mjcf_integrator=self.integrator_combo.currentText(),
            mjcf_solver=self.solver_combo.currentText(),
        )
        return options

    def reset_to_defaults(self) -> None:
        self.timestep_spin.setValue(DEFAULT_MJCF_TIMESTEP)
        self.damping_spin.setValue(DEFAULT_MJCF_DAMPING)
        self.armature_spin.setValue(DEFAULT_MJCF_ARMATURE)
        self.integrator_combo.setCurrentText(DEFAULT_MJCF_INTEGRATOR)
        self.solver_combo.setCurrentText(DEFAULT_MJCF_SOLVER)


class JointTypeWeightEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)
        self.default_weights = DEFAULT_JOINT_TYPE_WEIGHTS.copy()
        self.inputs = {}

        # Use QVBoxLayout as top-level layout
        layout = QtWidgets.QVBoxLayout(self)

        # Use QFormLayout for input fields
        form_layout = QtWidgets.QFormLayout()
        layout.addLayout(form_layout)

        for joint_type, value in self.default_weights.items():
            spin = QtWidgets.QDoubleSpinBox()
            spin.setMinimum(0.0)
            spin.setMaximum(100.0)
            spin.setSingleStep(1.0)
            spin.setValue(value)
            self.inputs[joint_type] = spin
            form_layout.addRow(joint_type + ":", spin)

        # Reset button
        reset_button = QtWidgets.QPushButton("Reset to Defaults")
        reset_button.clicked.connect(self.reset_to_defaults)
        layout.addWidget(reset_button, alignment=QtCore.Qt.AlignRight)

    def get_options(self) -> dict[str, JointTypeWeightsDict]:
        options = {
            "joint_type_weights": JointTypeWeightsDict(
                **{joint: spin.value() for joint, spin in self.inputs.items()}
            )
        }
        return options

    def reset_to_defaults(self) -> None:
        for joint, default_value in self.default_weights.items():
            self.inputs[joint].setValue(default_value)


class DebugOptionsEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)

        layout = QtWidgets.QVBoxLayout(self)
        clear_report_button = QtWidgets.QPushButton("Clear Report View")
        clear_report_button.clicked.connect(self.clear_report_view)
        layout.addWidget(clear_report_button)

    def clear_report_view(self) -> None:
        """Clear the contents of the FreeCAD Report View"""
        main_window = Gui.getMainWindow()
        if not main_window:
            log_message("Cannot find main window to clear report view", level="warning")
            return

        try:
            r = main_window.findChild(QtGui.QTextEdit, "Report view")
            r.clear()
            return
        except Exception as e:
            log_message(
                f"Error while trying to clear Report view: {str(e)}", level="error"
            )


class ExportTaskPanel:
    def __init__(self, on_accept_callback: Callable[[ExportParamsDict], bool]):
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
        self.export_dir_editor = ExportDirEditor()
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
            "MISSING TOOLTIP.",
        )

        ###############################################
        # Debug
        ###############################################
        self.debug_options_editor = DebugOptionsEditor()
        tool_index = toolbox.addItem(self.debug_options_editor, "Debug Options")
        toolbox.setItemToolTip(
            tool_index,
            "MISSING TOOLTIP.",
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
            **export_dir_options,
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
