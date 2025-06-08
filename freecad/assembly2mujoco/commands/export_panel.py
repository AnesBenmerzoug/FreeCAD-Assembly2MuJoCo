import os
from pathlib import Path
from typing import Callable, Literal, TypedDict

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtGui, QtWidgets

from freecad.assembly2mujoco.constants import (
    DEFAULT_STL_MESH_ANGULAR_DEFLECTION,
    DEFAULT_STL_MESH_LINEAR_DEFLECTION,
    DEFAULT_MESH_EXPORT_FORMAT,
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_TIMESTEP,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["ExportTaskPanel", "ExportParamsDict"]


class ExportParamsDict(TypedDict):
    export_dir: Path
    mesh_export_format: Literal["STL", "OBJ"]
    stl_mesh_linear_deflection: float
    stl_mesh_angular_deflection: float
    mjcf_timestep: float
    mjcf_damping: float
    mjcf_armature: float
    mjcf_integrator: Literal["implicitfast", "Euler", "implicit", "RK4"]
    mjcf_solver: Literal["PGS", "CG", "Newton"]


class ExportTaskPanel:
    def __init__(self, on_accept_callback: Callable[[ExportParamsDict], bool]):
        self.on_accept_callback = on_accept_callback
        # Get current document and its path
        self.doc = App.ActiveDocument
        if not self.doc:
            log_message("No active document", level="error")
            raise RuntimeError("No active document")

        self.default_dir = Path(self.doc.FileName).parent / "mujoco"
        self.setup_ui()
        self.set_default_values()

    def setup_ui(self) -> None:
        # Create the form widget
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Export FreeCAD Assembly to MuJoCo")

        # Create main layout
        main_layout = QtWidgets.QVBoxLayout(self.form)

        ###############################################
        # Directory selection section
        ###############################################
        dir_layout = QtWidgets.QHBoxLayout()
        dir_group = QtWidgets.QGroupBox("Export Directory")
        dir_group.setLayout(dir_layout)
        main_layout.addWidget(dir_group)

        self.dir_edit = QtWidgets.QLineEdit(os.fspath(self.default_dir))
        dir_layout.addWidget(self.dir_edit)

        browse_button = QtWidgets.QPushButton("Browse...")
        browse_button.clicked.connect(self.browse_export_directory)
        dir_layout.addWidget(browse_button)

        ###############################################
        # Mesh Export options
        ###############################################
        mesh_layout = QtWidgets.QFormLayout()
        mesh_group = QtWidgets.QGroupBox("Mesh Export Options")
        mesh_group.setLayout(mesh_layout)
        main_layout.addWidget(mesh_group)

        ### Mesh Format
        self.mesh_export_format_combo = QtWidgets.QComboBox()
        mesh_export_format_values = ["STL", "OBJ"]
        self.mesh_export_format_combo.addItems(mesh_export_format_values)
        self.mesh_export_format_combo.currentTextChanged.connect(
            self.mesh_export_format_changed_callback
        )
        mesh_layout.addRow("Format:", self.mesh_export_format_combo)

        ## STL Mesh quality
        self.stl_mesh_widget_container = QtWidgets.QWidget()
        mesh_layout.addRow("", self.stl_mesh_widget_container)
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

        ###############################################
        # MuJoCo MJCF parameters
        ###############################################
        mjcf_layout = QtWidgets.QFormLayout()
        mjcf_group = QtWidgets.QGroupBox("MuJoCo MJCF Parameters")
        mjcf_group.setLayout(mjcf_layout)
        main_layout.addWidget(mjcf_group)

        ## Timestep
        self.timestep_spin = QtWidgets.QDoubleSpinBox()
        self.timestep_spin.setRange(0.0001, 0.05)
        self.timestep_spin.setSingleStep(0.0001)
        self.timestep_spin.setDecimals(4)
        mjcf_layout.addRow("Timestep:", self.timestep_spin)

        ## Damping
        self.damping_spin = QtWidgets.QDoubleSpinBox()
        self.damping_spin.setRange(0.0, 10.0)
        self.damping_spin.setSingleStep(0.1)
        self.damping_spin.setDecimals(3)
        mjcf_layout.addRow("Default Damping:", self.damping_spin)

        ## Armature
        self.armature_spin = QtWidgets.QDoubleSpinBox()
        self.armature_spin.setRange(0.0, 1.0)
        self.armature_spin.setSingleStep(0.01)
        self.armature_spin.setDecimals(3)
        mjcf_layout.addRow("Default Armature:", self.armature_spin)

        # Additional common parameters
        self.integrator_combo = QtWidgets.QComboBox()
        integrator_values = ["implicitfast", "Euler", "implicit", "RK4"]
        self.integrator_combo.addItems(integrator_values)
        mjcf_layout.addRow("Integrator:", self.integrator_combo)

        self.solver_combo = QtWidgets.QComboBox()
        solver_values = ["PGS", "CG", "Newton"]
        self.solver_combo.addItems(solver_values)
        mjcf_layout.addRow("Solver:", self.solver_combo)

        ###############################################
        # Debug
        ###############################################
        debug_layout = QtWidgets.QVBoxLayout()
        debug_group = QtWidgets.QGroupBox("Debug Options")
        debug_group.setLayout(debug_layout)
        main_layout.addWidget(debug_group)

        clear_report_button = QtWidgets.QPushButton("Clear Report View")
        clear_report_button.clicked.connect(self.clear_report_view)
        debug_layout.addWidget(clear_report_button)

    def accept(self) -> bool:
        """Called when user clicks the export button in the task panel

        Returns:
            True, if the task panel should be closed. False otherwise.
        """
        # Get all parameters from UI
        export_dir = Path(self.dir_edit.text())
        export_dir.mkdir(exist_ok=True)

        # Validate export directory
        if not os.path.isdir(export_dir):
            QtWidgets.QMessageBox.warning(
                self.form,
                "Directory Error",
                f"The directory '{export_dir}' does not exist. Please select a valid directory.",
            )
            return False

        # Collect parameters
        export_params = ExportParamsDict(
            export_dir=export_dir,
            mesh_export_format=self.mesh_export_format_combo.currentText(),
            stl_mesh_linear_deflection=self.stl_mesh_linear_deflection_spin.value(),
            stl_mesh_angular_deflection=self.stl_mesh_angular_deflection_spin.value(),
            mjcf_timestep=self.timestep_spin.value(),
            mjcf_damping=self.damping_spin.value(),
            mjcf_armature=self.armature_spin.value(),
            mjcf_integrator=self.integrator_combo.currentText(),
            mjcf_solver=self.solver_combo.currentText(),
        )
        # Trigger callback
        return self.on_accept_callback(export_params)

    def set_default_values(self) -> None:
        # Mesh
        self.mesh_export_format_combo.setCurrentText(DEFAULT_MESH_EXPORT_FORMAT)
        # STL
        self.stl_mesh_linear_deflection_spin.setValue(
            DEFAULT_STL_MESH_LINEAR_DEFLECTION
        )
        self.stl_mesh_angular_deflection_spin.setValue(
            DEFAULT_STL_MESH_ANGULAR_DEFLECTION
        )
        # MJCF
        self.timestep_spin.setValue(DEFAULT_MJCF_TIMESTEP)
        self.damping_spin.setValue(DEFAULT_MJCF_DAMPING)
        self.armature_spin.setValue(DEFAULT_MJCF_ARMATURE)
        self.integrator_combo.setCurrentText(DEFAULT_MJCF_INTEGRATOR)
        self.solver_combo.setCurrentText(DEFAULT_MJCF_SOLVER)

    def browse_export_directory(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self.form, "Select Export Directory", self.dir_edit.text()
        )
        if directory:
            self.dir_edit.setText(directory)

    def mesh_export_format_changed_callback(self, value: Literal["STL", "OBJ"]) -> None:
        if value == "STL":
            self.stl_mesh_widget_container.show()
        elif value == "OBJ":
            self.stl_mesh_widget_container.hide()
        else:
            raise ValueError(f"Unknown mesh format '{value}'")

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
