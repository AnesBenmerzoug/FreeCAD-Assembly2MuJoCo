import os
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtGui, QtWidgets

from freecad.assembly2mujoco.constants import (
    DEFAULT_ANGULAR_DEFLECTION,
    DEFAULT_LINEAR_DEFLECTION,
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_TIMESTEP,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["ExportTaskPanel"]


class ExportTaskPanel:
    def __init__(self):
        # Get current document and its path
        self.doc = App.ActiveDocument
        if not self.doc:
            log_message("No active document", level="error")
            raise RuntimeError("No active document")

        self.default_dir = Path(self.doc.FileName).parent / "mujoco"

        self.setup_ui()

    def setup_ui(self) -> None:
        # Create the form widget
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle("Export FreeCAD Assembly to MuJoCo")

        # Create main layout
        main_layout = QtWidgets.QVBoxLayout(self.form)

        # Directory selection section
        dir_group = QtWidgets.QGroupBox("Export Directory")
        dir_layout = QtWidgets.QHBoxLayout()

        self.dir_edit = QtWidgets.QLineEdit(os.fspath(self.default_dir))
        dir_layout.addWidget(self.dir_edit)

        browse_button = QtWidgets.QPushButton("Browse...")
        browse_button.clicked.connect(self.browse_export_directory)
        dir_layout.addWidget(browse_button)

        dir_group.setLayout(dir_layout)
        main_layout.addWidget(dir_group)

        # STL Export options
        stl_group = QtWidgets.QGroupBox("STL Export Options")
        stl_layout = QtWidgets.QFormLayout()

        ## Mesh quality
        self.linear_deflection_spin = QtWidgets.QDoubleSpinBox()
        self.linear_deflection_spin.setRange(0.01, 1)
        self.linear_deflection_spin.setSingleStep(0.01)
        self.linear_deflection_spin.setDecimals(2)
        self.linear_deflection_spin.setValue(DEFAULT_LINEAR_DEFLECTION)
        stl_layout.addRow("Linear Deflection:", self.linear_deflection_spin)

        self.angular_deflection_spin = QtWidgets.QDoubleSpinBox()
        self.angular_deflection_spin.setRange(0.5, 5.0)
        self.angular_deflection_spin.setSingleStep(0.1)
        self.angular_deflection_spin.setDecimals(1)
        self.angular_deflection_spin.setValue(DEFAULT_ANGULAR_DEFLECTION)
        stl_layout.addRow("Angular Deflection:", self.angular_deflection_spin)

        stl_group.setLayout(stl_layout)
        main_layout.addWidget(stl_group)

        # MuJoCo MJCF parameters
        mjcf_group = QtWidgets.QGroupBox("MuJoCo MJCF Parameters")
        mjcf_layout = QtWidgets.QFormLayout()

        ## Timestep
        self.timestep_spin = QtWidgets.QDoubleSpinBox()
        self.timestep_spin.setRange(0.0001, 0.05)
        self.timestep_spin.setSingleStep(0.0001)
        self.timestep_spin.setDecimals(4)
        self.timestep_spin.setValue(DEFAULT_MJCF_TIMESTEP)
        mjcf_layout.addRow("Timestep:", self.timestep_spin)

        ## Damping
        self.damping_spin = QtWidgets.QDoubleSpinBox()
        self.damping_spin.setRange(0.0, 10.0)
        self.damping_spin.setSingleStep(0.1)
        self.damping_spin.setDecimals(3)
        self.damping_spin.setValue(DEFAULT_MJCF_DAMPING)
        mjcf_layout.addRow("Default Damping:", self.damping_spin)

        ## Armature
        self.armature_spin = QtWidgets.QDoubleSpinBox()
        self.armature_spin.setRange(0.0, 1.0)
        self.armature_spin.setSingleStep(0.01)
        self.armature_spin.setDecimals(3)
        self.armature_spin.setValue(DEFAULT_MJCF_ARMATURE)
        mjcf_layout.addRow("Default Armature:", self.armature_spin)

        # Additional common parameters
        self.integrator_combo = QtWidgets.QComboBox()
        integrator_values = ["implicitfast", "Euler", "implicit", "RK4"]
        integrator_values = [DEFAULT_MJCF_INTEGRATOR] + list(
            set(integrator_values).difference([DEFAULT_MJCF_INTEGRATOR])
        )
        self.integrator_combo.addItems(integrator_values)
        mjcf_layout.addRow("Integrator:", self.integrator_combo)

        self.solver_combo = QtWidgets.QComboBox()
        solver_values = ["PGS", "CG", "Newton"]
        solver_values = [DEFAULT_MJCF_SOLVER] + list(
            set(solver_values).difference([DEFAULT_MJCF_SOLVER])
        )
        self.solver_combo.addItems(solver_values)
        mjcf_layout.addRow("Solver:", self.solver_combo)

        mjcf_group.setLayout(mjcf_layout)
        main_layout.addWidget(mjcf_group)

        # Debug
        debug_group = QtWidgets.QGroupBox("Debug Options")
        debug_layout = QtWidgets.QVBoxLayout()

        clear_report_button = QtWidgets.QPushButton("Clear Report View")
        clear_report_button.clicked.connect(self.clear_report_view)
        debug_layout.addWidget(clear_report_button)

        debug_group.setLayout(debug_layout)
        main_layout.addWidget(debug_group)

    def browse_export_directory(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self.form, "Select Export Directory", self.dir_edit.text()
        )
        if directory:
            self.dir_edit.setText(directory)

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
