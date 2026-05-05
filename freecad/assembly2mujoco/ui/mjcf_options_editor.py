from typing import Any

from PySide import QtCore, QtWidgets

from freecad.assembly2mujoco.constants import (
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    DEFAULT_MJCF_TIMESTEP,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_ADD_SITES,
)

__all__ = ["MJCFOptionsEditor"]


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

        # Sites
        self.site_check = QtWidgets.QCheckBox()
        self.site_check.setCheckState(
            QtCore.Qt.CheckState.Checked
            if DEFAULT_MJCF_ADD_SITES
            else QtCore.Qt.CheckState.Unchecked
        )
        self.site_check.setToolTip("Add a site to each body")
        form_layout.addRow("Sites:", self.site_check)

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
            mjcf_add_sites=self.site_check.isChecked(),
        )
        return options

    def reset_to_defaults(self) -> None:
        self.timestep_spin.setValue(DEFAULT_MJCF_TIMESTEP)
        self.damping_spin.setValue(DEFAULT_MJCF_DAMPING)
        self.armature_spin.setValue(DEFAULT_MJCF_ARMATURE)
        self.integrator_combo.setCurrentText(DEFAULT_MJCF_INTEGRATOR)
        self.solver_combo.setCurrentText(DEFAULT_MJCF_SOLVER)
        self.site_check.setCheckState(
            QtCore.Qt.CheckState.Checked
            if DEFAULT_MJCF_ADD_SITES
            else QtCore.Qt.CheckState.Unchecked
        )
