from typing import Any, Literal

from PySide import QtCore, QtWidgets

from freecad.assembly2mujoco.constants import (
    DEFAULT_STL_MESH_ANGULAR_DEFLECTION,
    DEFAULT_STL_MESH_LINEAR_DEFLECTION,
    DEFAULT_MESH_EXPORT_FORMAT,
)

__all__ = ["MeshExportOptionsEditor"]


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
