from typing import TypedDict

from PySide import QtCore, QtWidgets

from freecad.assembly2mujoco.constants import DEFAULT_JOINT_TYPE_WEIGHTS

__all__ = ["JointTypeWeightEditor", "JointTypeWeightsDict"]


class JointTypeWeightsDict(TypedDict):
    Slider: float
    Revolute: float
    Cylindrical: float
    Ball: float
    Planar: float
    Fixed: float


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
                **{joint: spin.value() for joint, spin in self.inputs.items()}  # type: ignore
            )
        }
        return options

    def reset_to_defaults(self) -> None:
        for joint, default_value in self.default_weights.items():
            self.inputs[joint].setValue(default_value)
