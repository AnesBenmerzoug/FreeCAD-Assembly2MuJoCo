import os
from pathlib import Path

from PySide import QtWidgets, QtCore

__all__ = ["ExportDirEditor"]


class ExportDirEditor(QtWidgets.QWidget):
    def __init__(self, *args, doc) -> None:
        super().__init__(*args)
        self.doc = doc
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
        export_dir.mkdir(parents=True, exist_ok=True)

        # Validate export directory
        if not os.path.isdir(export_dir):
            QtWidgets.QMessageBox.warning(
                self,
                "Directory Error",
                f"The directory '{export_dir}' does not exist. Please select a valid directory.",
            )
            return {}

        options = {"export_dir": export_dir}
        return options

    def reset_to_defaults(self) -> None:
        self.dir_edit.setText(os.fspath(self.default_dir))
