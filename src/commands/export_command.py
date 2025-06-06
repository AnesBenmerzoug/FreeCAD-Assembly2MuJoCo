import os

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtWidgets

from src.core.mujoco_exporter import MuJoCoExporter
from src.commands.export_panel import ExportTaskPanel
from src.constants import WORKBENCH_ICON_FILE


__all__ = ["MuJoCoExportCommand"]


class MuJoCoExportCommand:
    """
    Command to export Assembly to MuJoCo.
    """

    def GetResources(self):
        return {
            "Pixmap": os.fspath(WORKBENCH_ICON_FILE),
            "MenuText": "Export to MuJoCo",
            "ToolTip": "Export the selected assembly to MuJoCo MJCF format",
        }

    def Activated(self):
        """
        Execute the export command
        """
        # Check if there's a valid selection
        selection = Gui.Selection.getSelection()

        if not selection:
            QtWidgets.QMessageBox.warning(
                None, "No Selection", "Please select an assembly to export."
            )
            return

        # Check if selection is an assembly
        selected_obj = selection[0]
        if not self.is_assembly(selected_obj):
            QtWidgets.QMessageBox.warning(
                None, "Invalid Selection", "Please select a valid assembly object."
            )
            return

        # Show export dialog
        panel = ExportTaskPanel()
        if panel.exec_() == QtWidgets.QDialog.Accepted:
            try:
                # Perform the export
                exporter = MuJoCoExporter()
                output_path = panel.get_output_path()
                exporter.export_assembly(selected_obj, output_path)

                QtWidgets.QMessageBox.information(
                    None, "Export Successful", f"Assembly exported to: {output_path}"
                )
            except Exception as e:
                QtWidgets.QMessageBox.critical(
                    None, "Export Failed", f"Failed to export assembly: {str(e)}"
                )

    def IsActive(self) -> bool:
        """
        Define when the command should be active
        """
        return App.ActiveDocument is not None

    def is_assembly(self, obj) -> bool:
        """
        Check if the object is a valid assembly
        """
        return obj.TypeId == "Assembly::AssemblyObject"
