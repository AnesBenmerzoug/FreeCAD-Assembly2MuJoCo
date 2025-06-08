import os

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtWidgets

from freecad.assembly2mujoco.core.mujoco_exporter import MuJoCoExporter
from freecad.assembly2mujoco.commands.base import BaseCommand
from freecad.assembly2mujoco.commands.export_panel import (
    ExportTaskPanel,
    ExportParamsDict,
)
from freecad.assembly2mujoco.constants import WORKBENCH_ICON_FILE


__all__ = ["MuJoCoExportCommand"]


class MuJoCoExportCommand(BaseCommand):
    """
    Command to export Assembly to MuJoCo.
    """

    def GetResources(self):
        return {
            "Pixmap": os.fspath(WORKBENCH_ICON_FILE),
            "MenuText": "Export to MuJoCo",
            "ToolTip": "Export the selected assembly as meshes and creates an MJCF file for MuJoCo",
        }

    def Activated(self) -> None:
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
        def on_accept_callback(export_params: ExportParamsDict) -> bool:
            try:
                # Perform the export
                exporter = MuJoCoExporter(**export_params)
                exporter.export_assembly(selected_obj)

                export_dir = export_params["export_dir"]
                QtWidgets.QMessageBox.information(
                    None, "Export Successful", f"Assembly exported to: {export_dir}"
                )
                return True
            except Exception as e:
                QtWidgets.QMessageBox.critical(
                    None, "Export Failed", f"Failed to export assembly: {str(e)}"
                )
                return False

        panel = ExportTaskPanel(on_accept_callback=on_accept_callback)
        Gui.Control.showDialog(panel)

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
