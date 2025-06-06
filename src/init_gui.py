import os

import FreeCADGui as Gui

from src.constants import ICONS_DIR, WORKBENCH_ICON_FILE, WORKBENCH_MENU_NAME
from src.commands.export_command import MuJoCoExportCommand


class AssemblyExportToMuJoCoWorkbench(Gui.Workbench):
    MenuText = WORKBENCH_MENU_NAME
    Tooltip = "Export FreeCAD assemblies to MuJoCo"
    Icon = os.fspath(WORKBENCH_ICON_FILE)

    def Initialize(self):
        commands = [MuJoCoExportCommand.__name__]
        self.appendToolbar(f"{WORKBENCH_MENU_NAME} Tools", commands)
        self.appendMenu(f"{WORKBENCH_MENU_NAME} Tools", commands)

        Gui.addIconPath(os.fspath(ICONS_DIR))


workbench = AssemblyExportToMuJoCoWorkbench()
Gui.addWorkbench(workbench)
