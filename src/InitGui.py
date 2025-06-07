import os

import FreeCADGui as Gui

from src.constants import ICONS_DIR, WORKBENCH_ICON_FILE, WORKBENCH_MENU_NAME
from src.commands import get_all_commands
from src.utils.helpers import log_message


log_message(f"Test: {WORKBENCH_MENU_NAME}")


class AssemblyExportToMuJoCoWorkbench(Gui.Workbench):
    MenuText = WORKBENCH_MENU_NAME
    Tooltip = "Export FreeCAD assemblies to MuJoCo"
    Icon = os.fspath(WORKBENCH_ICON_FILE)

    def Initialize(self):
        log_message("Initializing workbench")
        command_names = []
        for command_name, command_cls in get_all_commands().items():
            command_names.append(command_name)
            Gui.addCommand(command_name, command_cls())
        log_message(f"Adding {len(command_names)} commands")
        self.appendToolbar(f"{WORKBENCH_MENU_NAME} Tools", command_names)
        self.appendMenu(f"{WORKBENCH_MENU_NAME} Tools", command_names)
        log_message(f"Using icons in {ICONS_DIR}")
        Gui.addIconPath(os.fspath(ICONS_DIR))


workbench = AssemblyExportToMuJoCoWorkbench()
Gui.addWorkbench(workbench)
