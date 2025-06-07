import os
from pathlib import Path

import FreeCADGui as Gui

from src.commands import get_all_commands
from src.utils.helpers import log_message


ROOT_DIR = Path(__file__).parents[1].resolve()
SRC_DIR = ROOT_DIR / "src"
RESOURCES_DIR = SRC_DIR / "resources"
ICONS_DIR = RESOURCES_DIR / "icons"
WORKBENCH_ICON_FILE = os.fspath(ICONS_DIR / "assembly-to-mujoco-icon.svg")
WORKBENCH_MENU_NAME = "Assembly To MuJoCo"


class AssemblyExportToMuJoCoWorkbench(Gui.Workbench):
    MenuText = WORKBENCH_MENU_NAME
    Tooltip = "Export FreeCAD assemblies to MuJoCo"
    Icon = WORKBENCH_ICON_FILE

    def Initialize(self):
        log_message("Initializing workbench")
        command_names = []
        for command_name, command_cls in get_all_commands().items():
            command_names.append(command_name)
            Gui.addCommand(command_name, command_cls())
        log_message(f"Adding {len(command_names)} commands")
        self.appendToolbar(WORKBENCH_MENU_NAME, command_names)
        self.appendMenu(WORKBENCH_MENU_NAME, command_names)
        log_message(f"Using icons in {ICONS_DIR}")
        Gui.addIconPath(os.fspath(ICONS_DIR))


workbench = AssemblyExportToMuJoCoWorkbench()
Gui.addWorkbench(workbench)
