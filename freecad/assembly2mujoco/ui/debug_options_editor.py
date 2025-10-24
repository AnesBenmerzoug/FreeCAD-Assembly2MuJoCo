import FreeCADGui as Gui
from PySide import QtGui, QtWidgets

from freecad.assembly2mujoco.utils.helpers import log_message


__all__ = ["DebugOptionsEditor"]


class DebugOptionsEditor(QtWidgets.QWidget):
    def __init__(self, *args) -> None:
        super().__init__(*args)

        layout = QtWidgets.QVBoxLayout(self)
        clear_report_button = QtWidgets.QPushButton("Clear Report View")
        clear_report_button.clicked.connect(self.clear_report_view)
        layout.addWidget(clear_report_button)

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
