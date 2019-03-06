from PyQt5.QtWidgets import QDialog
from PyQt5 import uic
from debugger.debug_raw_signals_wizard import Debug_RawSignals_Wizard


form_class_subWindowDebugRawSignals = uic.loadUiType('./gui/debug_raw_signals.ui')[0]


class Subwindow_Debug_Raw_Signals(QDialog, form_class_subWindowDebugRawSignals):
    def __init__(self, mainWindow, ect_controller, debug_mode, filter_output, parent=None):
        super().__init__(parent)
        self.mainWindow = mainWindow
        self.ect_controller = ect_controller
        self.debug_mode = debug_mode
        self.filter_output = filter_output
        self.setupUI()

    def setupUI(self):
        self.setupUi(self)
        self.setWindowTitle('Debug Mode - Raw Signals')
        # Crate control wizard
        self.debugRawSignalsWizard = Debug_RawSignals_Wizard(self, self.ect_controller, self.debug_mode, self.filter_output)
        # Dialog button signal connection
        self.btnClose.clicked.connect(self.close)

    def closeEvent(self, QCloseEvent):
        self.mainWindow.show()