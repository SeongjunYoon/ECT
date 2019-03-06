from PyQt5.QtWidgets import QDialog
from PyQt5 import uic
from debugger.debug_fem_wizard import Debug_FEM_Wizard


form_class_subWindowDebugFEM = uic.loadUiType('./gui/debug_fem.ui')[0]

class Subwindow_Debug_FEM(QDialog, form_class_subWindowDebugFEM):
    def __init__(self, mainWindow, fem, parent=None):
        super().__init__(parent)
        self.mainWindow = mainWindow
        self.fem = fem
        self.setupUI()

    def setupUI(self):
        self.setupUi(self)
        self.setWindowTitle('Debug Mode - FEM')
        # Create control wizard
        self.debugFEMWizard = Debug_FEM_Wizard(self, self.fem)
        # Dialog button signal connection
        self.btnClose.clicked.connect(self.close)

    def closeEvent(self, QCloseEvent):
        self.mainWindow.show()



