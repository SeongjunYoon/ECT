from PyQt5.QtWidgets import QDialog
from PyQt5 import uic, QtCore
from viewer_wizard import Viewer_Wizard

form_class_viewer = uic.loadUiType('./gui/ect_viewer.ui')[0]

class Viewer(QDialog, form_class_viewer):
    def __init__(self, mainWizard, mainWindow, ect_controller, fem, parent=None):
        super().__init__(parent)
        self.mainWindow = mainWindow
        self.mainWizard = mainWizard
        self.ect_controller = ect_controller
        self.fem = fem
        self.setupUI()

    def setupUI(self):
        self.setupUi(self)
        self.setWindowTitle('ECT Viewer')
        self.setWindowFlags(QtCore.Qt.Window)
        # Create viewer wizard
        self.viewer = Viewer_Wizard(self, self.ect_controller, self.fem)
        # Hide mainWindow
        self.mainWindow.hide()

    def closeEvent(self, QCloseEvent):
        #
        self.viewer.stop_plot_update()
        #
        self.mainWizard.measure_status = 'Init'
        self.mainWindow.btnECTStart.setText('ECT Start')
        self.mainWindow.labelMeasStatus.setText('Press <ECT Start> Button to start measurement')
        self.mainWindow.show()