from PyQt5.QtWidgets import *
from PyQt5 import uic
import main_wizard


form_class = uic.loadUiType('./gui/mainWindow.ui')[0]


class MainWindow(QMainWindow, form_class):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUI()

    def setupUI(self):
        self.setupUi(self)
        self.setWindowTitle('ECT_ver.1')
        # Create controller wizard
        self.mainWizard = main_wizard.MainWizard(self)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    excepthook = sys.excepthook
    sys.excepthook = lambda t, val, tb: excepthook(t, val, tb)
    myApp = MainWindow()
    myApp.show()
    sys.exit(app.exec_())


