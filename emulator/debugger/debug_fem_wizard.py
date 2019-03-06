from PyQt5.QtWidgets import QFileDialog
from inverse_solver import inverse_solver
import numpy as np
import os

e0 = 8.85418781762039 * 0.000000000001  # Unit: F/m

class Debug_FEM_Wizard(QFileDialog):
    def __init__(self, debug_fem_subwindow, fem, parent=None):
        super().__init__(parent)
        # Get instances
        self.subwindow = debug_fem_subwindow
        self.fem = fem
        self.inv_solver = inverse_solver.InverseSolver()
        # Set signal connection
        self.setSignalConnect()
        # Disable forward cap calculation, fileopen and image buttons at first
        self.subwindow.btnForwardCapArray.setEnabled(False)
        self.subwindow.btnFileopen.setEnabled(False)
        self.subwindow.btnLBP.setEnabled(False)
        self.subwindow.btnLandweber.setEnabled(False)
        # Initialize sensitivity matrix loading progress bar
        self.subwindow.progBarSensitivity.setValue(0)
        # Initialize inverse algorithm parameters
        self.initInverseAlgoParam()

    def setSignalConnect(self):
        self.subwindow.btnSensitivityMap.clicked.connect(self.btnSensitivityMapClicked)
        self.subwindow.btnForwardCapArray.clicked.connect(self.btnForwardCapArrayClicked)
        self.subwindow.btnFileopen.clicked.connect(self.btnFileopenClicked)
        self.subwindow.btnClearResult.clicked.connect(self.btnClearResultClicked)
        self.subwindow.btnLBP.clicked.connect(self.btnLBPClicked)
        self.subwindow.btnLandweber.clicked.connect(self.btnLandweberClicked)

    def btnClearResultClicked(self):
        file_path = './fem_data/inverse_result/'
        file_path_LBP = file_path + 'LBP.npy'
        file_path_Landweber = file_path + 'Landweber.npy'
        if os.path.isfile(file_path_LBP):
            print('Delete previous LBP inverse result...')
            os.remove(file_path_LBP)
        if os.path.isfile(file_path_Landweber):
            print('Delete previous Landweber inverse result...')
            os.remove(file_path_Landweber)

    def btnSensitivityMapClicked(self):
        # Reset Progress bar
        self.subwindow.progBarSensitivity.setValue(0)
        # Disable all push-buttons
        self.subwindow.btnSensitivityMap.setEnabled(False)
        self.subwindow.btnForwardCapArray.setEnabled(False)
        self.subwindow.btnFileopen.setEnabled(False)
        self.subwindow.btnLBP.setEnabled(False)
        self.subwindow.btnLandweber.setEnabled(False)
        # Change sensitivity pushbutton text & do calculation
        self.subwindow.btnSensitivityMap.setText('Calculating')
        self.fem.calculate_sensitivity_map(self.subwindow.progBarSensitivity)
        # Enable sensitivity & fileopen push-buttons
        self.subwindow.btnSensitivityMap.setText('Calculate Sensitivity Map')
        self.subwindow.btnSensitivityMap.setEnabled(True)
        self.subwindow.btnForwardCapArray.setEnabled(True)
        self.subwindow.btnFileopen.setEnabled(True)

    def btnForwardCapArrayClicked(self):
        self.fem.debug_plot_forward_cap_array()

    def btnFileopenClicked(self):
        file_path = self.openFileNameDialog()
        self.subwindow.txtFilename.setText(file_path)
        self.C_phantom = np.loadtxt(file_path)
        self.C_phantom = self.C_phantom.reshape((self.C_phantom.size, 1))
        self.subwindow.btnLBP.setEnabled(True)
        self.subwindow.btnLandweber.setEnabled(True)

    def initInverseAlgoParam(self):
        # Tab LBP
        self.subwindow.lineLBPColorMap.setText('seismic')
        self.subwindow.lineLBPOpacity.setText('0.1')
        self.subwindow.lineLBPImageNorm.setText('linear')
        self.subwindow.lineLBPImageFilter.setText('threshold')
        # Tab Landweber
        self.subwindow.lineLandweberIterNum.setText('100')
        self.subwindow.lineLandweberColorMap.setText('seismic')
        self.subwindow.lineLandweberOpacity.setText('0.1')
        self.subwindow.lineLandweberImageNorm.setText('linear')
        self.subwindow.lineLandweberImageFilter.setText('threshold')

    def btnLBPClicked(self):
        # Load parameters
        cmap = self.subwindow.lineLBPColorMap.text()
        base_opacity = float(self.subwindow.lineLBPOpacity.text())
        norm_method = self.subwindow.lineLBPImageNorm.text()
        filter = self.subwindow.lineLBPImageFilter.text()
        opacity_param = self.subwindow.lineLBPOpacityParam.text()
        # Check previous result
        file_path = './fem_data/inverse_result/' + 'LBP.npy'
        if os.path.isfile(file_path):
            print('Loading previous inverse result')
            phantom_norm_image = np.load(file_path)
        else:
            print('Solving inverse problem')
            # Load capacitance array & normalization
            C_phantom_norm = (self.C_phantom - self.fem.Cmin) / (self.fem.Cmax - self.fem.Cmin)
            # Calculate by LBP algorithm
            phantom_norm_image = self.inv_solver.LBP(self.fem.sensitivity_map, C_phantom_norm)
            # Save inverse result
            np.save(file_path, phantom_norm_image)

            # (Optional) Quantitative values
            phantom_image = self.inv_solver.LBP(self.fem.sensitivity_map, self.C_phantom)
            np.savetxt('./fem_data/inverse_result/LBP_rel.txt', phantom_image / e0)
            np.savetxt('./fem_data/inverse_result/LBP_rel_positive_only.txt',
                       phantom_image[phantom_image >= np.float(0.0)] / e0)

        # Pyqtgraph OpenGL plot
        self.fem.debug_plot_image(phantom_norm_image,
                                  cmap=cmap,
                                  base_opacity=base_opacity,
                                  norm_method=norm_method,
                                  opacity_filter=filter,
                                  opacity_param=opacity_param)

    def btnLandweberClicked(self):
        # Disable start button
        self.subwindow.btnLandweber.setEnabled(False)
        # Load parameters
        iter_num = int(self.subwindow.lineLandweberIterNum.text())
        cmap = self.subwindow.lineLandweberColorMap.text()
        base_opacity = float(self.subwindow.lineLandweberOpacity.text())
        norm_method = self.subwindow.lineLandweberImageNorm.text()
        filter = self.subwindow.lineLandweberImageFilter.text()
        opacity_param = self.subwindow.lineandweberOpacityParam.text()
        # Check previous result
        file_path = './fem_data/inverse_result/' + 'Landweber.npy'
        if os.path.isfile(file_path):
            print('Loading previous inverse result')
            phantom_norm_image = np.load(file_path)
        else:
            print('Solving inverse problem')
            # Load capacitance array & normalization
            C_phantom_norm = (self.C_phantom - self.fem.Cmin) / (self.fem.Cmax - self.fem.Cmin)
            # Calculate by Landweber algorithm
            phantom_norm_image = self.inv_solver.Landweber(self.fem.sensitivity_map, C_phantom_norm, iter_num=iter_num)
            # Save inverse result
            np.save(file_path, phantom_norm_image)

            # (Optional) Quantitative values
            phantom_image = self.inv_solver.Landweber(self.fem.sensitivity_map, self.C_phantom, iter_num=iter_num)
            np.savetxt('./fem_data/inverse_result/Landweber_rel.txt', phantom_image / e0)
            np.savetxt('./fem_data/inverse_result/Landweber_rel_positive_only.txt',
                       phantom_image[phantom_image >= np.float(0.0)] / e0)

        # Enable start button
        self.subwindow.btnLandweber.setEnabled(True)
        # Pyqtgraph OpenGL plot
        self.fem.debug_plot_image(phantom_norm_image,
                                  cmap=cmap,
                                  base_opacity=base_opacity,
                                  norm_method=norm_method,
                                  opacity_filter=filter,
                                  opacity_param=opacity_param)

    def openFileNameDialog(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self, 'Open File', './fem_data/refcap',
                                                  'All Files (*);;Text Files (*.txt)', 'Text Files (*.txt)', options=options)
        return fileName

    def saveFileDialog(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        QFileDialog.setWindowTitle(self, 'Save File')
        fileName, _ = QFileDialog.getSaveFileName(self, 'Save File', './fem_data',
                                                  'All Files (*);;Text Files (*.txt)', 'Text Files (*.txt)', options=options)
        return fileName

