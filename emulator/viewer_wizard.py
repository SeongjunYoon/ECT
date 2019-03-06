from inverse_solver import inverse_solver
import image_filter
from PyQt5 import QtCore
import pyqtgraph.opengl as gl
import numpy as np

C_measured_size = 28

class Viewer_Wizard():
    def __init__(self, viewer, ect_controller, fem):
        # Get instances
        self.viewer = viewer
        self.ect_controller = ect_controller
        self.fem = fem
        self.inv_solver = inverse_solver.InverseSolver()
        self.image_filter = image_filter.ImageFilter()
        # Set signal connection
        self.setSignalConnect()
        # Initialize inverse algorithm parameters
        self.initInverseAlgoParam()
        # Load Cmin & Cmax for normalization
        self.Cmin = np.loadtxt(self.fem.measured_dir + '/Online_C_min.txt')
        self.Cmin = self.Cmin.reshape((self.Cmin.size, 1))
        self.Cmax = np.loadtxt(self.fem.measured_dir + '/Online_C_max.txt')
        self.Cmax = self.Cmax.reshape((self.Cmax.size, 1))
        #
        self.inverseAlgoFlag = 'LBP'
        # Connect openGL view to GUI
        self.setOpenGL()
        self.plotInit()
        # Create graph update timer
        self.setPlotUpdateTimer()
        # Start measuring
        self.start_plot_update()

    def setSignalConnect(self):
        self.viewer.btnLBP.clicked.connect(self.btnLBPClicked)
        self.viewer.btnLandweber.clicked.connect(self.btnLandweberClicked)

    def default_config(self):
        self.btnLBPClicked()

    def setOpenGL(self):
        # Dimension scale factor
        self.unit_scale = 1  # Mesh base scale => 1 : m, 1000: mm
        # Crate openGL instance
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=0.05 * self.unit_scale)
        # Add Grid Item
        gl_grid = gl.GLGridItem()
        gl_grid.scale(0.01 * self.unit_scale, 0.01 * self.unit_scale, 0.01 * self.unit_scale)
        self.gl_widget.addItem(gl_grid)
        # Connect openGL instance to GUI layout
        self.viewer.openglLayout.addWidget(self.gl_widget)

    def initInverseAlgoParam(self):
        # Tab LBP
        self.viewer.LBPColorMapLineEdit.setText('seismic')
        self.viewer.LBPNormMethodLineEdit.setText('linear')
        self.viewer.LBPGammaLineEdit.setText('0.0')
        self.viewer.LBPBaseOpacityLineEdit.setText('0.1')
        self.viewer.LBPOpacityFilterLineEdit.setText('threshold')
        self.viewer.LBPOpacityParamLineEdit.setText('0.0001')
        # Tab Landweber
        self.viewer.LandweberIterationNumberLineEdit.setText('5')
        self.viewer.LandweberColorMapLineEdit.setText('seismic')
        self.viewer.LandweberNormMethodLineEdit.setText('linear')
        self.viewer.LandweberGammaLineEdit.setText('0.0')
        self.viewer.LandweberBaseOpacityLineEdit.setText('0.1')
        self.viewer.LandweberOpacityFilterLineEdit.setText('threshold')
        self.viewer.LandweberOpacityParamLineEdit.setText('0.0001')

    def btnLBPClicked(self):
        # Load params
        self.cmap = self.viewer.LBPColorMapLineEdit.text()
        self.base_opacity = float(self.viewer.LBPBaseOpacityLineEdit.text())
        self.norm_method = self.viewer.LBPNormMethodLineEdit.text()
        self.gamma = self.viewer.LBPGammaLineEdit.text()
        self.opacity_filter = self.viewer.LBPOpacityFilterLineEdit.text()
        self.opacity_param = self.viewer.LBPOpacityParamLineEdit.text()
        self.inverseAlgoFlag = 'LBP'
        #
        if self.norm_method == 'relative':
            self.image_filter.relative_init = True

    def btnLandweberClicked(self):
        # Load params
        self.iter_num = int(self.viewer.LandweberIterationNumberLineEdit.text())
        self.cmap = self.viewer.LandweberColorMapLineEdit.text()
        self.base_opacity = float(self.viewer.LandweberBaseOpacityLineEdit.text())
        self.norm_method = self.viewer.LandweberNormMethodLineEdit.text()
        self.gamma = self.viewer.LandweberGammaLineEdit.text()
        self.opacity_filter = self.viewer.LandweberOpacityFilterLineEdit.text()
        self.opacity_param = self.viewer.LandweberOpacityParamLineEdit.text()
        self.inverseAlgoFlag = 'Landweber'
        #
        if self.norm_method == 'relative':
            self.image_filter.relative_init = True

    def setPlotUpdateTimer(self):
        # Create graph update timer
        self.timerDataUpdate = QtCore.QTimer()
        self.timerDataUpdate.timeout.connect(self.plotUpdate)

    def plotInit(self):
        #
        self.getLimitData()
        # Get ECT cap array data
        self.getData()
        # Load default image filter configuraiton
        self.default_config()
        # Set color normalization method
        (self.values_norm, self.color_values_norm) = self.image_filter.color_normalizer(value=self.epsilon_sensing_domain,
                                                                    cmap=self.cmap,
                                                                    base_opacity=self.base_opacity,
                                                                    norm_method=self.norm_method,
                                                                    gamma=self.gamma,
                                                                    initial=True,
                                                                    coordinates=self.fem.sensing_domain_coordinates)
        # Set opacity filter
        self.image_filter.opacity_filter(opacity_filter=self.opacity_filter,
                                         opacity_param=self.opacity_param,
                                         values=self.epsilon_sensing_domain,
                                         values_norm=self.values_norm,
                                         color_values_norm=self.color_values_norm,
                                         coordinates=self.fem.sensing_domain_coordinates)
        # Create openGL scatter plot and add
        self.item_dof = gl.GLScatterPlotItem(pos=self.fem.sensing_domain_coordinates,
                                             color=self.color_values_norm,
                                             size=0.001 * self.unit_scale,
                                             pxMode=False)
        self.item_dof.setGLOptions('additive')
        self.gl_widget.addItem(self.item_dof)
        # Add tracker
        self.trackerInit()

    def plotUpdate(self):
        # Get ECT cap array data
        self.getData()
        # Update info window
        self.updateInfoWindow()
        # Update color values
        (self.values_norm, self.color_values_norm) = self.image_filter.color_normalizer(value=self.epsilon_sensing_domain,
                                                                    cmap=self.cmap,
                                                                    base_opacity=self.base_opacity,
                                                                    norm_method=self.norm_method,
                                                                    gamma=self.gamma,
                                                                    initial=False,
                                                                    coordinates=self.fem.sensing_domain_coordinates)
        self.image_filter.opacity_filter(opacity_filter=self.opacity_filter,
                                         opacity_param=self.opacity_param,
                                         values= self.epsilon_sensing_domain,
                                         values_norm=self.values_norm,
                                         color_values_norm=self.color_values_norm,
                                         coordinates=self.fem.sensing_domain_coordinates)
        # Update openGL scatter plot
        self.item_dof.setData(pos=self.fem.sensing_domain_coordinates,
                              color=self.color_values_norm,
                              size=0.001 * self.unit_scale,
                              pxMode=False)
        # Update tracker
        self.trackerUpdate()

    def updateInfoWindow(self):
        self.max_epsilon = self.epsilon_sensing_domain.max()
        self.max_epsilon_position = self.fem.sensing_domain_coordinates[self.epsilon_sensing_domain.argmax()]
        self.viewer.infoBrowser.setText('-------------------------------------------------------')
        self.viewer.infoBrowser.append('-------------------------------------------------------')
        self.viewer.infoBrowser.append('Max Epsilon = {:.5}'.format(self.max_epsilon))
        self.viewer.infoBrowser.append('-------------------------------------------------------')
        self.viewer.infoBrowser.append('[Position of Max Epsilon]')
        self.viewer.infoBrowser.append('x = {:7.4} mm'.format(self.max_epsilon_position[0] * 1000))
        self.viewer.infoBrowser.append('y = {:7.4} mm'.format(self.max_epsilon_position[1] * 1000))
        self.viewer.infoBrowser.append('z = {:7.4} mm'.format(self.max_epsilon_position[2] * 1000))
        self.viewer.infoBrowser.append('-------------------------------------------------------')
        self.viewer.infoBrowser.append('-------------------------------------------------------')

    def getData(self):
        # Get C_norm
        self.C_measured = self.ect_controller.dsp.dspHalfArray
        self.C_measured = self.C_measured.reshape((C_measured_size, 1))
        self.C_norm = (self.C_measured - self.Cmin) / (self.Cmax - self.Cmin)
        # Solve inverse problem
        if self.inverseAlgoFlag == 'LBP':
            self.epsilon = self.inv_solver.LBP(self.fem.sensitivity_map, self.C_norm)
        elif self.inverseAlgoFlag == 'Landweber':
            self.epsilon = self.inv_solver.Landweber(self.fem.sensitivity_map, self.C_norm, self.iter_num)
        else:
            pass
        # Filter sensing domain data only
        self.epsilon_sensing_domain = self.epsilon[self.fem.air_cell_indices]

    def getLimitData(self):
        self.C_norm_min = (self.Cmin - self.Cmin) / (self.Cmax - self.Cmin)
        self.C_norm_max = (self.Cmax - self.Cmin) / (self.Cmax - self.Cmin)
        self.epsilon_min = self.inv_solver.LBP(self.fem.sensitivity_map, self.C_norm_min)
        self.epsilon_max = self.inv_solver.LBP(self.fem.sensitivity_map, self.C_norm_max)
        self.epsilon_min_sensing_domain = self.epsilon_min[self.fem.air_cell_indices]
        self.epsilon_max_sensing_domain = self.epsilon_max[self.fem.air_cell_indices]
        self.image_filter.set_norm_limit_value(min=self.epsilon_min_sensing_domain.min(),
                                               max=self.epsilon_max_sensing_domain.max())

    def trackerInit(self):
        # Set tracker point position, color, size
        tracker_init_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64).reshape((1,3))
        self.tracker_color = [0.0, 0.0, 1.0, 1.0]       # RGBA
        self.tracker_size = 0.005 * self.unit_scale
        # Create openGL scatter plot for tracker
        self.item_max_tracker = gl.GLScatterPlotItem(pos=tracker_init_pos,
                                                     color=self.tracker_color,
                                                     size=self.tracker_size,
                                                     pxMode=False)
        self.item_max_tracker.setGLOptions('additive')
        self.gl_widget.addItem(self.item_max_tracker)

    def trackerUpdate(self):
        self.item_max_tracker.setData(pos=self.max_epsilon_position.reshape((1,3)),
                                      color=self.tracker_color,
                                      size=self.tracker_size,
                                      pxMode=False)

    def start_plot_update(self):
        # Set image frame per sec (fps)
        fps = 30
        self.ect_controller.measureStart()
        self.timerDataUpdate.start(int(1000 / fps))

    def stop_plot_update(self):
        self.timerDataUpdate.stop()
        self.ect_controller.measureStop()

