import pyqtgraph as pg
from PyQt5 import QtGui


class GraphWizard:
    def __init__(self):
        # Create font instance
        self.setFont()
        # Create widget instance
        pg.setConfigOption('background', 'w')
        self.plot = pg.PlotWidget()
        # Show auto-scale button on the bottom-left corner
        self.plot.showButtons()
        # Theme setting
        self.setTheme()
        # Create Plot
        self.predefineRange(enable=False)
        self.curve = self.plot.plot(clear=True, symbol=None, pen=pg.mkPen('k', width=3), symbolPen='k', symbolSize=8, symbolBrush='k')

    def predefineRange(self, enable):
        if enable:
            self.plot.enableAutoRange()
        else:
            self.plot.enableAutoRange()

    def setFont(self):
        self.font = QtGui.QFont()
        self.font.setFamily('Arial')
        self.font.setPixelSize(14)
        self.font.setBold(True)

    def setTheme(self):
        self.plotAxisBottom = self.plot.getAxis('bottom')
        self.plotAxisBottom.setPen('k', width=3)
        self.plotAxisBottom.tickFont = self.font
        self.plotAxisBottom.setStyle(tickLength=-5, tickTextOffset=10, autoExpandTextSpace=True)
        self.plotAxisBottom.enableAutoSIPrefix(enable=False)
        #
        self.plotAxisLeft = self.plot.getAxis('left')
        self.plotAxisLeft.setPen('k', width=3)
        self.plotAxisLeft.tickFont = self.font
        self.plotAxisLeft.setStyle(tickLength=-5, tickTextOffset=10, autoExpandTextSpace=True)
        self.plotAxisLeft.setWidth(80)
        self.plotAxisBottom.enableAutoSIPrefix(enable=True)
        #
        self.plotAxisRight = self.plot.getAxis('right')
        #
        self.plotAxisTop = self.plot.getAxis('top')






