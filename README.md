# ECT System Development Project #

Custom Electrical Capacitance Tomography System with Hardware.
8 electrodes system.

Two main parts:
1. Embedded (C/C++): Multi-module PCB hardware with STM32H743ZI.
2. Emulator (Python): Computer UI for hardware control by USB-C.

---

# Embedded code #

Currently (Mar 6, 2019), STM32H7 MCU Package on STMCubeMX does not support DSP library. Manual import of DSP source files is essential.

1. Download CMSIS Pack from ARM (https://github.com/ARM-software/CMSIS_5).
2. Create "/DSP_Lib" & "/Lib" folders into "(Project root)/Drivers/CMSIS/".
3. Duplicate all files in ".../CMSIS/DSP/Source/" into "(Project root)/Drivers/CMSIS/DSP_Lib/".
4. Duplicate all files in ".../CMSIS/DSP/Include/" into "(Project root)/Drivers/CMSIS/Include/".
5. Duplicate ".../CMSIS/Lib/ARM" & ".../CMSIS/Lib/GCC" folders into "(Project root)/Drivers/CMSIS/Lib/".
6. From top menu in Eclipse IDE, go to "Project - Properties - C/C++ General - Paths and Symbols - Symbols" tab.
Add  "__FPU_PRESENT" and "ARM_MATH_CM7" to Symbol. Allocate "__FPU_PRESENT = 1". No allocation is needed to ARM_MATH_CM7.
7. From top menu in Eclipse IDE, go to "Project - Properties - C/C++ General - Paths and Symbols - Library Paths" tab.
Add ".../Drivers/CMSIS/Lib/".

---

# Emulator code #

## Pre-requisite modules ##

1. Pyqt5
https://pypi.org/project/PyQt5/
=> pip install pyqt5

2. Pyqtgraph
http://www.pyqtgraph.org/
=> pip install pyqtgraph

3. Numpy
http://www.numpy.org/
=> pip install numpy

4. pyusb
https://github.com/pyusb/pyusb
=> pip install pyusb

5. FEniCS
https://fenicsproject.org/
=> conda install -c conda-forge fenics

(Alternatively)
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:fenics-packages/fenics
sudo apt-get update
sudo apt-get install --no-install-recommends fenics

6. PyOpenGL
http://pyopengl.sourceforge.net/
=> pip install pyopengl

7. Matplotlib
=> conda install -c conda-forge matplotlib

8. Cython
=> pip install Cython

9. Scipy
=> pip install scipy

* Refer the offical websites for installation

---

# Caution #

Ubuntu's udev rule must be modified for usb connection by pyusb

1. Create rule file
=> sudo gedit /etc/udev/rules.d/99-ectusb.rules
2. Fill the contents:
=> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="666"
3. Save and restart udev
=> sudo udevadm trigger

