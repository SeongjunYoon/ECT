# ECT System Development Project
Custom Electrical Capacitance Tomography System with Hardware
8 Electrodes

Two main parts
(1) Embedded (C/C++): Multi-module PCB hardware with STM32H743ZI
(2) Emulator (Python): Computer UI for hardware control by USB-C

---
# Embedded code


---
# Emulator code
Pre-requisite modules

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

*** Caution ***
Ubuntu's udev rule must be modified for usb connection by pyusb

1. Create rule file
=> sudo gedit /etc/udev/rules.d/99-ectusb.rules
2. Fill the contents:
=> SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="666"
3. Save and restart udev
=> sudo udevadm trigger

