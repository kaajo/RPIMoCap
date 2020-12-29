# RPIMoCap
This project attempts to build an optical motion capture system with passive markers under 1000 EUR. For more information about the architecture check [WiKi](https://github.com/kaajo/RPIMoCap/wiki). 

#### Dependencies
* CMake
* Qt 5.14
* OpenCV 4
* Eigen
* Ceres Solver
* Avahi
 
#### Current state 
Implemented features:
* Simulation and visualization of the 3D scene
* Automatic discovery of cameras on the local network
* Multiple camera Wand calibration
* Basic triangulation

Planned for future releases:
* Save/Load project in simulation and main application
* Skeleton representation
* Advanced triangulation and motion tracking
* Export to relevant formats
