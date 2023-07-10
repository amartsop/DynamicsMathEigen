# Installation

1. Build library inside the root directory
```
mkdir && cd build
cmake ..
```
2. Install library (sudo might be required):
```
cmake --build . --target install
```
Default intallation directory:
1. Installation directory: /usr/local/lib/(dynamics_math_eigen.a)
2. Include directory: /usr/local/include/dynamics_math_eigen/dynamics_math_eigen.h

# Linking
```
find_package(dynamics_math_eigen CONFIG REQUIRED)
target_link_libraries(dme::dynamics_math_eigen)
```
