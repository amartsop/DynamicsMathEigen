Installation
--------------
Run inside the build directory: cmake --build . --target install
-> Installation directory: /usr/local/lib/(dynamics_math_eigen.a)
-> Include directory: /usr/local/include/dynamics_math_eigen/dynamics_math_eigen.h

Linking
--------------
1. find_package(dynamics_math_eigen CONFIG REQUIRED)
2. Inlude lib dme::dynamics_math_eigen)