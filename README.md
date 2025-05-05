# lp5

pacman -Su         # Finish the update if needed
pacman -S mingw-w64-x86_64-gcc   # Install MinGW-w64 64-bit toolchain


%%writefile main.cpp

%%script bash
g++ main.cpp -std=c++11
./a.out

