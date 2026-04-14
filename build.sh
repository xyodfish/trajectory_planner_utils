#!/bin/bash
# Build script for velocity_planning library

set -e

echo "========================================="
echo "Building Velocity Planning Library"
echo "========================================="

# Create build directory
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning old build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo ""
echo "Configuring CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_UNIFIED_TRAJECTORY_MODULES=ON \
    -DBUILD_UNIFIED_CL_MODULES=ON \
    -DBUILD_UNIFIED_SAS_MODULES=ON

# Build
echo ""
echo "Building..."
make -j$(nproc)

echo ""
echo "========================================="
echo "Build completed successfully!"
echo "========================================="
echo ""
echo "To run examples:"
echo "  cd build/examples"
echo "  ./example_trapezoidal"
echo "  ./example_multi_dof"
echo ""
echo "To install:"
echo "  sudo make install"
echo ""
