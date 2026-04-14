#!/bin/bash
# Quick verification script for Trajectory Planner Utils

echo "=========================================="
echo "Trajectory Planner Utils - Quick Verify"
echo "=========================================="
echo ""

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "❌ Build directory not found. Running build..."
    ./build.sh
    if [ $? -ne 0 ]; then
        echo "❌ Build failed!"
        exit 1
    fi
fi

echo "✅ Build directory exists"
echo ""

# Check library
if [ -f "build/libvelocity_planning.so" ]; then
    LIB_SIZE=$(ls -lh build/libvelocity_planning.so | awk '{print $5}')
    echo "✅ Library exists: $LIB_SIZE"
else
    echo "❌ Library not found!"
    exit 1
fi

echo ""
echo "Running examples..."
echo ""

# Run each example
EXAMPLES=("example_trapezoidal" "example_double_s" "example_straight_trajectory" "example_comparison" "example_multi_dof")
PASSED=0
FAILED=0

for example in "${EXAMPLES[@]}"; do
    if [ -f "build/examples/$example" ]; then
        echo -n "▶️  Testing $example... "
        if ./build/examples/$example > /dev/null 2>&1; then
            echo "✅ PASSED"
            ((PASSED++))
        else
            echo "❌ FAILED"
            ((FAILED++))
        fi
    else
        echo "❌ $example not found!"
        ((FAILED++))
    fi
done

echo ""
echo "=========================================="
echo "Test Results: $PASSED passed, $FAILED failed"
echo "=========================================="

if [ $FAILED -eq 0 ]; then
    echo "✅ ALL TESTS PASSED!"
    echo ""
    echo "Project is ready at:"
    echo "  /data/other_code/trajectory_planner_utils/"
    exit 0
else
    echo "❌ Some tests failed!"
    exit 1
fi
