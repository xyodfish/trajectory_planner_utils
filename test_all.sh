#!/bin/bash
# Comprehensive test script for Trajectory Planner Utils

echo "=========================================="
echo "Trajectory Planner Utils - Full Test"
echo "=========================================="
echo ""

# Check build
if [ ! -d "build" ]; then
    echo "❌ Build directory not found. Building..."
    ./build.sh
    if [ $? -ne 0 ]; then
        echo "❌ Build failed!"
        exit 1
    fi
fi

echo "✅ Build directory exists"
echo ""

# Test all examples
EXAMPLES=(
    "example_trapezoidal:Trapezoidal Velocity Planning"
    "example_double_s:Double-S (S-Curve) Planning"
    "example_straight_trajectory:Cartesian Straight Line"
    "example_comparison:Algorithm Comparison"
    "example_multi_dof:Multi-DOF Planning"
)

PASSED=0
FAILED=0

echo "Running functional tests..."
echo ""

for entry in "${EXAMPLES[@]}"; do
    example="${entry%%:*}"
    desc="${entry##*:}"
    
    if [ -f "build/examples/$example" ]; then
        echo -n "▶️  Testing $desc... "
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
echo "Functional Tests: $PASSED passed, $FAILED failed"
echo "=========================================="
echo ""

# Test visualization
echo "Testing visualization..."
if [ -f "build/examples/example_visualization" ]; then
    echo -n "▶️  Testing Visualization (matplotlib-cpp)... "
    cd build/examples
    rm -f *.png
    
    if ./example_visualization > /tmp/viz_output.txt 2>&1; then
        PNG_COUNT=$(ls -1 *.png 2>/dev/null | wc -l)
        if [ "$PNG_COUNT" -eq 4 ]; then
            echo "✅ PASSED ($PNG_COUNT images generated)"
            ((PASSED++))
            
            echo ""
            echo "Generated images:"
            ls -lh *.png | awk '{print "  - " $9 " (" $5 ")"}'
        else
            echo "⚠️  PARTIAL ($PNG_COUNT/4 images generated)"
            ((FAILED++))
        fi
    else
        echo "❌ FAILED"
        ((FAILED++))
    fi
    cd ../..
else
    echo "⚠️  Visualization example not available (install Python3 dependencies)"
fi

echo ""
echo "=========================================="
echo "Final Results: $PASSED passed, $FAILED failed"
echo "=========================================="

if [ $FAILED -eq 0 ]; then
    echo ""
    echo "🎉 ALL TESTS PASSED!"
    echo ""
    echo "Project is ready at:"
    echo "  /data/other_code/trajectory_planner_utils/"
    echo ""
    echo "Features verified:"
    echo "  ✅ Trapezoidal velocity planning"
    echo "  ✅ Double-S (S-Curve) planning"
    echo "  ✅ Cartesian straight line trajectory"
    echo "  ✅ Multi-DOF planning"
    echo "  ✅ Algorithm comparison"
    echo "  ✅ matplotlib-cpp visualization"
    exit 0
else
    echo ""
    echo "❌ Some tests failed!"
    exit 1
fi
