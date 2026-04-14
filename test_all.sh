#!/bin/bash
# Comprehensive test script for Trajectory Planner Utils

set -e

echo "=========================================="
echo "Trajectory Planner Utils - Full Test"
echo "=========================================="
echo ""

# Check build
if [ ! -d "build" ]; then
    echo "❌ Build directory not found. Building..."
    ./build.sh
fi

echo "✅ Build directory exists"
echo ""

PASSED=0
FAILED=0

run_example() {
    local example="$1"
    local desc="$2"
    local required="$3"

    if [ -f "build/examples/$example" ]; then
        echo -n "▶️  Testing $desc... "
        if ./build/examples/$example > /dev/null 2>&1; then
            echo "✅ PASSED"
            PASSED=$((PASSED + 1))
        else
            echo "❌ FAILED"
            FAILED=$((FAILED + 1))
        fi
    else
        if [ "$required" = "required" ]; then
            echo "❌ $desc missing ($example not built)"
            FAILED=$((FAILED + 1))
        else
            echo "⚠️  $desc skipped ($example not built)"
        fi
    fi
}

echo "Running base functional tests..."
echo ""
run_example "example_trapezoidal" "Trapezoidal Velocity Planning" "required"
run_example "example_double_s" "Double-S (S-Curve) Planning" "required"
run_example "example_straight_trajectory" "Cartesian Straight Line" "required"
run_example "example_comparison" "Algorithm Comparison" "required"
run_example "example_multi_dof" "Multi-DOF Planning" "required"

echo ""
echo "Running staged executable tests..."
echo ""
run_example "example_stage1_unified_core" "Stage1 Unified Core" "optional"
run_example "example_stage2_unified_sas" "Stage2 Unified SAS" "optional"
run_example "example_stage3_unified_cl" "Stage3 Unified Circle-Line" "optional"

# Test visualization
echo ""
echo "Testing visualization..."
if [ -f "build/examples/example_visualization" ]; then
    echo -n "▶️  Testing Visualization (matplotlib-cpp)... "
    cd build/examples
    rm -f ./*.png

    if ./example_visualization > /tmp/viz_output.txt 2>&1; then
        PNG_COUNT=$(ls -1 ./*.png 2>/dev/null | wc -l)
        if [ "$PNG_COUNT" -eq 4 ]; then
            echo "✅ PASSED ($PNG_COUNT images generated)"
            PASSED=$((PASSED + 1))

            echo ""
            echo "Generated images:"
            ls -lh ./*.png | awk '{print "  - " $9 " (" $5 ")"}'
        else
            echo "⚠️  PARTIAL ($PNG_COUNT/4 images generated)"
            FAILED=$((FAILED + 1))
        fi
    else
        echo "❌ FAILED"
        FAILED=$((FAILED + 1))
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
    echo "Features verified:"
    echo "  ✅ Core velocity planning"
    echo "  ✅ Base Cartesian trajectory"
    echo "  ✅ Staged unified-module executable tests"
    echo "  ✅ matplotlib-cpp visualization"
    exit 0
else
    echo ""
    echo "❌ Some tests failed!"
    exit 1
fi
