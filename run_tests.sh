#!/bin/bash
set -e

echo "Building and running Mock Tests..."

# Create build directory
mkdir -p build

# Compile mock environment and tests
g++ -I mock_arduino/include -I . mock_arduino/src/mock_arduino.cpp test_mock.cpp -o build/test_mock
g++ -I mock_arduino/include -I . mock_arduino/src/mock_arduino.cpp mock_arduino/src/test_basic_filter.cpp -o build/test_basic_filter
g++ -I mock_arduino/include -I . mock_arduino/src/mock_arduino.cpp mock_arduino/src/test_dma_filter.cpp -o build/test_dma_filter

echo "Running Mock Environment test:"
./build/test_mock

echo "Running BasicFilter test:"
./build/test_basic_filter

echo "Running DmaFilter test:"
./build/test_dma_filter

echo "All tests passed!"
