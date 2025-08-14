# STM32F0 Mandelbrot Set Benchmark

A performance benchmarking project for the STM32F0 microcontroller using Mandelbrot set calculations to evaluate computational capabilities and arithmetic precision.

## Overview

This project implements and benchmarks two different calculation methods for generating the Mandelbrot set on the STM32F0 microcontroller:
- **Fixed-point arithmetic** - Optimized for speed
- **Double-precision floating-point** - Optimized for accuracy

The Mandelbrot set is a collection of complex numbers where the iteration of the formula `f(z) = z² + c` (starting at z = 0) remains bounded and does not diverge to infinity. This mathematical computation provides an excellent benchmark for testing:
- Computational capabilities
- Floating-point arithmetic performance
- Memory management efficiency
- Processing optimization

## Benchmarking Methodology

### Image Dimensions Tested
The implementation is benchmarked across five different image resolutions:
- 128 × 128 pixels
- 160 × 160 pixels
- 192 × 192 pixels
- 224 × 224 pixels
- 256 × 256 pixels

### Metrics Collected
- **Execution times** - Measured using HAL timing functions
- **Checksums** - Validated against reference Python implementation
- **Accuracy comparison** - Between fixed-point and double-precision methods

### Visual Feedback
LEDs are used to provide real-time visual indicators for:
- Calculation start events
- Calculation completion events
- Method identification

## Expected Results

**Fixed-point arithmetic:**
- Faster execution times
- Potential precision loss in complex calculations

**Double-precision arithmetic:**
- Higher mathematical accuracy
- Longer computation times

## Implementation Details

The project consists of C code implementations for both calculation methods, specifically optimized for the STM32F0 microcontroller architecture. Results are validated against the provided reference implementation (`Mandelbrot.py`) to ensure computational accuracy.

## Getting Started

### Prerequisites
- STM32F0 development board
- STM32CubeIDE or compatible development environment
- HAL library for STM32F0
- Reference Python script (`Mandelbrot.py`) for validation

### Hardware Requirements
- STM32F0 microcontroller
- LEDs for visual feedback
- Debug interface for timing measurements

## Performance Analysis

This benchmark demonstrates the fundamental trade-off between computational speed and numerical precision in embedded systems, providing insights into optimal arithmetic selection for resource-constrained microcontroller applications.

---

*This project serves as a practical exploration of computational performance characteristics and arithmetic precision trade-offs in embedded microcontroller systems.*
