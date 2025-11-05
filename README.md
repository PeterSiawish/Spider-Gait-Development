# Spider Gait Development

This project implements a spider robot gait optimization system using genetic algorithms. The project includes both MATLAB and Python implementations for spider leg kinematics and visualization.

## Project Overview

The project aims to develop optimal gaits for an 8-legged spider robot using genetic algorithms. The spider model features:

- 8 legs with 3 joints each (24 total joints)
- Forward kinematics implementation
- 3D visualization capabilities
- Genetic algorithm optimization for gait generation

### Key Components

1. **Spider Model**

   - Each leg has 3 segments: Coxa, Femur, and Tibia
   - Segments lengths: [1.2, 0.7, 1.0] units respectively
   - Body shape: Elliptical with axes a=1.5, b=1.0

2. **Kinematics**

   - Forward leg kinematics implementation
   - Joint angle calculations
   - 3D pose visualization

3. **Gait Optimization**
   - Genetic Algorithm implementation
   - Population size: 1000 chromosomes
   - Each chromosome represents a full gait (300 poses × 24 joints)
   - Parametric solution approach (proposed)

## Dependencies

### Python

- NumPy
- Matplotlib
- Math (standard library)
- Random (standard library)
- Time (standard library)

### MATLAB

- Standard MATLAB installation (no additional toolboxes required)

## Setup and Installation

1. Install Python dependencies:

```bash
pip install numpy matplotlib
```

2. For MATLAB implementation:

- Ensure MATLAB is installed
- Add the MatLab Files directory to MATLAB path

## Usage

### Python Implementation

1. Run the main genetic algorithm:

```bash
python Code/main.py
```

## Implementation Details

### Genetic Algorithm Approach

The project implements two approaches for gait optimization:

1. **Direct Angle Representation**

   - Chromosome: 300 poses × 24 joints = 7200 floating-point numbers
   - Each number represents a joint angle in radians [-π, π]
   - Population size: 1000 chromosomes

2. **Parametric Solution** (Proposed)
   - Uses sinusoidal functions for smooth transitions
   - angle(t) = A _ sin(2 _ π _ B _ t + C) + D
   - Parameters per joint: A (amplitude), B (frequency), C (phase), D (offset)
   - Reduces chromosome size and ensures smoother gaits

### Joint Configuration

- Leg arrangement: ['L1', 'L2', 'L3', 'L4', 'R4', 'R3', 'R2', 'R1']
- Base angles:
  - Left legs: [45°, 75°, 105°, 135°]
  - Right legs: [-135°, -105°, -75°, -45°]
- Fixed coxa elevation: 30°

## Future Work

1. Implementation of fitness function for gait evaluation
2. Crossover and mutation operators for the genetic algorithm
3. Gait optimization criteria:
   - Stability
   - Energy efficiency
   - Movement smoothness
4. Implementation of the parametric solution approach

## License

This project is licensed under the standard MIT License

## Contributors

Peter Siawishh - petersrood@gmail.com
