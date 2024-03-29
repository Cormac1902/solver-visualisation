# Solver Visualisation Program
## Description
This is a C++ program to display a SAT formula, as well as visualise the solving process of a particular solver if desired.

## Dependencies
Requires:
- [VTK](https://vtk.org/download/)
- [CryptoMiniSAT](https://github.com/msoos/cryptominisat)
- ZeroMQ (```sudo apt-get install libzmq-dev```)

Optional:
- [Modified version of MapleLCMDistChronoBT](https://github.com/Cormac1902/solver-visualisation-MapleLCMDistChronoBT)
- [Modified version of MiniSat](https://github.com/Cormac1902/solver-visualisation-minisat)

## Installation
1. Copy SVisZMQ to include directory:
    ```shell
   sudo cp lib/s_vis_zmq.hpp usr/include
   ```

1. Install MapleLCMDistChronoBT and/or MiniSAT if desired
1. Build with cmake:
    ```shell
    cmake --build cmake-build-debug --target all -- -j 6
    ```
1. Run using s_vis executable in cmake-build-debug folder

## Usage
- Visualise a SAT instance with:
    ```shell
    s_vis <sat-instance>
    ```
  Please note the CNF file must have CRLF line endings; not LF
- Change graph level using the numbers on the keyboard
- Visualise a solver run using the following keys:
    - B: Basic WalkSAT algorithm (will run indefinitely)
    - C: CryptoMiniSAT (experimental)
    - G: MapleLCMDistChronoBT
    - M: MiniSAT
    - W: WalkSAT 

## Roadmap
- Fix CryptoMiniSAT
- Include solvers as part of program (as opposed to being run with system commands)
- Include more solvers 
- Create UI

## Support
CormacGeraghty@outlook.ie

## Author
Cormac Geraghty
