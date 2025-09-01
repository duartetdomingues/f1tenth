# Installation

## Linux/Mac

### Prerequisites
We assume you have: git, make, cmake installed on your system.

### Clone acados
Clone acados and its submodules by running:
```
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

### Build and install `acados`
A CMake and a Makefile based build system is available in acados.
Note that only the `CMake` build system is tested using CI and is thus recommended.
Please choose one and proceed with the corresponding paragraph.

#### **CMake** (recommended)
Install `acados` as follows:
```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_DAQP=ON, a list of CMake options is provided below
make install -j4
```
My cmd: 
```
cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OPENMP=ON -DACADOS_MATLAB=ON -DACADOS_INSTALL_DIR="/home/desktop/acados"
```

#### CMake options:
Below is a list of CMake options available for configuring the `acados` build.
These options can be passed to the `cmake` command using the `-D` flag, e.g., `cmake -DOPTION_NAME=VALUE ..`.
Adjust these options based on your requirements.

| **Option Name**                | **Description**                                            | **Default Value** |
|--------------------------------|------------------------------------------------------------|-------------------|
| `ACADOS_WITH_QPOASES`          | Compile acados with optional QP solver qpOASES             | `OFF`             |
| `ACADOS_WITH_DAQP`             | Compile acados with optional QP solver DAQP                | `OFF`             |
| `ACADOS_WITH_QPDUNES`          | Compile acados with optional QP solver qpDUNES             | `OFF`             |
| `ACADOS_WITH_OSQP`             | Compile acados with optional QP solver OSQP                | `OFF`             |
| `ACADOS_WITH_HPMPC`            | Compile acados with optional QP solver HPMPC               | `OFF`             |
| `ACADOS_WITH_QORE`             | Compile acados with optional QP solver QORE (experimental) | `OFF`             |
| `ACADOS_WITH_OOQP`             | Compile acados with optional QP solver OOQP (experimental) | `OFF`             |
| `BLASFEO_TARGET`               | BLASFEO Target architecture, see BLASFEO repository for more information. Possible values include: `X64_AUTOMATIC`, `GENERIC`, `X64_INTEL_SKYLAKE_X`, `X64_INTEL_HASWELL`, `X64_INTEL_SANDY_BRIDGE`, `X64_INTEL_CORE`, `X64_AMD_BULLDOZER`, `ARMV8A_APPLE_M1`, `ARMV8A_ARM_CORTEX_A76`, `ARMV8A_ARM_CORTEX_A73`, `ARMV8A_ARM_CORTEX_A57`, `ARMV8A_ARM_CORTEX_A55`, `ARMV8A_ARM_CORTEX_A53`, `ARMV7A_ARM_CORTEX_A15`, `ARMV7A_ARM_CORTEX_A9`, `ARMV7A_ARM_CORTEX_A7` | `X64_AUTOMATIC`   |
| `LA`                           | Linear algebra optimization level for BLASFEO  | `HIGH_PERFORMANCE`|
| `ACADOS_WITH_SYSTEM_BLASFEO`   | Use BLASFEO found via `find_package(blasfeo)` instead of compiling it | `OFF`             |
| `HPIPM_TARGET`                 | HPIPM Target architecture. Possible values: `AVX`, `GENERIC` | `GENERIC` |
| `ACADOS_WITH_OPENMP`           | OpenMP parallelization                                        | `OFF`             |
| `ACADOS_NUM_THREADS`           | Number of threads for OpenMP parallelization within one NLP solver. If not set, `omp_get_max_threads` will be used to determine the number of threads. If multiple solves should be parallelized, e.g. with an `AcadosOcpBatchSolver` or `AcadosSimBatchSolver`, set this to 1. | Not set |
| `ACADOS_SILENT`                | No console status output                                      | `OFF`             |
| `ACADOS_DEBUG_SQP_PRINT_QPS_TO_FILE` | Print QP inputs and outputs to file in SQP                    | `OFF`             |
| `ACADOS_DEVELOPER_DEBUG_CHECKS` | Enable developer debug checks                 | `OFF`             |
| `CMAKE_BUILD_TYPE`             | Build type (e.g., Release, Debug, etc.)                              | `Release`         |
| `ACADOS_UNIT_TESTS`            | Compile unit tests                                            | `OFF`             |
| `ACADOS_EXAMPLES`              | Compile C examples                                              | `OFF`             |
| `ACADOS_OCTAVE`                | Octave interface CMake tests                         | `OFF`             |
| `ACADOS_PYTHON`                | Python interface CMake tests (Note: Python interface installation is independent of this)    | `OFF`             |
| `BUILD_SHARED_LIBS`            | Build shared libraries             | `ON` (non-Windows)|
<!-- Deprecated, remove everywhere? -->
<!-- | `ACADOS_LINT`                  | Compile Lint                                                  | `OFF`             | -->

For more details on specific options, refer to the comments in the `CMakeLists.txt` file.
