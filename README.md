# Demo — AUV Simulation Quick Start

## Prerequisites

- Ubuntu Linux (tested on 22.04)
- OpenGL 4.3 capable GPU with official drivers
- CMake >= 3.10, GCC with C++17 support
- Python 3 with pip
- conda or venv environment (recommended)

## Step 1: Install nng_comm

The `nng_comm` module provides JSON-based communication (Pub/Sub and Req/Rep) over NNG.

### 1.1 Install nng C library (v2.0.0-alpha.7)

```bash
sudo apt-get update && sudo apt-get install -y cmake build-essential
git clone --branch v2.0.0-alpha.7 https://github.com/nanomsg/nng.git /tmp/nng
cd /tmp/nng && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc) && sudo make install && sudo ldconfig
```

### 1.2 Install nlohmann/json (v3.11.3)

```bash
git clone --branch v3.11.3 https://github.com/nlohmann/json.git /tmp/nlohmann_json
cd /tmp/nlohmann_json && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DJSON_BuildTests=OFF ..
make -j$(nproc) && sudo make install
```

### 1.3 Install pynng (Python)

```bash
pip install pynng==0.9.0
```

### 1.4 Build nng_comm

```bash
cd nng_comm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

See [nng_comm/README.md](nng_comm/README.md) for full API reference and testing instructions.

## Step 2: Install Stonefish

This is a customized version of [Stonefish](https://stonefish.readthedocs.io) with added SBES (Single Beam Echo Sounder) sensor support. It differs from the upstream release.

### 2.1 Install dependencies

```bash
sudo apt-get install -y libglm-dev libsdl2-dev libfreetype6-dev
```

### 2.2 Build and install

```bash
cd stonefish
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Step 3: Build the Simulator

```bash
cd simulator
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## Step 4: Run the Simulation

```bash
cd contour_tracker
bash start_all.sh
```

This launches the full simulation pipeline including the simulator, OGM, and contour tracker.
