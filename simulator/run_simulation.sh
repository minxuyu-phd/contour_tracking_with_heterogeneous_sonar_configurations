#!/bin/bash

# AUV Simulation Launcher Script

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo "=== AUV Simulation Launcher ==="
echo ""

# Check if built
if [ ! -f "build/simulator" ]; then
    echo -e "${RED}Error: Simulation binary not found${NC}"
    echo "Please build the project first:"
    echo "  mkdir build && cd build && cmake .. && make"
    exit 1
fi

# Check config
CONFIG_DIR="config/"
if [ ! -f "$CONFIG_DIR/config.jsonc" ]; then
    echo -e "${RED}Error: Missing config file: ${CONFIG_DIR}config.jsonc${NC}"
    exit 1
fi

echo -e "${GREEN}Configuration OK${NC}"
echo ""

# Launch simulation
echo "Starting simulation..."
echo "Press Ctrl+C to stop"
echo ""

./build/simulator "$CONFIG_DIR"
