#!/usr/bin/env bash
set -euo pipefail

# ── Configuration ──────────────────────────────────────────────────────────
export PYTHONUNBUFFERED=1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Color codes for prefixes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

COLORS=("$RED" "$GREEN" "$YELLOW" "$BLUE" "$MAGENTA" "$CYAN" "$WHITE")

# Services: "name|working_dir|command"
SERVICES=(
  "Simulator|/home/bsa/shared/nngtest/simulator|./run_simulation.sh"
  "OGM|/home/bsa/shared/nngtest/ogm|python3 main.py"
  "Adaptive Scanner|$SCRIPT_DIR|python3 msis_adaptive_scanner.py"
  "PID Controller|$SCRIPT_DIR|python3 depth_cruise_controller.py"
  "LOS Guidance|$SCRIPT_DIR|python3 los_guidance.py"
  "Contour Fitting|$SCRIPT_DIR|python3 contour_fitting.py"
  "Data Recorder|$SCRIPT_DIR|python3 data_recorder.py"
)

PIDS=()
NAMES=()

# ── Parse arguments ───────────────────────────────────────────────────────
CYCLE_COUNT=0
CYCLE_TIMEOUT=0

usage() {
  echo "Usage: $0 [--cycle N --timeout S]"
  echo ""
  echo "Options:"
  echo "  --cycle N    Run all services N times (requires --timeout)"
  echo "  --timeout S  Wait S seconds before stopping and restarting (requires --cycle)"
  echo ""
  echo "Without arguments, services run until Ctrl+C."
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cycle)
      [[ -z "${2:-}" ]] && usage
      CYCLE_COUNT="$2"
      shift 2
      ;;
    --timeout)
      [[ -z "${2:-}" ]] && usage
      CYCLE_TIMEOUT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown option: $1"
      usage
      ;;
  esac
done

# Validate: --cycle and --timeout must be used together
if (( CYCLE_COUNT > 0 && CYCLE_TIMEOUT <= 0 )); then
  echo "Error: --cycle requires --timeout"
  usage
fi
if (( CYCLE_TIMEOUT > 0 && CYCLE_COUNT <= 0 )); then
  echo "Error: --timeout requires --cycle"
  usage
fi

# ── Kill process tree recursively ─────────────────────────────────────────
# Sends a signal to a process and all its descendants (children, grandchildren, etc.)
kill_tree() {
  local target_pid=$1
  local sig=${2:-TERM}
  # First recurse into children so we kill leaf processes before parents
  local children
  children=$(pgrep -P "$target_pid" 2>/dev/null) || true
  for child in $children; do
    kill_tree "$child" "$sig"
  done
  # Don't kill ourselves
  if [[ "$target_pid" != "$$" ]]; then
    kill -"$sig" "$target_pid" 2>/dev/null || true
  fi
}

# ── Cleanup handler ───────────────────────────────────────────────────────
cleanup() {
  echo ""
  echo -e "${YELLOW}Shutting down all services...${NC}"

  # SIGTERM the entire process tree of each tracked PID
  for i in "${!PIDS[@]}"; do
    local pid="${PIDS[$i]}"
    local name="${NAMES[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
      echo -e "  Stopping ${COLORS[$i]}$name${NC} (PID $pid)..."
    fi
  done
  # Kill all descendants of this script (covers both sides of every pipeline)
  kill_tree $$

  # Wait for processes to exit (up to 5 seconds)
  local timeout=5
  local elapsed=0
  while (( elapsed < timeout )); do
    local remaining
    remaining=$(pgrep -P $$ 2>/dev/null) || true
    if [[ -z "$remaining" ]]; then
      break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  # Force kill any survivors
  kill_tree $$ 9

  wait 2>/dev/null || true
  PIDS=()
  NAMES=()
  echo -e "${GREEN}All services stopped.${NC}"
}

# ── Launch all services ───────────────────────────────────────────────────
launch_services() {
  PIDS=()
  NAMES=()

  echo "========================================"
  echo "  Contour Tracker - Service Launcher"
  echo "========================================"
  echo ""

  for i in "${!SERVICES[@]}"; do
    IFS='|' read -r name workdir cmd <<< "${SERVICES[$i]}"
    color="${COLORS[$i]}"

    echo -e "[$((i+1))/${#SERVICES[@]}] Starting ${color}$name${NC} in $workdir"
    echo -e "       Command: $cmd"

    # Launch the process, piping output through sed to add color-coded prefix
    (cd "$workdir" && exec $cmd) 2>&1 | sed -u "s/^/$(printf "${color}[%-17s]${NC} " "$name")/" &
    PIDS+=($!)
    NAMES+=("$name")

    # Wait 3 seconds between launches (except after the last one)
    if (( i < ${#SERVICES[@]} - 1 )); then
      sleep 3
    fi
  done

  echo ""
  echo "========================================"
  echo "  All ${#SERVICES[@]} services started"
  echo "========================================"
  echo ""
  echo "PIDs:"
  for i in "${!PIDS[@]}"; do
    echo -e "  ${COLORS[$i]}${NAMES[$i]}${NC}: ${PIDS[$i]}"
  done
  echo ""
}

# ── Main ──────────────────────────────────────────────────────────────────
if (( CYCLE_COUNT > 0 )); then
  # Cycle mode: run N cycles, each with a timeout
  # In cycle mode, trap Ctrl+C to stop everything immediately
  CYCLE_INTERRUPTED=false
  trap 'CYCLE_INTERRUPTED=true; cleanup; exit 1' SIGINT SIGTERM

  for (( cycle=1; cycle<=CYCLE_COUNT; cycle++ )); do
    echo ""
    echo -e "${CYAN}════════════════════════════════════════${NC}"
    echo -e "${CYAN}  Cycle $cycle / $CYCLE_COUNT  (timeout: ${CYCLE_TIMEOUT}s)${NC}"
    echo -e "${CYAN}════════════════════════════════════════${NC}"
    echo ""

    launch_services

    echo "Waiting ${CYCLE_TIMEOUT} seconds before stopping..."
    echo "Press Ctrl+C to abort all cycles"
    echo "========================================"
    echo ""

    # Wait for timeout, but check for interruption
    for (( s=1; s<=CYCLE_TIMEOUT; s++ )); do
      if $CYCLE_INTERRUPTED; then
        break
      fi
      sleep 1
    done

    if $CYCLE_INTERRUPTED; then
      break
    fi

    echo ""
    echo -e "${YELLOW}Timeout reached (${CYCLE_TIMEOUT}s). Stopping services for cycle $cycle...${NC}"
    cleanup

    # Brief pause between cycles (except after the last one)
    if (( cycle < CYCLE_COUNT )); then
      echo ""
      echo -e "${CYAN}Pausing 30 seconds before next cycle...${NC}"
      sleep 30
    fi
  done

  echo ""
  echo -e "${GREEN}════════════════════════════════════════${NC}"
  echo -e "${GREEN}  All $CYCLE_COUNT cycles completed.${NC}"
  echo -e "${GREEN}════════════════════════════════════════${NC}"
else
  # Normal mode: run until Ctrl+C
  trap cleanup SIGINT SIGTERM

  launch_services

  echo "Press Ctrl+C to stop all services"
  echo "========================================"
  echo ""

  # Wait for all child processes (or for the trap to fire)
  wait
fi
