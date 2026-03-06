"""Entry point: argument parsing, component assembly, startup."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading

from core.comm import start_contour_server, start_pruning, start_subscribers
from core.config import AppConfig
from core.grid import OccupancyGrid


def main() -> None:
    parser = argparse.ArgumentParser(description="2D Occupancy Grid Map")
    parser.add_argument("--config", default="config.json", help="Path to JSON config file")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyGame visualization")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(threadName)s] %(levelname)s %(name)s: %(message)s",
    )
    logger = logging.getLogger("main")

    # Load config
    config = AppConfig.load(args.config)
    logger.info("Config loaded: cell_size=%.3f", config.cell_size)

    if args.no_viz:
        config.visualization.enabled = False

    # Create core components
    grid = OccupancyGrid(config.cell_size)
    stop_event = threading.Event()

    # SIGINT handler
    def _sigint(signum, frame):
        logger.info("SIGINT received, stopping...")
        stop_event.set()

    signal.signal(signal.SIGINT, _sigint)

    # Start communication threads
    subscribers = start_subscribers(config, grid, stop_event)
    contour_server = start_contour_server(config, grid, stop_event)

    # Start pruning thread
    pruner = start_pruning(config, grid, stop_event)

    logger.info("All threads started. %d subscribers + contour server.", len(subscribers))

    # Main thread: visualization or idle wait
    if config.visualization.enabled:
        try:
            from core.visualizer import Visualizer

            viz = Visualizer(
                config.visualization.window_width,
                config.visualization.window_height,
                config.visualization.fps,
                grid,
                contour_server,
                camera_x=config.visualization.camera_x,
                camera_y=config.visualization.camera_y,
                render_threshold=config.visualization.render_threshold,
                grid_opacity=config.visualization.grid_opacity,
                follow_auv=config.visualization.follow_auv,
            )
            viz.run(stop_event)
        except ImportError:
            logger.warning("PyGame not available, falling back to idle wait.")
            config.visualization.enabled = False

    if not config.visualization.enabled:
        logger.info("No visualization. Waiting for stop signal (Ctrl+C)...")
        try:
            stop_event.wait()
        except KeyboardInterrupt:
            stop_event.set()

    # Shutdown
    logger.info("Shutting down...")
    stop_event.set()

    for s in subscribers:
        s.join(timeout=2.0)
    contour_server.join(timeout=2.0)
    if pruner is not None:
        pruner.join(timeout=2.0)

    logger.info("Done.")


if __name__ == "__main__":
    main()
