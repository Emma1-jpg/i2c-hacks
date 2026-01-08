#!/usr/bin/env python3
"""
BNO055 IMU 3D Visualizer

Real-time 3D visualization of orientation data from a BNO055 IMU sensor.
Shows an aircraft-like model that rotates based on heading, roll, and pitch,
along with a compass rose and orientation readouts.

Usage:
    python main.py           # Use real BNO055 sensor
    python main.py --mock    # Use mock sensor for testing without hardware
"""

import sys
import time
import argparse
import pygame

from sensor import BNO055, MockSensor
from renderer import IMURenderer


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='3D IMU Visualizer for BNO055'
    )
    parser.add_argument(
        '--mock', action='store_true',
        help='Use mock sensor (for testing without hardware)'
    )
    parser.add_argument(
        '--bus', type=int, default=2,
        help='I2C bus number (default: 2 for i915 gmbus vga)'
    )
    parser.add_argument(
        '--address', type=lambda x: int(x, 0), default=0x29,
        help='I2C address (default: 0x29)'
    )
    parser.add_argument(
        '--width', type=int, default=1024,
        help='Window width (default: 1024)'
    )
    parser.add_argument(
        '--height', type=int, default=768,
        help='Window height (default: 768)'
    )
    parser.add_argument(
        '--rate', type=float, default=50,
        help='Update rate in Hz (default: 50)'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Create sensor
    if args.mock:
        print("Using mock sensor (no hardware)")
        sensor = MockSensor()
    else:
        print(f"Connecting to BNO055 on bus {args.bus}, address 0x{args.address:02x}")
        sensor = BNO055(bus_num=args.bus, address=args.address)

    # Create renderer
    renderer = IMURenderer(width=args.width, height=args.height)

    # Calculate frame timing
    frame_time = 1.0 / args.rate

    try:
        with sensor:
            renderer.init()
            print("Visualization started. Press ESC to quit.")

            clock = pygame.time.Clock()

            while True:
                # Handle events
                if not renderer.handle_events():
                    break

                # Read sensor
                orientation = sensor.read_euler()
                calibration = sensor.get_calibration()

                # Update and render
                renderer.update(orientation, calibration)
                renderer.render()

                # Maintain frame rate
                clock.tick(args.rate)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        renderer.cleanup()

    print("Visualization closed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
