import subprocess
import argparse
import time
import os


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='sofaScript')
    parser.add_argument('-g', '--generic', action='store_true')
    parser.add_argument('-c', '--cube', action='store_true')
    parser.add_argument('-s', '--sphere', action='store_true')
    parser.add_argument('--hand', action='store_true')
    parser.add_argument('--arm750', action='store_true')
    parser.add_argument('--sm', action='store_true')

    args = parser.parse_args()
    cmd = ["runSofa", "src/main.py"]

    if args.generic:
        # time.sleep(3)
        cmd.extend(["--argv", "generic"])
    if args.cube:
        cmd.extend(["--argv", "cube"])
    if args.sphere:
        cmd.extend(["--argv", "sphere"])
    if args.hand:
        cmd.extend(["--argv", "hand"])
    if args.arm750:
        cmd.extend(["--argv", "arm750"])
    if args.sm:
        cmd.extend(["--argv", "sm"])

    subprocess.run(cmd)
