"""
Actual helper script to execute code.
It takes care of proper error handling (e.g. if you press CTRL+C) and the difference between
running code on the robot vs. in simulation.

Usage:
  python3 run.py --sim lab1 [for simulation]
  python3 run.py lab1 [to run on a robot]
"""


import sys
import argparse
import importlib

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", help="run specified module")
    parser.add_argument("--sim", help="Run using VREP simulation", action="store_true")
    args = parser.parse_args()
    clientID = None
    if args.sim:
        from factory import FactorySimulation
        factory = FactorySimulation()
    else:
        from factory import FactoryCreate
        factory = FactoryCreate()
    try:
        if args.run.endswith(".py"):
            args.run = args.run[:-3]
        mod = importlib.import_module(args.run)
        Run = getattr(mod, "Run")
        r = Run(factory)
        r.run()
    except KeyboardInterrupt:
        pass
    except:
        factory.close()
        raise

    factory.close()

    # quit
    sys.exit()
