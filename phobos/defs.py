import os

EULER_CONVENTION = 'xyz'
RPY_CONVENTION = 'xyz'

HYRODYN_AVAILABLE = False
try:
    import hyrodyn
    if "AUTOPROJ_CURRENT_ROOT" in os.environ.keys():
        HYRODYN_AVAILABLE = True
        print("HyRoDyn tests available.", flush=True)
    else:
        HYRODYN_AVAILABLE = False
        print("HyRoDyn can't be used as the autoproj env hasn't be sourced.", flush=True)
except ImportError:
    print("HyRoDyn tests not available.", flush=True)

PYBULLET_AVAILABLE = False
try:
    import pybullet as pb
    PYBULLET_AVAILABLE = True
    print("Pybullet tests available.", flush=True)
except ImportError:
    print("Pybullet tests not available.", flush=True)

DEIMOS_AVAILABLE = False
try:
    from deimos.deimos import Deimos
    DEIMOS_AVAILABLE = True
    print("Deimos available.", flush=True)
except ImportError:
    print("Deimos not available.", flush=True)
