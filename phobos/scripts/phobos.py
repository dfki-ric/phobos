#!python
from pkg_resources import get_distribution, DistributionNotFound

try:
    # Change here if project is renamed and does not equal the package name
    __version__ = get_distribution("phobos").version
except DistributionNotFound:
    __version__ = 'v2.0.0'
finally:
    del get_distribution, DistributionNotFound


def main():
    import sys
    from .. import scripts

    print(f"\n*** This is phobos {__version__} ***")
    script_files = [f for f in dir(scripts) if not f.startswith("__") and f != "phobos"]
    available_scripts = [(f, getattr(scripts, f).INFO, None) for f in script_files if getattr(scripts, f).can_be_used()]
    unavailable_scripts = [(f, getattr(scripts, f).INFO, getattr(scripts, f).cant_be_used_msg()) for f in script_files
                           if not getattr(scripts, f).can_be_used()]

    if len(sys.argv) > 1 and sys.argv[1] in [ascr[0] for ascr in available_scripts + unavailable_scripts]:
        if sys.argv[1] in unavailable_scripts:
            print(f"Attention: Script might not work properly:" + getattr(scripts, sys.argv[1]).cant_be_used_msg())
        if "--cProfile" in sys.argv:
            print("Running with profiler")
            sys.argv.remove("--cProfile")
            try:
                import cProfile
            except ImportError:
                import profile as cProfile
            import pstats
            import traceback
            from pstats import SortKey
            import datetime
            retval = 4
            with cProfile.Profile() as pr:
                try:
                    retval = getattr(scripts, sys.argv[1]).main(sys.argv[2:])
                except KeyboardInterrupt:
                    traceback.print_exc()
            with open(datetime.datetime.now().isoformat()+"_phobos_cProfile.log", "w") as s:
                s.write(f"# {sys.argv}")
                sortby = SortKey.CUMULATIVE
                ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
                ps.print_stats()
            sys.exit(retval)
        else:
            sys.exit(getattr(scripts, sys.argv[1]).main(sys.argv[2:]))
    else:
        print("Phobos is a tool to process simulation models \n")
        print("Usage:")
        print("phobos COMMAND ARGUMENTS")
        print("Commands:")
        spaces = 0
        for script in available_scripts + unavailable_scripts:
            if len(script[0]) > spaces:
                spaces = len(script[0])
        spaces += 3
        for script, info, _ in sorted(available_scripts):
            print("       "+script+" "*(spaces-len(script))+info)
        for script, info, error in unavailable_scripts:
            print("     X "+script+" "*(spaces-len(script))+info+"  UNAVAILABLE: "+error)


if __name__ == "__main__":
    main()
