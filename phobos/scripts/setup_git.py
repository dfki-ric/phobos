#!python3

def can_be_used():
    return True


def cant_be_used_msg():
    return ""


INFO = 'Sets-up a git repository for a simulation/control model.'


def main(args):
    import argparse
    import os
    import pkg_resources
    from ..utils import git, misc
    from ..commandline_logging import setup_logger_level, BASE_LOG_LEVEL

    parser = argparse.ArgumentParser(description=INFO, prog="phobos " + os.path.basename(__file__)[:-3])
    parser.add_argument('directory', type=str, help='Path to model directory')
    parser.add_argument('-r', '--remote', type=str, help='The git-repository remote url (required for -c)', action="store", default=None)
    parser.add_argument('-i', '--init', help='Call git init (instead of -c)', action="store_true", default=False)
    parser.add_argument('-c', '--clone', help='Clone the repo (requires remote; instead of -i)', action="store_true", default=False)
    parser.add_argument('-e', '--apply-on-existing', help='Use this when you want to apply this on an existing directory', action="store_true", default=False)
    parser.add_argument('-l', '--no-lfs', help='Do not use git-lfs', action="store_true", default=False)
    parser.add_argument("--loglevel", help="The log level", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        default=BASE_LOG_LEVEL)
    args = parser.parse_args(args)
    log = setup_logger_level(log_level=args.loglevel)

    if not os.path.exists(args.directory):
        log.info("Creating directory")
        if args.clone:
            assert args.remote is not None
            os.makedirs(os.path.basename(args.directory), exist_ok=True)
            git.clone(
                repo=args.remote,
                target=args.directory,
                cwd=args.directory
            )
        else:
            os.makedirs(args.directory, exist_ok=True)
    elif not args.apply_on_existing:
        raise FileExistsError(args.directory + " already exists!")

    if args.init:
        assert not args.clone
        log.info("Initalizing repository")
        misc.execute_shell_command("git init", args.directory)
        if args.remote:
            git.add_remote(args.directory, args.remote, "origin")

    if not args.no_lfs:
        git.install_lfs(
            args.directory,
            ["*.stl", "*.obj", ".mtl", "*.bobj", "*.iv", "*.dae", "*.blend", "*.pdf", "*.jpg", "*.jpeg", "*.png"]
        )
        misc.execute_shell_command("git add .gitattributes", cwd=args.directory)

    git.ignore(args.directory, ["*.blend1", "*.log", "*.gv"])
    misc.execute_shell_command("git add .gitignore", cwd=args.directory)

    # add manifest.xml if necessary
    manifest_path = os.path.join(args.directory, "manifest.xml")
    author, maintainer, url = git.get_repo_data(args.directory)
    if not os.path.isfile(manifest_path):
        misc.copy(None, pkg_resources.resource_filename("phobos", "data/manifest.xml.in"), manifest_path)
        with open(manifest_path, "r") as manifest:
            content = manifest.read()
            content = misc.regex_replace(content, {
                "\$INPUTNAME": os.path.basename(args.directory),
                "\$AUTHOR": "<author>" + author + "</author>",
                "\$MAINTAINER": "<maintainer>"+maintainer+"</maintainer>",
                "\$URL": "<url>" + url + "</url>",
            })
        with open(manifest_path, "w") as manifest:
            manifest.write(content)

    return 0


if __name__ == '__main__':
    import sys
    main(sys.argv)
