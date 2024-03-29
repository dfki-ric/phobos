import os

from . import misc
from .misc import execute_shell_command
from ..commandline_logging import get_logger

log = get_logger(__name__)


class MergeRequest(object):
    target = "master"
    title = ""
    description = ""
    mention = ""
    _tag = ""

    def __init__(self):
        if "CI_PROJECT_NAME" in os.environ.keys() and "CI_COMMIT_SHORT_SHA" in os.environ.keys():
            self._tag = " MR generated by the CI from " + os.environ["CI_PROJECT_NAME"] + "-commit " \
                        + os.environ["CI_COMMIT_SHORT_SHA"]

    def get_options(self):
        self.description = self.description.replace("\n", "<br/>")
        options = " -o merge_request.create"
        options += " -o merge_request.target='" + self.target + "'"
        options += " -o merge_request.title='" + self.title + "'"
        options += " -o merge_request.description='" + self.mention + " " + self.description + "<br/>" + self._tag + "'"
        if len(self.mention.strip().strip("@").split()) != 0:
            options += " -o merge_request.assign='" + self.mention.strip().strip("@").split()[0] + "'"
        return options


def get_root(cwd):
    try:
        root_path, _ = execute_shell_command("git rev-parse --show-toplevel", cwd=cwd, silent=True)
        return root_path.strip()
    except Exception:
        return None


def clone(repo, target, branch=None, cwd=None, recursive=False, ignore_failure=False, commit_id=None,
          shallow=1, pipeline=None):
    execute_shell_command("git lfs install || true", cwd)
    if os.path.exists(target):
        log.info(f"Deleting {pipeline.relpath(target) if pipeline is not None else target}")
        os.system("rm -rf {}".format(target))
    cmd = "git clone"
    if branch is not None:
        cmd += " -b " + branch
    if recursive:
        cmd += " --recurse-submodules "
    if (commit_id is None or shallow > 1) and shallow is not False and shallow is not None and shallow != 0:
        cmd += " --depth " + str(shallow) + " "
        if recursive:
            cmd += " --shallow-submodules "
    cmd += " " + repo + " " + target  # not loading with --recursive as we don't need the meshes submodule
    if cwd is None and pipeline is not None:
        cwd = pipeline.root
    if ignore_failure:
        cmd += " || true"
    execute_shell_command(cmd, cwd)
    if commit_id is not None:
        checkout(commit_id, target, force=True)


def checkout(commit_id, repo, force=False):
    execute_shell_command("git stash", repo)
    execute_shell_command("git checkout " + ("-f " if force else "") + commit_id, repo)
    execute_shell_command("git submodule update --init --recursive ", repo)


def update(repo, update_remote="autobuild", update_target_branch="$CI_UPDATE_TARGET_BRANCH"):
    execute_shell_command("git stash", repo)
    execute_shell_command("git branch " + update_target_branch + " || true", repo)
    execute_shell_command("git checkout " + update_target_branch, repo)
    execute_shell_command("git remote -v update || true", repo)
    execute_shell_command("git fetch --all || true", repo)
    execute_shell_command("git reset --hard " + update_remote + "/" + update_target_branch + " || true", repo)
    if update_target_branch.startswith("$"):
        update_target_branch = os.environ[update_target_branch[1:]]
    if update_target_branch == "master":
        execute_shell_command("git pull " + update_remote + " $CI_UPDATE_TARGET_BRANCH -s ours || true", repo)
    else:  # update_target_branch == "$CI_UPDATE_TARGET_BRANCH"
        execute_shell_command("git pull " + update_remote + " master -s ours || true", repo)


def revision(repo):
    commit_hash, _ = execute_shell_command("git rev-parse HEAD", repo)
    return commit_hash.strip()


def get_previous_commit_hash(repo):
    commit_hash, _ = execute_shell_command("git rev-parse HEAD^1", repo)
    return commit_hash.strip()


def get_branch(repo):
    branch_name, _ = execute_shell_command("git rev-parse --abbrev-ref HEAD", repo)
    return branch_name.strip()


def get_commit_message(repo):
    commit_message, _ = execute_shell_command("echo $(git log -1 --pretty=%B)", repo)
    return commit_message


def commit(repo, message=None, origin_repo=None):
    if message is None and "CI_COMMIT_SHORT_SHA" in os.environ.keys() and "CI_PROJECT_NAME" in os.environ.keys():
        message = "[CI] "
        if "CI_COMMIT_MESSAGE" in os.environ.keys():
            message += os.environ["CI_COMMIT_MESSAGE"] + "\n"
        elif origin_repo is not None:
            try:
                message += get_commit_message(origin_repo) + "\n"
            except:
                pass
        message += "Update from phobos-CI run \nBased on " + os.environ["CI_PROJECT_NAME"] + " commit " + os.environ[
            "CI_COMMIT_SHORT_SHA"]
    elif message is None:
        message = "Commit generated by manual pipeline script run"
    log.info(f"Commiting to {repo}")
    commit_message = ""
    for m in message.split("\n"):
        commit_message += " -m '" + m + "'"
    execute_shell_command("git add -A * || true", repo)
    execute_shell_command("git commit " + commit_message + " || true", repo)
    return revision(repo)


def reset(repo, remote, branch):
    execute_shell_command("git reset --hard " + remote + "/" + branch, repo)


def add_remote(repo, target_remote_url, target_remote_name="target_remote"):
    execute_shell_command("git remote add " + target_remote_name + " " + target_remote_url + " || true", repo)
    execute_shell_command("git remote -v update || true", repo)


def push(repo, remote="target_remote", branch="$CI_UPDATE_TARGET_BRANCH", merge_request=None):
    options = "" if merge_request is None else merge_request.get_options()
    execute_shell_command("git push " + remote + " " + branch + " " + options, repo, dry_run=False)
    return os.environ[branch[1:]] if branch.startswith("$") else branch


def clear_repo(repo):
    """Deletes everything in this repo"""
    log.info(f"Empty repo {repo}")
    if os.path.isfile(os.path.join(repo, ".gitmodules")):
        execute_shell_command("git submodule deinit -f * || true", repo)
        execute_shell_command("rm .gitmodules || true", repo)
        execute_shell_command("echo '' >> .gitmodules || true", repo)
        execute_shell_command("git add .gitmodules || true", repo)
    for f in os.listdir(repo):
        if not (f.startswith(".git") or f.lower() in ["readme.md", "manifest.xml", "scripts", "blender"]):
            execute_shell_command("git rm -rf --cached " + f + " || true", repo)
            execute_shell_command("rm -rf " + f + " || true", repo)


def add_submodule(repo, remote, path, commit=None, branch="master"):
    execute_shell_command("git remote rename autobuild origin || true", repo)
    execute_shell_command("git submodule add -b " + branch + " -f " + remote + " " + path, repo)
    execute_shell_command("git submodule update --init --recursive " + path, repo)
    if commit is not None:
        execute_shell_command("git remote update", os.path.join(repo, path))
        execute_shell_command("git checkout --detach " + commit, os.path.join(repo, path))
    execute_shell_command("git remote rename origin autobuild || true", repo)


def install_lfs(repo, track):
    execute_shell_command("git remote rename autobuild origin || true", repo)
    execute_shell_command("git lfs install || true", repo, silent=True)
    if ".gitattributes" not in os.listdir(repo) or False:
        if type(track) is str:
            execute_shell_command("git lfs track '" + track.lower() + "' || true", repo, silent=True)
            execute_shell_command("git lfs track '" + track.upper() + "' || true", repo, silent=True)
        else:
            for t in track:
                execute_shell_command("git lfs track '" + t.lower() + "' || true", repo, silent=True)
                execute_shell_command("git lfs track '" + t.upper() + "' || true", repo, silent=True)
    execute_shell_command("git remote rename origin autobuild || true", repo)
    execute_shell_command("git add .gitattributes || true", repo)


def ignore(repo, ignore):
    ign_file = os.path.join(repo, ".gitignore")
    if type(ignore) == str:
        ignore = ignore.split()
    content = []
    if os.path.isfile(ign_file):
        with open(ign_file, "r") as f:
            content = [l if not l.endswith("\n") else l[:-1] for l in f.readlines()]
    with open(".gitignore", "a") as f:
        for i in ignore:
            if i not in content:
                f.write(i+"\n")


def get_repo_data(directory):
    author, _ = execute_shell_command("git show -s | grep Author || true", directory)
    author = " ".join(author.split()[1:])
    maintainer, _ = execute_shell_command("git config user.name; git config user.email", directory, silent=True)
    maintainer = " ".join(maintainer.split())
    if maintainer == " ":
        maintainer = ""
    if author == "":
        author = maintainer
    url, _ = execute_shell_command("git remote get-url --push autobuild || true", directory, silent=True)
    if url == "":
        url, _ = execute_shell_command("git remote get-url --push origin || true", directory, silent=True)
    return author.strip().replace("<", "").replace(">", ""), maintainer.strip().replace("<", "").replace(">", ""),\
           url.strip()


def has_diff(repo, file=None):
    file = os.path.abspath(file)
    repo = os.path.abspath(repo)
    try:
        if file is None:
            execute_shell_command("git diff --exit-code", cwd=repo)
        else:
            execute_shell_command("git diff --exit-code " + os.path.relpath(file, repo), cwd=repo)
        return False
    except Exception as _:
        return True


def create_pipeline_badge(pipeline, label, message, color, target, filename=None):
    """Creates a badge with label as name and saves it to the target directory"""
    name = label.replace("-", "--")
    message = message.replace("-", "--")
    available_colors = ["brightgreen", "green", "yellowgreen", "yellow", "orange", "red", "blue", "lightgrey",
                        "blueviolet",
                        "success", "important", "critical", "informational", "inactive"]
    if color.lower() not in available_colors:
        raise AssertionError("Allowed colors are " + repr(available_colors) + ", not " + color)
    misc.create_dir(pipeline, target)
    badge = os.path.join(target, label) + ".svg" if filename is None else os.path.join(target, filename)
    execute_shell_command('curl "' + os.path.join("https://img.shields.io/badge/",
                                                  "-".join([name, message, color.lower()])) + '" >' + badge)
    return badge
