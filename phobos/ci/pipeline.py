import os.path
import sys
import traceback
from copy import deepcopy
import yaml

from ..defs import *
from .test_model import TestModel
from .model import Model
from .combined_model import CombinedModel
from .model_testing import ModelTest
from .compare_model import CompareModel
from .xtype_model import XTypeModel
from ..utils import git, misc

# Failure States:
F_LOAD = int('00000001', 2)
F_PROC = int('00000010', 2)
F_TEST = int('00000100', 2)
F_DEPL = int('00001000', 2)
NA_LOAD = int('00010000', 2)
NA_PROC = int('00100000', 2)
NA_TEST = int('01000000', 2)
NA_DEPL = int('10000000', 2)


class Pipeline(yaml.YAMLObject):
    def __init__(self, configfile, processed_model_exists):
        self.processing_failed = {}
        self.test_results = {}
        self.configdir = os.path.abspath(os.path.dirname(configfile))
        self.processed_model_exists = processed_model_exists
        if not os.path.isfile(configfile):
            raise Exception('{} not found!'.format(configfile))

        kwargs = yaml.safe_load(open(configfile, 'r'))['pipeline']

        for (k, v) in kwargs.items():
            setattr(self, k, v)

        self.root = os.path.abspath(os.path.join(self.configdir, self.root))
        print("Working from", self.root, flush=True)
        self.git_rev = git.revision(os.path.abspath(self.configdir))
        self.temp_dir = os.path.join(self.root, "temp")
        self.faillog = os.path.join(self.temp_dir, "failures.txt")
        self.test_protocol = os.path.join(self.temp_dir, "test_protocol.txt")
        if os.path.isfile(self.faillog) and processed_model_exists:
            with open(self.faillog, "r") as f:
                self.processing_failed = yaml.safe_load(f.read())

        for k, v in self.modeltypes.items():
            if k != "default":
                for dk, dv in self.modeltypes["default"].items():
                    if dk not in v.keys():
                        v.update({dk: deepcopy(dv)})

        for name, cfg in self.modeltypes.items():
            cfg.update({
                "meshespath": self.meshes[cfg["output_mesh_format"]]
            })

        print("Model types:\n", yaml.safe_dump(self.modeltypes, default_flow_style=False), flush=True)
        print("Models to process:", self.model_definitions, flush=True)
        print("Finished reading config", configfile, flush=True)

        self.models = []
        imported_meshes = []
        order = 0
        for md in self.model_definitions:
            order += 1
            print("Loading ", md, "...", flush=True)
            if md[:-4] not in self.processing_failed.keys():
                self.processing_failed[md[:-4]] = {"load": "N/A", "process": "N/A", "test": "N/A", "deploy": "N/A",
                                                   "order": order}
            try:
                cfg = yaml.safe_load(open(os.path.join(self.configdir, md), "r").read())
                cfg_ = cfg[list(cfg.keys())[0]]
            except KeyError as e:
                self.processing_failed[md[:-4]]["load"] = ''.join(traceback.format_exception(None, e, e.__traceback__))
                print("Could not load model config " + md)
                traceback.print_exc()
                continue
            fstate = self._get_model_failure_state(md[:-4])
            self.processing_failed[md[:-4]]["load"] = ""
            if bool(fstate & (F_PROC | NA_PROC)) and processed_model_exists:
                print("WARNING: Skipping model " + md + " as it was not successfully processed")
                self.processing_failed[md[:-4]]["load"] += \
                    "Skipping model " + md + " as it was not successfully processed!\n"
                continue
            elif bool(fstate & (F_PROC | NA_PROC)):
                # reset processing error
                self.processing_failed[md[:-4]]["process"] = "N/A"
            if "derived_base" in cfg_.keys() and self._get_model_failure_state(cfg_["derived_base"][:-4]) & (
                    F_LOAD | F_PROC):
                print("WARNING: Skipping model " + md + " as parent model was not successfully processed!")
                self.processing_failed[md[:-4]]["load"] += \
                    "Skipping model " + md + " as parent model was not successfully processed!\n"
                continue
            if "depends_on" in cfg_.keys() and any(["derived_base" in v.keys() and self._get_model_failure_state(
                    v["derived_base"][:-4]) & (F_LOAD | F_PROC) for _, v in cfg_["depends_on"].items()]):
                print("WARNING: Skipping model " + md + " as at least one parent model was not successfully processed!")
                self.processing_failed[md[:-4]]["load"] += \
                    "Skipping model " + md + " as at least one parent model was not successfully processed!\n"
                continue
            try:
                if list(cfg.keys())[0] == "model":
                    self.models += [Model(os.path.join(self.configdir, md), self, self.processed_model_exists)]
                elif list(cfg.keys())[0] == "combined_model":
                    self.models += [CombinedModel(os.path.join(self.configdir, md), self, self.processed_model_exists)]
                elif list(cfg.keys())[0] == "xtype_model":
                    self.models += [XTypeModel(os.path.join(self.configdir, md), self, self.processed_model_exists)]
                else:
                    self.processing_failed[md[:-4]][
                        "load"] = "Skipping " + md + " as it is no valid model definition!\n"
                    print(self.processing_failed[md[:-4]]["load"])
                    continue
                for m in self.models[-1].get_imported_meshes():
                    for m2 in imported_meshes:
                        if os.path.basename(m) == os.path.basename(m2) and os.path.dirname(m) != os.path.dirname(m):
                            raise Exception(
                                "You can not import two different meshes with the same name!\n"
                                "In model {} you tried to import:\n  {} but\n  {} already exists!".format(md, m, m2)
                            )
                imported_meshes += self.models[-1].get_imported_meshes()
                self.processing_failed[md[:-4]]["load"] = "Good"
            except Exception as e:
                self.processing_failed[md[:-4]][
                    "load"] += "Loading " + md + " failed with the following error:" + ''.join(
                    traceback.format_exception(None, e, e.__traceback__))
                print(self.processing_failed[md[:-4]]["load"])
                continue
        if os.path.exists(os.path.join(self.temp_dir)):
            with open(self.faillog, "w") as f:
                f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def _get_model_failure_state(self, modelname):
        state = 0
        if modelname not in self.processing_failed:
            return state
        if not self.processing_failed[modelname]["load"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["load"].upper().startswith("N/A"):
            state += F_LOAD
        if not self.processing_failed[modelname]["process"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["process"].upper().startswith("N/A"):
            state += F_PROC
        if not self.processing_failed[modelname]["test"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["test"].upper().startswith("N/A"):
            state += F_TEST
        if not self.processing_failed[modelname]["deploy"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["deploy"].upper().startswith("N/A"):
            state += F_DEPL
        if self.processing_failed[modelname]["load"].upper().startswith("N/A"):
            state += NA_LOAD
        if self.processing_failed[modelname]["process"].upper().startswith("N/A"):
            state += NA_PROC
        if self.processing_failed[modelname]["test"].upper().startswith("N/A"):
            state += NA_TEST
        if self.processing_failed[modelname]["deploy"].upper().startswith("N/A"):
            state += NA_DEPL
        return state

    def _get_model_na_state(self, modelname):
        state = 0
        if modelname not in self.processing_failed:
            return state

        return state

    def number_unfinished_models(self):
        """Returns the number of models that have not been successfully or completely processed"""
        failures = [self._get_model_failure_state(mn[:-4]) & (
                F_LOAD | F_PROC | F_TEST | F_DEPL | NA_LOAD | NA_PROC | NA_TEST | NA_DEPL) for mn in
                    self.model_definitions]
        return len([x for x in failures if x])

    def has_failure(self):
        return any([self._get_model_failure_state(mn[:-4]) & (F_LOAD | F_PROC | F_TEST | F_DEPL) for mn in
                    self.model_definitions])

    def get_coverage(self, phases=None, allow_na=False):
        if phases is None:
            phases = []
        if len(phases) == 0:
            phases = ["process", "test", "deploy"]
        log = yaml.safe_load(open(self.faillog, "r").read())
        all_models = float(len(log.keys()))
        fails = 0.0
        all_models *= len(phases)
        for p in phases:
            for k, v in log.items():
                if allow_na:
                    fails += not v[p].upper().startswith("GOOD") or not v[p].upper().startswith("N/A")
                else:
                    fails += not v[p].upper().startswith("GOOD")
        return 1 - fails / all_models

    def print_fail_log(self, file=sys.stdout):
        log = yaml.safe_load(open(self.faillog, "r").read())
        print("\nModel Failure Report:\n---------------------", file=file)
        report = [(k, v) for k, v in log.items()]
        order_val = ["order", "load", "process", "test", "deploy"]
        if len(report) == 0:
            print("Nothing done that could be reported!", file=file)
        for r in sorted(report, key=lambda x: x[1]["order"]):
            print(r[0] + ":", file=file)
            values = [(k, v) for k, v in r[1].items()]
            values = sorted(values, key=lambda x: order_val.index(x[0]))
            for v in values:
                if v[0] == "order":
                    continue
                print("  " + v[0] + ": \t",
                      str(v[1]) if v[1].upper().startswith("GOOD") or v[1].upper().startswith("N/A") else "Error",
                      file=file)
                if not v[1].upper().startswith("GOOD") and not v[1].upper().startswith("N/A"):
                    for link in v[1].split("\n"):
                        print("    " + link, file=file)

    def process_models(self):
        # delete the temp_dir if there is already one
        misc.recreate_dir(self, self.temp_dir)
        with open(self.faillog, "w") as f:
            f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

        # create the central mesh folders
        for name, cfg in self.modeltypes.items():
            misc.create_dir(self, os.path.join(self.temp_dir, cfg["meshespath"]))
        if any(["also_export_bobj" in x and x["also_export_bobj"] is True for x in
                [v for k, v in self.modeltypes.items() if "smurf" in k]]):
            misc.create_dir(self, os.path.join(self.temp_dir, self.meshes["bobj"]))
        if any([hasattr(x, "export_kccd") and x.export_kccd is not False for x in self.models]):
            misc.create_dir(self, os.path.join(self.temp_dir, self.meshes["iv"]))
        for model in self.models:
            print("\nProcessing", model.modelname, "model...", flush=True)
            print("\nProcessing", model.modelname, "model...", flush=True, file=sys.stderr)
            if self._get_model_failure_state(model.modelname) & (F_LOAD | NA_LOAD):
                self.processing_failed[model.modelname]["process"] = \
                    "Skipping ", model.modelname, " as it model definition file wasn't loaded successfully!"
                print(self.processing_failed[model.modelname]["process"], flush=True)
                print(self.processing_failed[model.modelname]["process"], flush=True, file=sys.stderr)
                continue
            try:
                model.process()
                model.export()
                self.processing_failed[model.modelname]["process"] = "Good"
            except Exception as e:
                print("\nFailed processing", model.modelname, "model. Skipping to next...", flush=True)
                print("\nFailed processing", model.modelname, "model with the following error:\n", e, flush=True,
                      file=sys.stderr)
                self.processing_failed[model.modelname]["process"] = ''.join(
                    traceback.format_exception(None, e, e.__traceback__))
                traceback.print_exc()
        with open(self.faillog, "w") as f:
            f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def test_models(self):
        """Runs the configured test_routines over all models"""
        with open(self.faillog, "r") as f:
            self.processing_failed = yaml.safe_load(f.read())
        failures_ignored_for = []
        for model in self.models:
            fstate = self._get_model_failure_state(model.modelname)
            if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
                print("\nSkipping", model.modelname,
                      "model as it wasn't succesfully created. (Code: " + bin(fstate) + ")", flush=True)
                print("\nSkipping", model.modelname,
                      "model as it wasn't succesfully created. (Code: " + bin(fstate) + ")", flush=True,
                      file=sys.stderr)
                continue
            model.processed_model_exists = True
            model._load_robot()
            print("\nTesting", model.modelname, "model...", flush=True)
            print("\nTesting", model.modelname, "model...", flush=True, file=sys.stderr)
            try:
                model.recreate_sym_links()
                commit_hash = ""
                compare_model = None
                if any(["compare" in x for x in model.typedef["model_tests"]]) and type(model.compare_model) is dict:
                    # Load compare model
                    compare_model_path = os.path.join(model.tempdir, "compare_model")
                    git.clone(
                        self,
                        model.compare_model["git"],
                        compare_model_path,
                        branch=model.compare_model["branch"],
                        recursive=True,
                        ignore_failure=True
                    )
                    if "ignore_failing_tests_for" not in model.compare_model.keys():
                        model.compare_model["ignore_failing_tests_for"] = "None"
                    if os.path.exists(os.path.join(compare_model_path, model.compare_model["model_in_repo"])):
                        commit_hash, _ = misc.execute_shell_command("git rev-parse HEAD", compare_model_path)
                        if commit_hash.startswith(str(model.compare_model["ignore_failing_tests_for"])):
                            failures_ignored_for += [model.modelname]
                        print("Loading compare model:",
                              self.relpath(os.path.join(compare_model_path, model.compare_model["model_in_repo"])),
                              flush=True)
                        try:
                            compare_model = CompareModel(
                                name=model.robotname,
                                directory=compare_model_path,
                                robotfile=os.path.join(compare_model_path, model.compare_model["model_in_repo"]),
                                submechanisms_file=os.path.join(
                                    compare_model_path,
                                    model.compare_model["submechanisms_in_repo"]
                                    if "submechanisms_in_repo" in model.compare_model
                                    else "submechanisms/submechanisms.yml"
                                ) if hasattr(model, "submechanisms_file") else None
                            )
                        except Exception as e:
                            model.compare_model["issues"] = repr(e)
                            print("Failed to load compare model. Exception was:\n",
                                  ''.join(traceback.format_exception(None, e, e.__traceback__)) + "\n")
                            traceback.print_exc()
                    else:
                        print("WARNING: Compare model not found!", flush=True, file=sys.stderr)

                model_test = ModelTest(model, compare_model)
                print("\nRunning info procedures:", flush=True)
                for p in dir(model_test):
                    if p.startswith("info_"):
                        getattr(model_test, p)()
                print("\nRunning test procedures:", flush=True)
                self.test_results[model.modelname] = {
                    "ignore_failure": commit_hash.startswith(
                        str(model.compare_model["ignore_failing_tests_for"])) if compare_model is not None else False,
                    "compare_model_present": model_test.old is not None,
                    "compare_model": model.compare_model
                }

                def add_test_result(model_name, test_name, value):
                    i = 2
                    if test_name in self.test_results[model.modelname].keys():
                        while test_name + " " + str(i) in self.test_results[model.modelname].keys():
                            i += 1
                        test_name += " " + str(i)
                    self.test_results[model_name][test_name] = value
                    return value

                # these tests will be run always
                obligatory_tests = ["topological_self_consistency"]
                for otest in obligatory_tests:
                    if otest not in model.typedef["model_tests"]:
                        model.typedef["model_tests"] += [otest]
                # let's go testing
                for test in model.typedef["model_tests"]:
                    if type(test) is str:
                        print("  -> ", test, flush=True)
                        if not add_test_result(model.modelname, test, getattr(model_test, "test_" + test)()):
                            print("Test", test, "failed for", model.modelname, flush=True)
                    elif type(test) is dict and list(test.keys())[0] == "hyrodynChecks":
                        if not HYRODYN_AVAILABLE:
                            print("Hyrodyn checks not possible, as Hyrodyn couldn't be loaded", flush=True)
                        for htest in test["hyrodynChecks"]:
                            if type(htest) is str:
                                print("  -> ", htest, flush=True)
                                if not add_test_result(model.modelname, htest,
                                                       getattr(model_test, "test_hyrodyn_" + htest)()):
                                    print("Hyrodyn-Test", htest, "failed for", model.modelname, flush=True)
                            elif type(htest) is dict and "move_hyrodyn_model" in htest.keys():
                                k, v = list(htest.items())[0]
                                getattr(model_test, k)(v)
                                print("  -> ", k, flush=True)
                            elif type(htest) is dict:
                                k, v = list(htest.items())[0]
                                print("  -> ", k, flush=True)
                                if not add_test_result(model.modelname, k, getattr(model_test, "test_hyrodyn_" + k)(v)):
                                    print("Hyrodyn-Test", k, "failed for", model.modelname, flush=True)
                            else:
                                print("Couldn't process test definition", htest)
                    elif type(test) is dict:
                        k, v = list(test.items())[0]
                        print("  -> ", test, flush=True)
                        if not add_test_result(model.modelname, k, getattr(model_test, "test_" + k)(v)):
                            print("Hyrodyn-Test", test, "failed for", model.modelname, flush=True)
                self.processing_failed[model.modelname]["test"] = "Good"
            except Exception as e:
                print("\nFailed testing", model.modelname, "model. Skipping to next...", flush=True)
                print("\nFailed testing", model.modelname, "model with the following error:\n", e, flush=True,
                      file=sys.stderr)
                self.processing_failed[model.modelname]["test"] = ''.join(
                    traceback.format_exception(None, e, e.__traceback__)) + "\n"
                traceback.print_exc()
        test_protocol = {"all": ""}
        test_protocol["all"] = misc.append_string(test_protocol["all"], "----------\nTest protocol:", print=True,
                                                  flush=True)
        success = True
        for modelname, test in self.test_results.items():
            test_protocol[modelname] = ""
            test_protocol[modelname] = misc.append_string(test_protocol[modelname], "  " + modelname, end="",
                                                          print=True, flush=True)
            if test["ignore_failure"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Ignoring failures)", end="",
                                                              print=True, flush=True)
            if not test["compare_model_present"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Compare model not present):",
                                                              end="", print=True, flush=True)
            test_protocol[modelname] = misc.append_string(test_protocol[modelname], "\n    " + yaml.safe_dump(
                {"Compare Model": test["compare_model"]}, default_flow_style=False, indent=6) + "    Test Results:",
                                                          flush=True, print=True)
            for testname, result in test.items():
                if testname in ["ignore_failure", "compare_model_present", "compare_model"]:
                    continue
                else:
                    sign = "+"
                    if type(result) is str:
                        sign = "o"
                    elif not result:
                        sign = "-"
                    test_protocol[modelname] = misc.append_string(test_protocol[modelname], "    ", sign,
                                                                  testname + ":",
                                                                  str(result) if type(result) is bool else result,
                                                                  print=True, flush=True)
                    if not result and not test["ignore_failure"]:
                        success = False
                        if self.processing_failed[modelname]["test"].upper().startswith("GOOD") or \
                                self.processing_failed[modelname]["test"].upper().startswith("N/A"):
                            self.processing_failed[modelname]["test"] = ""
                        self.processing_failed[modelname][
                            "test"] += "Test " + testname + " failed (and was not ignored)!\n"
                    #     print(" !!! Causes Failure !!!", flush=True)
                    # else:
                    #     print(flush=True)
            with open(self.faillog, "w") as f:
                f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))
        print("The test routine", "succeeded!" if success else "failed!", flush=True)
        with open(self.test_protocol, "w") as f:
            f.write(yaml.safe_dump(test_protocol, default_flow_style=False))
        return success

    def deploy_models(self):
        """Moves everything from the temp to the repositories and pushes it to the repos"""
        with open(self.faillog, "r") as f:
            self.processing_failed = yaml.safe_load(f.read())
        mesh_repos = {}
        # copying the files to the repos
        uses_lfs = False
        if "BADGE_DIRECTORY" in os.environ.keys():
            # reset badges
            for model in self.models:
                git.create_pipeline_badge(self, model.modelname, "not processed", "inactive",
                                          target="${BADGE_DIRECTORY}")
            for name, path in self.meshes.items():
                git.create_pipeline_badge(self, os.path.basename(path), "not processed", "inactive",
                                          target="${BADGE_DIRECTORY}", filename=name + ".svg")
        for name, path in self.meshes.items():
            print("\nDeploying meshes", path, flush=True)
            print("\nDeploying meshes", path, flush=True, file=sys.stderr)
            repo = os.path.join(self.root, path)
            git.update(repo, update_target_branch="$CI_MESH_UPDATE_TARGET_BRANCH")
            if os.path.basename(path).lower().startswith("lfs"):
                uses_lfs = True
                git.install_lfs(repo,
                                track=["*.obj", "*.stl", "*.dae", "*.bobj", "*.iv", "*.mtl", "*.OBJ", "*.STL", "*.DAE",
                                       "*.BOBJ", "*.IV", "*.MTL"])
            git.clear_repo(repo)
            misc.copy(self, os.path.join(self.temp_dir, path) + "/*", os.path.join(self.root, path))
            commit_hash = git.commit(repo, origin_repo=os.path.abspath(self.configdir))
            if "BADGE_DIRECTORY" in os.environ.keys():
                git.create_pipeline_badge(self, os.path.basename(path).lower(), commit_hash[:6], 'informational',
                                          target="${BADGE_DIRECTORY}",
                                          filename=name + ".svg")
            mesh_repos.update({
                name: {
                    "repo": repo,
                    "commit": commit_hash
                }
            })
            git.add_remote(repo, self.remote_base + "/" + path)
            # mr = git.MergeRequest()
            # mr.target = self.mr_target_branch
            # mr.title = self.mr_title
            # mr.description = self.mr_description
            # if hasattr(self, "mr_mention"):
            #     mr.mention = self.mr_mention
            # for m in self.models:
            #     if hasattr(m, "mr_mention"):
            #         mr.mention += " "+m.mr_mention
            # git.push(repo, merge_request=mr)
            git.push(repo, branch="$CI_MESH_UPDATE_TARGET_BRANCH")
        for model in self.models:
            fstate = self._get_model_failure_state(model.modelname)
            if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
                print("\nSkipping", model.modelname,
                      "model as it wasn't successfully created. (Code: " + bin(fstate) + ")", flush=True)
                print("\nSkipping", model.modelname,
                      "model as it wasn't successfully created. (Code: " + bin(fstate) + ")", flush=True,
                      file=sys.stderr)
                continue
            print("\nDeploying", model.modelname, "model...", flush=True)
            print("\nDeploying", model.modelname, "model...", flush=True, file=sys.stderr)
            try:
                dpl_msg = model.deploy(mesh_repos, uses_lfs=uses_lfs,
                                       failed_model=bool(fstate & F_TEST) or bool(fstate & NA_TEST))
                self.processing_failed[model.modelname]["deploy"] = "Good" + " (" + dpl_msg + ")"
            except Exception as e:
                print("\nFailed deploying", model.modelname, "model. Skipping to next...", flush=True)
                print("\nFailed deploying", model.modelname, "model with the following error:\n", e, flush=True,
                      file=sys.stderr)
                self.processing_failed[model.modelname]["deploy"] = ''.join(
                    traceback.format_exception(None, e, e.__traceback__))
                traceback.print_exc()
            if "BADGE_DIRECTORY" in os.environ.keys():
                state = self._get_model_failure_state(model.modelname)
                if state & F_LOAD:
                    color = "inactive"
                    text = "not processed"
                elif state & (F_PROC | F_DEPL):
                    color = "red"
                    text = "failed"
                elif state & F_TEST and not state & (F_DEPL | NA_DEPL):
                    color = "yellow"
                    text = "${CI_UPDATE_TARGET_BRANCH}"
                else:
                    color = "green"
                    text = "master"
                git.create_pipeline_badge(self, model.modelname, text, color, target="${BADGE_DIRECTORY}")
        with open(self.faillog, "w") as f:
            f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def relpath(self, path):
        return os.path.relpath(path, self.root)


class TestingPipeline(yaml.YAMLObject):
    def __init__(self, root, configfile):
        self.root = root
        self.configfile = os.path.join(self.root, configfile)
        self.configdir = os.path.dirname(self.configfile)
        if not os.path.isfile(self.configfile):
            raise Exception('{} not found!'.format(self.configfile))

        kwargs = yaml.safe_load(open(configfile, 'r'))['test']

        for (k, v) in kwargs.items():
            setattr(self, k, v)

        print("Working from", self.root, flush=True)
        self.git_rev = git.revision(os.path.abspath(self.root))
        self.git_branch = git.get_branch(os.path.abspath(self.root))
        self.temp_dir = os.path.join(self.root, "temp")
        self.test_protocol = os.path.join(self.temp_dir, "test_protocol.txt")
        self.test_results = {}
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)
        assert (hasattr(self, "model"))
        self.model = TestModel(root=self.root, **self.model)
        self.n_failed_tests = 0
        self.n_done_tests = 0
        self.processing_failed = {self.model.modelname: {}}

        print("Finished reading config", configfile, flush=True)

    def relpath(self, path):
        return os.path.relpath(path, self.root)

    def get_coverage(self):
        return 1 - (self.n_failed_tests / self.n_done_tests)

    def test_models(self):
        """Runs the configured test_routines over all models"""
        failures_ignored_for = []
        print("\nTesting", self.model.modelname, "model...", flush=True)
        print("\nTesting", self.model.modelname, "model...", flush=True, file=sys.stderr)
        try:
            commit_hash = ""
            compare_model = None
            if any(["compare" in x for x in self.tests]):
                assert (hasattr(self, "compare_model") and self.compare_model is not None)
                # Load compare model
                compare_model_path = os.path.join(self.temp_dir, "compare_model")
                print(self.git_branch, str(self.compare_model["branch"]).strip())
                git.clone(
                    self,
                    self.compare_model["git"],
                    compare_model_path,
                    branch=self.compare_model["branch"],
                    recursive=True,
                    ignore_failure=True,
                    commit_id=git.get_previous_commit_hash(self.root) if self.git_branch == self.compare_model[
                        "branch"].strip() else None,
                    shallow=2
                )
                if self.git_rev == git.revision(compare_model_path):
                    git.checkout(git.get_previous_commit_hash(compare_model_path), compare_model_path)
                print("Comparing with compare model at commit", git.revision(compare_model_path), flush=True)
                if "ignore_failing_tests_for" not in self.compare_model.keys():
                    self.compare_model["ignore_failing_tests_for"] = "None"
                if os.path.exists(os.path.join(compare_model_path, self.compare_model["model_in_repo"])):
                    commit_hash = git.revision(compare_model_path)
                    self.compare_model["commit"] = commit_hash[:8]
                    if commit_hash.startswith(str(self.compare_model["ignore_failing_tests_for"])):
                        failures_ignored_for += [self.model.modelname]
                    print("Loading compare model:",
                          self.relpath(os.path.join(compare_model_path, self.compare_model["model_in_repo"])),
                          flush=True)
                    compare_model = CompareModel(
                        name=self.model.modelname,
                        directory=compare_model_path,
                        robotfile=os.path.join(compare_model_path, self.compare_model["model_in_repo"]),
                        submechanisms_file=os.path.join(
                            compare_model_path,
                            self.compare_model["submechanisms_path"]
                            if "submechanisms_path" in self.compare_model
                            else "submechanisms/submechanisms.yml"
                        ) if hasattr(self.model,
                                     "submechanisms_file") and self.model.submechanisms_file is not None else None
                    )
                else:
                    print("WARNING: Compare model not found!", flush=True, file=sys.stderr)

            model_test = ModelTest(self.model, compare_model)
            print("\nRunning info procedures:", flush=True)
            for p in dir(model_test):
                if p.startswith("info_"):
                    getattr(model_test, p)()
            print("\nRunning test procedures:", flush=True)
            self.test_results[self.model.modelname] = {
                "ignore_failure": commit_hash.startswith(
                    str(self.compare_model["ignore_failing_tests_for"])) if compare_model is not None else False,
                "compare_model_present": model_test.old is not None,
                "compare_model": self.compare_model,
            }

            def add_test_result(model_name, test_name, value):
                i = 2
                if test_name in self.test_results[self.model.modelname].keys():
                    while test_name + " " + str(i) in self.test_results[self.model.modelname].keys():
                        i += 1
                    test_name += " " + str(i)
                self.test_results[model_name][test_name] = value
                return value

            # these tests will be run always
            obligatory_tests = ["topological_self_consistency"]
            for otest in obligatory_tests:
                if otest not in self.tests:
                    self.tests += [otest]
            # let's go testing
            for test in self.tests:
                if type(test) is str:
                    print("  -> ", test, flush=True)
                    if not add_test_result(self.model.modelname, test, getattr(model_test, "test_" + test)()):
                        print("Test", test, "failed for", self.model.modelname, flush=True)
                elif type(test) is dict and list(test.keys())[0] == "hyrodynChecks":
                    if not HYRODYN_AVAILABLE:
                        print("Hyrodyn checks not possible, as Hyrodyn couldn't be loaded", flush=True)
                    for htest in test["hyrodynChecks"]:
                        if type(htest) is str:
                            print("  -> ", htest, flush=True)
                            if not add_test_result(self.model.modelname, htest,
                                                   getattr(model_test, "test_hyrodyn_" + htest)()):
                                print("Hyrodyn-Test", htest, "failed for", self.model.modelname, flush=True)
                        elif type(htest) is dict and "move_hyrodyn_model" in htest.keys():
                            k, v = list(htest.items())[0]
                            getattr(model_test, k)(v)
                            print("  -> ", k, flush=True)
                        elif type(htest) is dict:
                            k, v = list(htest.items())[0]
                            print("  -> ", k, flush=True)
                            if not add_test_result(self.model.modelname, k,
                                                   getattr(model_test, "test_hyrodyn_" + k)(v)):
                                print("Hyrodyn-Test", k, "failed for", self.model.modelname, flush=True)
                        else:
                            print("Couldn't process test definition", htest)
                elif type(test) is dict:
                    k, v = list(test.items())[0]
                    print("  -> ", test, flush=True)
                    if not add_test_result(self.model.modelname, k, getattr(model_test, "test_hyrodyn_" + k)(v)):
                        print("Hyrodyn-Test", test, "failed for", self.model.modelname, flush=True)
            self.processing_failed[self.model.modelname]["test"] = "Good"
        except Exception as e:
            print("\nFailed testing", self.model.modelname, "model. Skipping to next...", flush=True)
            print("\nFailed testing", self.model.modelname, "model with the following error:\n", e, flush=True,
                  file=sys.stderr)
            self.processing_failed[self.model.modelname]["test"] = ''.join(
                traceback.format_exception(None, e, e.__traceback__)) + "\n"
            traceback.print_exc()
        test_protocol = {"all": ""}
        test_protocol["all"] = misc.append_string(test_protocol["all"], "----------\nTest protocol:", print=True,
                                                  flush=True)
        success = True
        for modelname, test in self.test_results.items():
            test_protocol[modelname] = ""
            test_protocol[modelname] = misc.append_string(test_protocol[modelname],
                                                          "  " + modelname + " Commit: " + self.git_rev[:8], end="",
                                                          print=True, flush=True)
            if test["ignore_failure"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Ignoring failures)", end="",
                                                              print=True, flush=True)
            if not test["compare_model_present"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Compare model not present):",
                                                              end="", print=True, flush=True)
            test_protocol[modelname] = misc.append_string(test_protocol[modelname], "\n    " + yaml.safe_dump(
                {"Compare Model": test["compare_model"]}, default_flow_style=False, indent=6) + "    Test Results:",
                                                          flush=True, print=True)
            for testname, result in test.items():
                if testname in ["ignore_failure", "compare_model_present", "compare_model", "commit_hash"]:
                    continue
                else:
                    sign = "+"
                    if type(result) is str:
                        sign = "o"
                    elif not result:
                        sign = "-"
                    self.n_done_tests += 1
                    test_protocol[modelname] = misc.append_string(test_protocol[modelname], "    ", sign,
                                                                  testname + ":",
                                                                  str(result) if type(result) is bool else result,
                                                                  print=True, flush=True)
                    if not result and not test["ignore_failure"]:
                        self.n_failed_tests += 1
                        success = False
                        if self.processing_failed[modelname]["test"].upper().startswith("GOOD") or \
                                self.processing_failed[modelname]["test"].upper().startswith("N/A"):
                            self.processing_failed[modelname]["test"] = ""
                        self.processing_failed[modelname][
                            "test"] += "Test " + testname + " failed (and was not ignored)!\n"
                    #     print(" !!! Causes Failure !!!", flush=True)
                    # else:
                    #     print(flush=True)
        print("The test routine", "succeeded!" if success else "failed!", flush=True)
        with open(self.test_protocol, "w") as f:
            f.write(yaml.safe_dump(test_protocol, default_flow_style=False))
        return success


# Todo use inheritance to remove redundant code
class XTypePipeline(yaml.YAMLObject):
    def __init__(self, configfile, processed_model_exists, only_create=False):
        self.processing_failed = {}
        self.test_results = {}
        self.configdir = os.path.dirname(configfile)
        self.processed_model_exists = processed_model_exists
        if not os.path.isfile(configfile):
            raise Exception('{} not found!'.format(configfile))
        self.configfile = configfile
        kwargs = yaml.safe_load(open(configfile, 'r'))['assemble']

        self.modeltypes = {"xtype": kwargs}

        for (k, v) in kwargs.items():
            setattr(self, k, v)

        self.root = os.getcwd()
        print("Working from", self.root, flush=True)
        self.git_rev = git.revision(os.path.abspath(self.configdir))
        self.temp_dir = os.path.join(self.root, "temp")
        self.faillog = os.path.join(self.temp_dir, "failures.txt")
        self.test_protocol = os.path.join(self.temp_dir, "test_protocol.txt")
        if os.path.isfile(self.faillog) and processed_model_exists:
            with open(self.faillog, "r") as f:
                self.processing_failed = yaml.safe_load(f.read())

        self.meshespath = self.meshes[self.output_mesh_format]

        print("Finished reading config", configfile, flush=True)

        self.model = None
        imported_meshes = []

        self.processing_failed = {"load": "N/A", "process": "N/A", "test": "N/A", "deploy": "N/A"}
        try:
            yaml.safe_load(open(configfile, "r").read())["xtype_model"]
        except KeyError as e:
            self.processing_failed["load"] = ''.join(traceback.format_exception(None, e, e.__traceback__))
            print("Could not load model config!")
            traceback.print_exc()
        fstate = self._get_model_failure_state()
        self.processing_failed["load"] = ""
        if bool(fstate & (F_PROC | NA_PROC)) and processed_model_exists:
            print("WARNING: Model was not successfully processed")
            self.processing_failed["load"] += \
                "Model was not successfully processed!\n"
        elif bool(fstate & (F_PROC | NA_PROC)):
            # reset processing error
            self.processing_failed["process"] = "N/A"
        try:
            self.model = XTypeModel(configfile, self, self.processed_model_exists, only_create=only_create)
            self.model.typedef["meshespath"] = self.meshespath
            for m in self.model.get_imported_meshes():
                for m2 in imported_meshes:
                    if os.path.basename(m) == os.path.basename(m2) and os.path.dirname(m) != os.path.dirname(m):
                        raise ("You can not import two different meshes with the same name!\n"
                               "In the model you tried to import:\n  {} but\n  {} already exists!".format(m, m2))
            imported_meshes += self.model.get_imported_meshes()
            self.processing_failed["load"] = "Good"
        except Exception as e:
            self.processing_failed["load"] += "Loading failed with the following error:" + ''.join(
                traceback.format_exception(None, e, e.__traceback__))
            print(self.processing_failed["load"])
        if os.path.exists(os.path.join(self.temp_dir)):
            with open(self.faillog, "w") as f:
                f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def _get_model_failure_state(self):
        state = 0
        if not self.processing_failed["load"].upper().startswith("GOOD") and\
           not self.processing_failed["load"].upper().startswith("N/A"):
            state += F_LOAD
        if not self.processing_failed["process"].upper().startswith("GOOD") and\
           not self.processing_failed["process"].upper().startswith("N/A"):
            state += F_PROC
        if not self.processing_failed["test"].upper().startswith("GOOD") and\
           not self.processing_failed["test"].upper().startswith("N/A"):
            state += F_TEST
        if not self.processing_failed["deploy"].upper().startswith("GOOD") and\
           not self.processing_failed["deploy"].upper().startswith("N/A"):
            state += F_DEPL
        if self.processing_failed["load"].upper().startswith("N/A"):
            state += NA_LOAD
        if self.processing_failed["process"].upper().startswith("N/A"):
            state += NA_PROC
        if self.processing_failed["test"].upper().startswith("N/A"):
            state += NA_TEST
        if self.processing_failed["deploy"].upper().startswith("N/A"):
            state += NA_DEPL
        return state

    def has_failure(self):
        return self._get_model_failure_state() & (F_LOAD | F_PROC | F_TEST | F_DEPL)

    def get_coverage(self, phases=None):
        if phases is None:
            phases = []
        if len(phases) == 0:
            phases = ["process", "test", "deploy"]
        log = yaml.safe_load(open(self.faillog, "r").read())
        all_models = float(len(log.keys()))
        fails = 0.0
        all_models *= len(phases)
        for p in phases:
            fails += not log[p].upper().startswith("GOOD")
        return 1 - fails / all_models

    def print_fail_log(self, file=sys.stdout):
        log = yaml.safe_load(open(self.faillog, "r").read())
        print("\nModel Failure Report:\n---------------------", file=file)
        report = log
        order_val = ["load", "process", "test", "deploy"]
        if len(report) == 0:
            print("Nothing done that could be reported!", file=file)
        print(self.model.modelname + ":", file=file)
        values = [(k, v) for k, v in report.items()]
        values = sorted(values, key=lambda x: order_val.index(x[0]))
        for v in values:
            print("  " + v[0] + ": \t",
                  str(v[1]) if v[1].upper().startswith("GOOD") or v[1].upper().startswith("N/A") else "Error",
                  file=file)
            if not v[1].upper().startswith("GOOD") and not v[1].upper().startswith("N/A"):
                for line in v[1].split("\n"):
                    print("    " + line, file=file)

    def process_models(self):
        # delete the temp_dir if there is already one
        misc.recreate_dir(self, self.temp_dir)
        with open(self.faillog, "w") as f:
            f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

        # create the central mesh folders
        misc.create_dir(self, os.path.join(self.temp_dir, self.meshespath))
        if hasattr(self, "also_export_bobj") and self.also_export_bobj is True:
            misc.create_dir(self, os.path.join(self.temp_dir, self.meshes["bobj"]))
        if hasattr(self, "export_kccd") and self.export_kccd:
            misc.create_dir(self, os.path.join(self.temp_dir, self.meshes["iv"]))
        print("\nProcessing model...", flush=True)
        print("\nProcessing model...", flush=True, file=sys.stderr)
        if self._get_model_failure_state() & (F_LOAD | NA_LOAD):
            self.processing_failed["process"] = "Skipping as it model definition file wasn't loaded successfully!"
            print(self.processing_failed["process"], flush=True)
            print(self.processing_failed["process"], flush=True, file=sys.stderr)
        else:
            try:
                self.model.process()
                self.model.export()
                os.remove(os.path.join(self.model.exportdir, self.model.typedef["meshespath"]))
                os.rename(
                    os.path.join(self.temp_dir, self.model.typedef["meshespath"]),
                    os.path.join(self.model.exportdir, self.model.typedef["meshespath"]))
                if len(os.listdir(os.path.dirname(os.path.join(self.temp_dir, self.model.typedef["meshespath"])))) == 0:
                    os.removedirs(os.path.dirname(os.path.join(self.temp_dir, self.model.typedef["meshespath"])))
                if self.model.typedef["also_export_bobj"] is True:
                    os.remove(os.path.join(self.model.exportdir, self.meshes["bobj"]))
                    os.rename(
                        os.path.join(self.temp_dir, self.meshes["bobj"]),
                        os.path.join(self.model.exportdir, self.meshes["bobj"]))
                    if len(os.listdir(os.path.dirname(os.path.join(self.temp_dir, self.meshes["bobj"])))) == 0:
                        os.removedirs(os.path.dirname(os.path.join(self.temp_dir, self.meshes["bobj"])))
                if hasattr(self.model, "export_kccd") and self.model.export_kccd:
                    os.remove(os.path.join(self.model.exportdir, self.meshes["iv"]))
                    os.rename(
                        os.path.join(self.temp_dir, self.meshes["iv"]),
                        os.path.join(self.model.exportdir, self.meshes["iv"]))
                    if len(os.listdir(os.path.dirname(os.path.join(self.temp_dir, self.meshes["iv"])))) == 0:
                        os.removedirs(os.path.dirname(os.path.join(self.temp_dir, self.meshes["iv"])))
                self.processing_failed["process"] = "Good"
            except Exception as e:
                print("\nFailed processing model. Skipping to next...", flush=True)
                print("\nFailed processing model with the following error:\n", e, flush=True, file=sys.stderr)
                self.processing_failed["process"] = ''.join(traceback.format_exception(None, e, e.__traceback__))
                traceback.print_exc()
        with open(self.faillog, "w") as f:
            f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def test_models(self):
        raise NotImplementedError
        # """Runs the configured test_routines over all models"""
        # with open(self.faillog, "r") as f:
        #     self.processing_failed = yaml.safe_load(f.read())
        # failures_ignored_for = []
        # fstate = self._get_model_failure_state(self.model.modelname)
        # if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
        #     print("\nModel wasn't successfully created. (Code: "+bin(fstate)+")", flush=True)
        #     raise AssertionError("Model wasn't successfully created. (Code: "+bin(fstate)+")",)
        # self.model.processed_model_exists = True
        # self.model._load_robot()
        # print("\nTesting", self.model.modelname, "model...", flush=True)
        # print("\nTesting", self.model.modelname, "model...", flush=True, file=sys.stderr)
        # try:
        #     self.model.recreate_sym_links()
        #     commit_hash = ""
        #     compare_model = None
        #     if any(["compare" in x for x in self.model.typedef["model_tests"]]):
        #         # Load compare model
        #         compare_model_path = os.path.join(self.model.tempdir, "compare_model")
        #         git.clone(
        #             self,
        #             self.model.compare_model["git"],
        #             compare_model_path,
        #             branch=self.model.compare_model["branch"],
        #             recursive=True,
        #             ignore_failure=True
        #         )
        #         if "ignore_failing_tests_for" not in self.model.compare_model.keys():
        #             self.model.compare_model["ignore_failing_tests_for"] = "None"
        #         if os.path.exists(os.path.join(compare_model_path, self.model.compare_model["model_in_repo"])):
        #             commit_hash, _ = misc.executeShellCommand("git rev-parse HEAD", compare_model_path)
        #             if commit_hash.startswith(str(self.model.compare_model["ignore_failing_tests_for"])):
        #                 failures_ignored_for += [self.model.modelname]
        #             print("Loading compare model:", self.relpath(os.path.join(compare_model_path,
        #                   self.model.compare_model["model_in_repo"])), flush=True)
        #             try:
        #                 compare_model = CompareModel(
        #                     name=self.model.robotname,
        #                     directory=compare_model_path,
        #                     robotfile=os.path.join(compare_model_path, self.model.compare_model["model_in_repo"]),
        #                     submechanisms_path=os.path.join(
        #                         compare_model_path,
        #                         self.model.compare_model["submechanisms_in_repo"]
        #                         if "submechanisms_in_repo" in self.model.compare_model
        #                         else "submechanisms/submechanisms.yml"
        #                     ) if hasattr(model, "submechanisms_file") else None
        #                 )
        #             except Exception as e:
        #                 self.model.compare_model["issues"] = repr(e)
        #                 print("Failed to load compare model. Exception was:\n",
        #                       ''.join(traceback.format_exception(None, e, e.__traceback__)) + "\n")
        #                 traceback.print_exc()
        #         else:
        #             print("WARNING: Compare model not found!", flush=True, file=sys.stderr)
        #
        #     model_test = ModelTest(self.model, compare_model)
        #     print("\nRunning info procedures:", flush=True)
        #     for p in dir(model_test):
        #         if p.startswith("info_"):
        #             getattr(model_test, p)()
        #     print("\nRunning test procedures:", flush=True)
        #     self.test_results[self.model.modelname] = {
        #         "ignore_failure": commit_hash.startswith(str(self.model.compare_model["ignore_failing_tests_for"]))
        #         if compare_model is not None else False,
        #         "compare_model_present": model_test.old is not None,
        #         "compare_model": self.model.compare_model
        #     }
        #
        #     def add_test_result(modelname, testname, value):
        #         i = 2
        #         if testname in self.test_results[self.model.modelname].keys():
        #             while testname+" "+str(i) in self.test_results[self.model.modelname].keys():
        #                 i += 1
        #             testname += " "+str(i)
        #         self.test_results[modelname][testname] = value
        #         return value
        #     # these tests will be run always
        #     obligatory_tests = ["topological_self_consistency"]
        #     for otest in obligatory_tests:
        #         if otest not in self.model.typedef["model_tests"]:
        #             self.model.typedef["model_tests"] += [otest]
        #     # let's go testing
        #     for test in self.model.typedef["model_tests"]:
        #         if type(test) is str:
        #             print("  -> ", test, flush=True)
        #             if not add_test_result(self.model.modelname, test, getattr(model_test, "test_"+test)()):
        #                 print("Test", test, "failed for", self.model.modelname, flush=True)
        #         elif type(test) is dict and list(test.keys())[0] == "hyrodynChecks":
        #             if not HYRODYN_AVAILABLE:
        #                 print("Hyrodyn checks not possible, as Hyrodyn couldn't be loaded", flush=True)
        #             for htest in test["hyrodynChecks"]:
        #                 if type(htest) is str:
        #                     print("  -> ", htest, flush=True)
        #                     if not add_test_result(self.model.modelname, htest,
        #                                            getattr(model_test, "test_hyrodyn_"+htest)()):
        #                         print("Hyrodyn-Test", htest, "failed for", self.model.modelname, flush=True)
        #                 elif type(htest) is dict and "move_hyrodyn_model" in htest.keys():
        #                     k, v = list(htest.items())[0]
        #                     getattr(model_test, k)(v)
        #                     print("  -> ", k, flush=True)
        #                 elif type(htest) is dict:
        #                     k, v = list(htest.items())[0]
        #                     print("  -> ", k, flush=True)
        #                     if not add_test_result(self.model.modelname, k,
        #                                            getattr(model_test, "test_hyrodyn_"+k)(v)):
        #                         print("Hyrodyn-Test", k, "failed for", self.model.modelname, flush=True)
        #                 else:
        #                     print("Couldn't process test definition", htest)
        #         elif type(test) is dict:
        #             k, v = list(test.items())[0]
        #             print("  -> ", test, flush=True)
        #             if not add_test_result(self.model.modelname, k, getattr(model_test, "test_hyrodyn_" + k)(v)):
        #                 print("Hyrodyn-Test", test, "failed for", self.model.modelname, flush=True)
        #     self.processing_failed[self.model.modelname]["test"] = "Good"
        # except Exception as e:
        #     print("\nFailed testing", self.model.modelname, "model. Skipping to next...", flush=True)
        #     print("\nFailed testing", self.model.modelname, "model with the following error:\n", e, flush=True,
        #           file=sys.stderr)
        #     self.processing_failed[self.model.modelname]["test"] = ''.join(traceback.format_exception(None, e,
        #     e.__traceback__)) + "\n"
        #     traceback.print_exc()
        # test_protocol = {"all": ""}
        # test_protocol["all"] = misc.appendString(test_protocol["all"], "----------\nTest protocol:",
        #                                          print=True, flush=True)
        # success = True
        # for modelname, test in self.test_results.items():
        #     test_protocol[modelname] = ""
        #     test_protocol[modelname] = misc.appendString(test_protocol[modelname], "  "+modelname, end="",
        #                                                  print=True, flush=True)
        #     if test["ignore_failure"]:
        #         test_protocol[modelname] = misc.appendString(test_protocol[modelname], " (Ignoring failures)",
        #                                                      end="", print=True, flush=True)
        #     if not test["compare_model_present"]:
        #         test_protocol[modelname] = misc.appendString(test_protocol[modelname],
        #                                                      " (Compare model not present):",
        #                                                      end="", print=True, flush=True)
        #     test_protocol[modelname] = misc.appendString(
        #         test_protocol[modelname], "\n    "+yaml.safe_dump({"Compare Model": test["compare_model"]},
        #         default_flow_style=False, indent=6)+"    Test Results:", flush=True, print=True)
        #     for testname, result in test.items():
        #         if testname in ["ignore_failure", "compare_model_present", "compare_model"]:
        #             continue
        #         else:
        #             sign = "+"
        #             if type(result) is str:
        #                 sign = "o"
        #             elif not result:
        #                 sign = "-"
        #             test_protocol[modelname] = misc.appendString(
        #                 test_protocol[modelname], "    ", sign, testname+":",
        #                 str(result) if type(result) is bool else result, print=True, flush=True)
        #             if not result and not test["ignore_failure"]:
        #                 success = False
        #                 if self.processing_failed[modelname]["test"].upper().startswith("GOOD") or\
        #                    self.processing_failed[modelname]["test"].upper().startswith("N/A"):
        #                     self.processing_failed[modelname]["test"] = ""
        #                 self.processing_failed[modelname]["test"] += "Test " + testname + ""
        #                     "failed (and was not ignored)!\n"
        #             #     print(" !!! Causes Failure !!!", flush=True)
        #             # else:
        #             #     print(flush=True)
        #     with open(self.faillog, "w") as f:
        #         f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))
        # print("The test routine", "succeeded!" if success else "failed!", flush=True)
        # with open(self.test_protocol, "w") as f:
        #     f.write(yaml.safe_dump(test_protocol, default_flow_style=False))
        # return success

    def deploy_models(self):
        raise NotImplementedError
        # """Moves everything from the temp to the repositories and pushes it to the repos"""
        # with open(self.faillog, "r") as f:
        #     self.processing_failed = yaml.safe_load(f.read())
        # mesh_repos = {}
        # # copying the files to the repos
        # uses_lfs = False
        # for name, path in self.meshes.items():
        #     print("\nDeploying meshes", path, flush=True)
        #     print("\nDeploying meshes", path, flush=True, file=sys.stderr)
        #     repo = os.path.join(self.root, path)
        #     git.update(repo, update_target_branch="$CI_MESH_UPDATE_TARGET_BRANCH")
        #     if os.path.basename(path).lower().startswith("lfs"):
        #         uses_lfs = True
        #         git.installLfs(repo, track=["*.obj",  "*.stl",  "*.dae", "*.bobj", "*.iv", "*.mtl",
        #                                     "*.OBJ",  "*.STL",  "*.DAE", "*.BOBJ", "*.IV", "*.MTL"])
        #     git.clearRepo(repo)
        #     misc.copy(self, os.path.join(self.temp_dir, path) + "/*", os.path.join(self.root, path))
        #     commit_hash = git.commit(repo, origin_repo=os.path.abspath(self.configdir))
        #     mesh_repos.update({
        #         name: {
        #             "repo": repo,
        #             "commit": commit_hash
        #         }
        #     })
        #     git.addRemote(repo, self.remote_base+"/"+path)
        #     # mr = git.MergeRequest()
        #     # mr.target = self.mr_target_branch
        #     # mr.title = self.mr_title
        #     # mr.description = self.mr_description
        #     # if hasattr(self, "mr_mention"):
        #     #     mr.mention = self.mr_mention
        #     # for m in self.models:
        #     #     if hasattr(m, "mr_mention"):
        #     #         mr.mention += " "+m.mr_mention
        #     # git.push(repo, merge_request=mr)
        #     git.push(repo, branch="$CI_MESH_UPDATE_TARGET_BRANCH")
        # for model in self.models:
        #     fstate = self._get_model_failure_state(model.modelname)
        #     if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
        #         print("\nSkipping", model.modelname, "model as it wasn't successfully created."
        #               " (Code: "+bin(fstate)+")", flush=True)
        #         print("\nSkipping", model.modelname, "model as it wasn't successfully created."
        #               " (Code: "+bin(fstate)+")", flush=True, file=sys.stderr)
        #         continue
        #     print("\nDeploying", model.modelname, "model...", flush=True)
        #     print("\nDeploying", model.modelname, "model...", flush=True, file=sys.stderr)
        #     try:
        #         model.deploy(mesh_repos, uses_lfs=uses_lfs,
        #                      failed_model=bool(fstate & F_TEST) or bool(fstate & NA_TEST))
        #         self.processing_failed[model.modelname]["deploy"] = "Good" + " (pushed to " +
        #                                                             ("develop" if bool(fstate & F_TEST) else "master")
        #                                                             + ")"
        #     except Exception as e:
        #         print("\nFailed deploying", model.modelname, "model. Skipping to next...", flush=True)
        #         print("\nFailed deploying", model.modelname, "model with the following error:\n", e,
        #               flush=True, file=sys.stderr)
        #         self.processing_failed[model.modelname]["deploy"] = ''.join(
        #           traceback.format_exception(None, e,e.__traceback__))
        #         traceback.print_exc()
        # with open(self.faillog, "w") as f:
        #     f.write(yaml.safe_dump(self.processing_failed, default_flow_style=False))

    def relpath(self, path):
        return os.path.relpath(path, self.root)
