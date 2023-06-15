import os.path
import sys
import traceback

import yaml

from .base_model import BaseModel
from .compare_model import CompareModel
from .model_testing import ModelTest
from .test_model import TestModel
from ..commandline_logging import get_logger
from ..defs import *
from ..utils import git, misc

log = get_logger(__name__)

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
    def __init__(self, configfile, processed_model_exists, subclass=False):
        self.processing_failed = {}
        self.test_results = {}
        self.configdir = os.path.abspath(os.path.dirname(configfile))
        self.processed_model_exists = processed_model_exists
        if not os.path.isfile(configfile):
            raise Exception('{} not found!'.format(configfile))

        self.root = ".."
        self.model_definitions = []
        kwargs = load_json(open(configfile, 'r'))['pipeline']
        for (k, v) in kwargs.items():
            setattr(self, k, v)

        self.temp_dir = os.path.join(self.root, "temp")
        self.faillog = os.path.join(self.temp_dir, "failures.txt")
        self.test_protocol = os.path.join(self.temp_dir, "test_protocol.txt")

        if not subclass:
            assert hasattr(self, "model_definitions") and len(self.model_definitions) > 0
            assert hasattr(self, "root") and self.root is not None and len(self.root) > 0
    
            self.root = os.path.abspath(os.path.join(self.configdir, self.root))
            log.info(f"Working from {self.root}")
            self.git_rev = git.revision(os.path.abspath(self.configdir))
            self.temp_dir = os.path.join(self.root, "temp")
            self.faillog = os.path.join(self.temp_dir, "failures.txt")
            self.test_protocol = os.path.join(self.temp_dir, "test_protocol.txt")
            if os.path.isfile(self.faillog) and processed_model_exists:
                with open(self.faillog, "r") as f:
                    self.processing_failed = load_json(f.read())
    
            log.debug(f"Models to process: {self.model_definitions}")
            log.debug(f"Finished reading config {configfile}")
    
            self.models = []
            input_meshes = []
            order = 0
            for md in self.model_definitions:
                order += 1
                log.info(f"Loading {md} ...")
                if md[:-4] not in self.processing_failed.keys():
                    self.processing_failed[md[:-4]] = {"load": "N/A", "process": "N/A", "test": "N/A", "deploy": "N/A",
                                                       "order": order}
                try:
                    _file = os.path.join(self.configdir, md)
                    if not os.path.isfile(_file):
                        raise AssertionError(f"File {_file} does not exist!")
                    cfg = load_json(open(_file, "r").read())
                    cfg_ = cfg[list(cfg.keys())[0]]
                    if "input_models" not in cfg_.keys():
                        raise AssertionError(f"{_file} lacks input_models key!")
                except Exception as e:
                    self.processing_failed[md[:-4]]["load"] = ''.join(traceback.format_exception(None, e, e.__traceback__))
                    log.error("Could not load model config " + md)
                    traceback.print_exc()
                    continue
                fstate = self._get_model_failure_state(md[:-4])
                self.processing_failed[md[:-4]]["load"] = ""
                if bool(fstate & (F_PROC | NA_PROC)) and processed_model_exists:
                    log.warning("Skipping model " + md + " as it was not successfully processed")
                    self.processing_failed[md[:-4]]["load"] += \
                        "Skipping model " + md + " as it was not successfully processed!\n"
                    continue
                elif bool(fstate & (F_PROC | NA_PROC)):
                    # reset processing error
                    self.processing_failed[md[:-4]]["process"] = "N/A"
                if any(["derived_base" in v.keys() and self._get_model_failure_state(
                        v["derived_base"][:-4]) & (F_LOAD | F_PROC) for _, v in cfg_["input_models"].items()]):
                    log.warning("Skipping model " + md + " as at least one parent model was not successfully processed!")
                    self.processing_failed[md[:-4]]["load"] += \
                        "Skipping model " + md + " as at least one parent model was not successfully processed!\n"
                    continue
                try:
                    if list(cfg.keys())[0] == "model":
                        self.models += [BaseModel(os.path.join(self.configdir, md), self, self.processed_model_exists)]
                    else:
                        self.processing_failed[md[:-4]]["load"] = f"Skipping {md} as it is no valid model definition!\n"
                        log.error(self.processing_failed[md[:-4]]["load"])
                        continue
                    for m in self.models[-1].get_input_meshes():
                        for m2 in input_meshes:
                            if os.path.basename(m) == os.path.basename(m2) and os.path.dirname(m) != os.path.dirname(m):
                                raise Exception(
                                    "You can not import two different meshes with the same name!\n"
                                    "In model {} you tried to import:\n  {} but\n  {} already exists!".format(md, m, m2)
                                )
                    input_meshes += self.models[-1].get_input_meshes()
                    self.processing_failed[md[:-4]]["load"] = "Good"
                except Exception as e:
                    self.processing_failed[md[:-4]]["load"] += "Loading " + md + " failed with the following error:" +\
                                                               ''.join(traceback.format_exception(None, e, e.__traceback__))
                    log.error(self.processing_failed[md[:-4]]["load"])
                    continue
            if os.path.exists(os.path.join(self.temp_dir)):
                with open(self.faillog, "w") as f:
                    f.write(dump_json(self.processing_failed, default_flow_style=False))

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
        failures = [self._get_model_failure_state(mn[:-4]) &
                    (F_LOAD | F_PROC | F_TEST | F_DEPL | NA_LOAD | NA_PROC | NA_TEST | NA_DEPL)
                    for mn in self.model_definitions]
        return len([x for x in failures if x])

    def has_failure(self):
        return any([self._get_model_failure_state(mn[:-4]) & (F_LOAD | F_PROC | F_TEST | F_DEPL) for mn in
                    self.model_definitions])

    def get_coverage(self, phases=None, allow_na=False):
        if phases is None:
            phases = []
        if len(phases) == 0:
            phases = ["process", "test", "deploy"]
        log = load_json(open(self.faillog, "r").read())
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
        log = load_json(open(self.faillog, "r").read())
        print("\nPipeline Report:\n---------------------", file=file)
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
            f.write(dump_json(self.processing_failed, default_flow_style=False))

        # create the central mesh folders
        export_meshes = {}
        for model in self.models:
            export_meshes = misc.merge_default(model.export_meshes, export_meshes)
        for mt, mp in export_meshes.items():
            misc.create_dir(self, os.path.join(self.temp_dir, str(mp)))
            ext = mt
            misc.copy(self, os.path.join(self.root, str(mp), "*."+ext), os.path.join(self.temp_dir, str(mp)))
        processed_meshes = set()
        for model in self.models:
            log.info(f"\nProcessing {model.modelname} model...")
            if self._get_model_failure_state(model.modelname) & (F_LOAD | NA_LOAD):
                self.processing_failed[model.modelname]["process"] = \
                    "Skipping ", model.modelname, " as it model definition file wasn't loaded successfully!"
                log.info(self.processing_failed[model.modelname]["process"])
                continue
            try:
                model.process()
                model.export()
                processed_meshes = processed_meshes.union(model.processed_meshes)
                self.processing_failed[model.modelname]["process"] = "Good"
            except Exception as e:
                log.error(f"\nFailed processing {model.modelname} model with the following error and skipped to next:\n {e}")
                self.processing_failed[model.modelname]["process"] = ''.join(
                    traceback.format_exception(None, e, e.__traceback__))
                traceback.print_exc()
        # Remove all mesh files we have initially copied but that haven't been processed
        existing_meshes = []
        for mt, mp in export_meshes.items():
            # misc.create_dir(self, os.path.join(self.temp_dir, str(mp)))
            existing_meshes += misc.list_files(os.path.join(self.temp_dir, str(mp)),
                                               ignore=["\.gv", "\.pdf", "\.git*", "\_history.log", "README\.md", "manifest\.xml"],
                                               resolve_symlinks=True, abs_path=True)
        existing_meshes = set(existing_meshes)
        processed_meshes = set(processed_meshes)
        for unused_mesh in existing_meshes - processed_meshes:
            log.debug("Removing unused mesh: "+unused_mesh)
            assert os.path.isfile(unused_mesh)
            os.remove(unused_mesh)
        with open(self.faillog, "w") as f:
            f.write(dump_json(self.processing_failed, default_flow_style=False))

    def test_models(self):
        """Runs the configured test_routines over all models"""
        with open(self.faillog, "r") as f:
            self.processing_failed = load_json(f.read())
        failures_ignored_for = []
        for model in self.models:
            fstate = self._get_model_failure_state(model.modelname)
            if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
                log.warning(f"\nSkipping {model.modelname} model as it wasn't successfully created."
                            f"(Code: " + bin(fstate) + ")")
                continue
            model.processed_model_exists = True
            model._load_robot()
            log.info(f"\nTesting {model.modelname} model...")
            print(f"\nTesting {model.modelname} model...")
            try:
                model.recreate_sym_links()
                commit_hash = ""
                compare_model = None
                if any(["compare" in x for x in model.test["tests"]]) and type(model.test["compare_model"]) is dict:
                    # Load compare model
                    compare_model_path = os.path.join(model.tempdir, "compare_model")
                    git.clone(
                        pipeline=self,
                        repo=model.test["compare_model"]["git"],
                        target=compare_model_path,
                        branch=model.test["compare_model"]["branch"],
                        recursive=True,
                        ignore_failure=False
                    )
                    if "ignore_failing_tests_for" not in model.test["compare_model"].keys():
                        model.test["compare_model"]["ignore_failing_tests_for"] = "None"
                    if os.path.exists(os.path.join(compare_model_path, model.test["compare_model"]["model_in_repo"])):
                        commit_hash, _ = misc.execute_shell_command("git rev-parse HEAD", compare_model_path)
                        if commit_hash.startswith(str(model.test["compare_model"]["ignore_failing_tests_for"])):
                            failures_ignored_for += [model.modelname]
                        log.debug(
                            f"Loading compare model: "
                            f"{self.relpath(os.path.join(compare_model_path, model.test['compare_model']['model_in_repo']))}"
                        )
                        try:
                            compare_model = CompareModel(
                                name=model.robotname,
                                directory=compare_model_path,
                                robotfile=os.path.join(compare_model_path, model.test["compare_model"]["model_in_repo"]),
                                submechanisms_file=os.path.join(
                                    compare_model_path,
                                    model.test["compare_model"]["submechanisms_in_repo"]
                                ) if "submechanisms_in_repo" in model.test["compare_model"] and model.test["compare_model"]["submechanisms_in_repo"] is not None else None
                            )
                        except Exception as e:
                            model.test["compare_model"]["issues"] = repr(e)
                            log.error("Failed to load compare model. Exception was:\n" +
                                      ''.join(traceback.format_exception(None, e, e.__traceback__)) + "\n")
                            traceback.print_exc()
                    else:
                        log.warning("Compare model not found!")

                model_test = ModelTest(model, compare_model)
                log.info("\nRunning info procedures:")
                for p in dir(model_test):
                    if p.startswith("info_"):
                        getattr(model_test, p)()
                log.info("\nRunning test procedures:")
                self.test_results[model.modelname] = {
                    "ignore_failure": commit_hash.startswith(
                        str(model.test["compare_model"]["ignore_failing_tests_for"])) if compare_model is not None else False,
                    "compare_model_present": model_test.old is not None,
                    "compare_model": model.test["compare_model"]
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
                obligatory_tests = ["topological_self_consistency", "file_consistency"]
                for otest in obligatory_tests:
                    if otest not in model.test["tests"]:
                        model.test["tests"] += [otest]
                # let's go testing
                for test in model.test["tests"]:
                    if type(test) is str:
                        print("  -> ", test)
                        if not add_test_result(model.modelname, test, getattr(model_test, "test_" + test)()):
                            print("Test", test, "failed for", model.modelname)
                    elif type(test) is dict and list(test.keys())[0] == "hyrodynChecks":
                        if not HYRODYN_AVAILABLE:
                            print("Hyrodyn checks not possible, as Hyrodyn couldn't be loaded")
                        for htest in test["hyrodynChecks"]:
                            if type(htest) is str:
                                print("  -> ", htest)
                                if not add_test_result(model.modelname, htest,
                                                       getattr(model_test, "test_hyrodyn_" + htest)()):
                                    print("Hyrodyn-Test", htest, "failed for", model.modelname)
                            elif type(htest) is dict and "move_hyrodyn_model" in htest.keys():
                                k, v = list(htest.items())[0]
                                getattr(model_test, k)(v)
                                print("  -> ", k)
                            elif type(htest) is dict:
                                k, v = list(htest.items())[0]
                                print("  -> ", k)
                                if not add_test_result(model.modelname, k, getattr(model_test, "test_hyrodyn_" + k)(v)):
                                    print("Hyrodyn-Test", k, "failed for", model.modelname)
                            else:
                                log.error(f"Couldn't process test definition {htest}")
                    elif type(test) is dict:
                        k, v = list(test.items())[0]
                        log.info(f"  -> {test}")
                        if not add_test_result(model.modelname, k, getattr(model_test, "test_" + k)(v)):
                            log.error(f"Hyrodyn-Test {test} failed for {model.modelname}")
                self.processing_failed[model.modelname]["test"] = "Good"
            except Exception as e:
                log.error(f"\nFailed testing {model.modelname} model with the following error and skipped to next:\n {e}")
                self.processing_failed[model.modelname]["test"] = ''.join(
                    traceback.format_exception(None, e, e.__traceback__)) + "\n"
                traceback.print_exc()
        test_protocol = {"all": ""}
        test_protocol["all"] = misc.append_string(test_protocol["all"], "----------\nTest protocol:", print=True)
        success = True
        for modelname, test in self.test_results.items():
            test_protocol[modelname] = ""
            test_protocol[modelname] = misc.append_string(test_protocol[modelname], "  " + modelname, end="",
                                                          print=True)
            if test["ignore_failure"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Ignoring failures)", end="",
                                                              print=True)
            if not test["compare_model_present"]:
                test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Compare model not present):",
                                                              end="", print=True)
            test_protocol[modelname] = misc.append_string(test_protocol[modelname], "\n    " + dump_yaml(
                {"Compare Model": test["compare_model"]}, default_flow_style=False, indent=6) + "    Test Results:",
                                                          print=True)
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
                                                                  print=True)
                    if not result and not test["ignore_failure"]:
                        success = False
                        if self.processing_failed[modelname]["test"].upper().startswith("GOOD") or \
                                self.processing_failed[modelname]["test"].upper().startswith("N/A"):
                            self.processing_failed[modelname]["test"] = ""
                        self.processing_failed[modelname][
                            "test"] += "Test " + testname + " failed (and was not ignored)!\n"
                    #     print(" !!! Causes Failure !!!")
                    # else:
                    #     print(flush=True)
            with open(self.faillog, "w") as f:
                f.write(dump_json(self.processing_failed, default_flow_style=False))
        print("The test routine", "succeeded!" if success else "failed!")
        with open(self.test_protocol, "w") as f:
            f.write(dump_json(test_protocol, default_flow_style=False))
        return success

    def deploy_models(self):
        """Moves everything from the temp to the repositories and pushes it to the repos"""
        with open(self.faillog, "r") as f:
            self.processing_failed = load_json(f.read())
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
            log.info(f"\nDeploying meshes... {path}")
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
            git.push(repo, branch="$CI_MESH_UPDATE_TARGET_BRANCH")
        for model in self.models:
            fstate = self._get_model_failure_state(model.modelname)
            if bool(fstate & (F_LOAD | F_PROC | NA_LOAD | NA_PROC)):
                log.warning(f"\nSkipping {model.modelname} model as it wasn't successfully created."
                            f"(Code: " + bin(fstate) + ")")
                continue
            log.info(f"\nDeploying {model.modelname} model...")
            try:
                dpl_msg = model.deploy(mesh_repos, uses_lfs=uses_lfs,
                                       failed_model=bool(fstate & F_TEST) or bool(fstate & NA_TEST))
                self.processing_failed[model.modelname]["deploy"] = "Good" + " (" + dpl_msg + ")"
            except Exception as e:
                log.error("\nFailed deploying {model.modelname} model with the following error and skipped to next:\n {e}")
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
            f.write(dump_json(self.processing_failed, default_flow_style=False))

    def relpath(self, path):
        return os.path.relpath(path, self.root)


class TestingPipeline(Pipeline):
    def __init__(self, root, configfile, place_temp_in=None):
        super(TestingPipeline, self).__init__(configfile, processed_model_exists=True, subclass=True)
        self.root = root
        self.configfile = os.path.join(self.root, configfile)
        self.configdir = os.path.dirname(self.configfile)
        self.processed_model_exists = True
        if not os.path.isfile(self.configfile):
            raise Exception('{} not found!'.format(self.configfile))

        print("Working from", self.root)
        self.git_rev = git.revision(os.path.abspath(self.root))
        self.git_branch = git.get_branch(os.path.abspath(self.root))
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)
        assert (hasattr(self, "model"))
        load_failed = False
        self.models = [TestModel(root=self.root, test=self.test, tempdir=self.temp_dir, **self.model)]
        try:
            self.models[0]._load_robot()
        except Exception as e:
            load_failed = e
            log.error(f"Failed loading model:\n{e}")
        self.n_failed_tests = 0
        self.n_done_tests = 0

        print("Finished reading config", configfile)

        self.processing_failed = {self.models[0].modelname: {
            "load": "GOOD" if not load_failed else "ERROR:\n"+str(load_failed),
            "test": "N/A", "order": 0}
        }

        if os.path.exists(os.path.join(self.temp_dir)):
            with open(self.faillog, "w") as f:
                f.write(dump_json(self.processing_failed, default_flow_style=False))

    def relpath(self, path):
        return os.path.relpath(path, self.root)

    def get_coverage(self, phases=None):
        return super(TestingPipeline, self).get_coverage(["test"])

    # def test_models(self):
    #     """Runs the configured test_routines over all models"""
    #     failures_ignored_for = []
    #     log.info(f"\nTesting {self.model.modelname} model...")
    #     try:
    #         commit_hash = ""
    #         compare_model = None
    #         if any(["compare" in x for x in self.tests]):
    #             if not (hasattr(self, "compare_model") and self.compare_model is not None):
    #                 log.error("Giving compare model is mandatory, e.g.specify the previous version in this repo.")
    #                 sys.exit(1)
    #             if "model_in_repo" not in self.compare_model:
    #                 self.compare_model["model_in_repo"] = self.model.model_in_repo
    #             if "submechanisms_in_repo" not in self.compare_model:
    #                 self.compare_model["submechanisms_in_repo"] = self.model.submechanisms_in_repo
    #             # Load compare model
    #             compare_model_path = os.path.join(self.temp_dir, "compare_model")
    #             log.info(f"{self.git_branch}, {str(self.compare_model['branch']).strip()}")
    #             git.clone(
    #                 self,
    #                 self.compare_model["git"],
    #                 compare_model_path,
    #                 branch=self.compare_model["branch"],
    #                 recursive=True,
    #                 ignore_failure=True,
    #                 commit_id=git.get_previous_commit_hash(self.root) if self.git_branch == self.compare_model[
    #                     "branch"].strip() else None,
    #                 shallow=2
    #             )
    #             if self.git_rev == git.revision(compare_model_path):
    #                 git.checkout(git.get_previous_commit_hash(compare_model_path), compare_model_path)
    #             log.info(f"Comparing with compare model at commit {git.revision(compare_model_path)}")
    #             if "ignore_failing_tests_for" not in self.compare_model.keys():
    #                 self.compare_model["ignore_failing_tests_for"] = "None"
    #             if os.path.exists(os.path.join(compare_model_path, self.compare_model["model_in_repo"])):
    #                 commit_hash = git.revision(compare_model_path)
    #                 self.compare_model["commit"] = commit_hash[:8]
    #                 if commit_hash.startswith(str(self.compare_model["ignore_failing_tests_for"])):
    #                     failures_ignored_for += [self.model.modelname]
    #                 log.info(f"Loading compare model: {self.relpath(os.path.join(compare_model_path, self.compare_model['model_in_repo']))}")
    #                 compare_model = CompareModel(
    #                     name=self.model.modelname,
    #                     directory=compare_model_path,
    #                     robotfile=os.path.join(compare_model_path, self.compare_model["model_in_repo"]),
    #                     submechanisms_file=os.path.join(
    #                         compare_model_path,
    #                         self.compare_model["submechanisms_path"]
    #                         if "submechanisms_path" in self.compare_model
    #                         else "submechanisms/submechanisms.yml"
    #                     ) if hasattr(self.model,
    #                                  "submechanisms_file") and self.model.submechanisms_file is not None else None
    #                 )
    #             else:
    #                 log.warning("Compare model not found!")
    #
    #         model_test = ModelTest(self.model, compare_model)
    #         log.info("\nRunning info procedures:")
    #         for p in dir(model_test):
    #             if p.startswith("info_"):
    #                 getattr(model_test, p)()
    #         log.info("\nRunning test procedures:")
    #         self.test_results[self.model.modelname] = {
    #             "ignore_failure": commit_hash.startswith(
    #                 str(self.compare_model["ignore_failing_tests_for"])) if compare_model is not None else False,
    #             "compare_model_present": model_test.old is not None,
    #             "compare_model": self.compare_model,
    #         }
    #
    #         def add_test_result(model_name, test_name, value):
    #             i = 2
    #             if test_name in self.test_results[self.model.modelname].keys():
    #                 while test_name + " " + str(i) in self.test_results[self.model.modelname].keys():
    #                     i += 1
    #                 test_name += " " + str(i)
    #             self.test_results[model_name][test_name] = value
    #             return value
    #
    #         # these tests will be run always
    #         obligatory_tests = ["topological_self_consistency"]
    #         for otest in obligatory_tests:
    #             if otest not in self.tests:
    #                 self.tests += [otest]
    #         # let's go testing
    #         for test in self.tests:
    #             if type(test) is str:
    #                 log.info("  -> {test}")
    #                 if not add_test_result(self.model.modelname, test, getattr(model_test, "test_" + test)()):
    #                     log.error(f"Test {test} failed for {self.model.modelname}")
    #             elif type(test) is dict and list(test.keys())[0] == "hyrodynChecks":
    #                 if not HYRODYN_AVAILABLE:
    #                     log.warning("Hyrodyn checks not possible, as Hyrodyn couldn't be loaded")
    #                 for htest in test["hyrodynChecks"]:
    #                     if type(htest) is str:
    #                         log.info(f"  -> {htest}")
    #                         if not add_test_result(self.model.modelname, htest,
    #                                                getattr(model_test, "test_hyrodyn_" + htest)()):
    #                             log.error(f"Hyrodyn-Test {htest} failed for {self.model.modelname}")
    #                     elif type(htest) is dict and "move_hyrodyn_model" in htest.keys():
    #                         k, v = list(htest.items())[0]
    #                         getattr(model_test, k)(v)
    #                         log.info(f"  -> {k}")
    #                     elif type(htest) is dict:
    #                         k, v = list(htest.items())[0]
    #                         log.info(f"  -> {k}")
    #                         if not add_test_result(self.model.modelname, k,
    #                                                getattr(model_test, "test_hyrodyn_" + k)(v)):
    #                             log.error(f"Hyrodyn-Test {k} failed for {self.model.modelname}")
    #                     else:
    #                         log.error(f"Couldn't process test definition {htest}")
    #             elif type(test) is dict:
    #                 k, v = list(test.items())[0]
    #                 log.info(f"  -> {test}")
    #                 if not add_test_result(self.model.modelname, k, getattr(model_test, "test_hyrodyn_" + k)(v)):
    #                     log.error(f"Hyrodyn-Test {test} failed for {self.model.modelname}")
    #         self.processing_failed[self.model.modelname]["test"] = "Good"
    #     except Exception as e:
    #         log.error(f"\nFailed testing {self.model.modelname} model with the following error and skipped to next:\n {e}")
    #         self.processing_failed[self.model.modelname]["test"] = ''.join(
    #             traceback.format_exception(None, e, e.__traceback__)) + "\n"
    #         traceback.print_exc()
    #     test_protocol = {"all": ""}
    #     test_protocol["all"] = misc.append_string(test_protocol["all"], "----------\nTest protocol:", print=True,
    #                                               flush=True)
    #     success = True
    #     for modelname, test in self.test_results.items():
    #         test_protocol[modelname] = ""
    #         test_protocol[modelname] = misc.append_string(test_protocol[modelname],
    #                                                       "  " + modelname + " Commit: " + self.git_rev[:8], end="",
    #                                                       print=True)
    #         if test["ignore_failure"]:
    #             test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Ignoring failures)", end="",
    #                                                           print=True)
    #         if not test["compare_model_present"]:
    #             test_protocol[modelname] = misc.append_string(test_protocol[modelname], " (Compare model not present):",
    #                                                           end="", print=True)
    #         test_protocol[modelname] = misc.append_string(test_protocol[modelname], "\n    " + dump_yaml(
    #             {"Compare Model": test["compare_model"]}, default_flow_style=False, indent=6) + "    Test Results:",
    #                                                       flush=True, print=True)
    #         for testname, result in test.items():
    #             if testname in ["ignore_failure", "compare_model_present", "compare_model", "commit_hash"]:
    #                 continue
    #             else:
    #                 sign = "+"
    #                 if type(result) is str:
    #                     sign = "o"
    #                 elif not result:
    #                     sign = "-"
    #                 self.n_done_tests += 1
    #                 test_protocol[modelname] = misc.append_string(test_protocol[modelname], "    ", sign,
    #                                                               testname + ":",
    #                                                               str(result) if type(result) is bool else result,
    #                                                               print=True)
    #                 if not result and not test["ignore_failure"]:
    #                     self.n_failed_tests += 1
    #                     success = False
    #                     if self.processing_failed[modelname]["test"].upper().startswith("GOOD") or \
    #                             self.processing_failed[modelname]["test"].upper().startswith("N/A"):
    #                         self.processing_failed[modelname]["test"] = ""
    #                     self.processing_failed[modelname][
    #                         "test"] += "Test " + testname + " failed (and was not ignored)!\n"
    #                 #     print(" !!! Causes Failure !!!")
    #                 # else:
    #                 #     print(flush=True)
    #     print("The test routine", "succeeded!" if success and self.n_done_tests > 0 else "failed!")
    #     with open(self.test_protocol, "w") as f:
    #         f.write(dump_json(test_protocol, default_flow_style=False))
    #     return success

    def _get_model_failure_state(self, modelname):
        state = 0
        if modelname not in self.processing_failed:
            return state
        if not self.processing_failed[modelname]["load"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["load"].upper().startswith("N/A"):
            state += F_LOAD
        if not self.processing_failed[modelname]["test"].upper().startswith("GOOD") and not \
                self.processing_failed[modelname]["test"].upper().startswith("N/A"):
            state += F_TEST
        if self.processing_failed[modelname]["load"].upper().startswith("N/A"):
            state += NA_LOAD
        if self.processing_failed[modelname]["test"].upper().startswith("N/A"):
            state += NA_TEST
        return state

