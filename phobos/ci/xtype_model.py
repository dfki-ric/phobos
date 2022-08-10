from ..core import Robot
from ..defs import *

from .combined_model import CombinedModel


class XTypeModel(CombinedModel):
    def __init__(self, configfile, pipeline, processed_model_exists=True, only_create=False):
        configfile = load_json(open(configfile, 'r'))
        if only_create:
            if "model" in configfile["xtype_model"].keys():
                configfile = {
                    "xtype_model": {
                        "robotname": configfile["xtype_model"]["robotname"],
                        "modelname": configfile["xtype_model"]["modelname"],
                        "model": configfile["xtype_model"]["model"]
                    }
                }
            elif "join" in configfile["xtype_model"].keys() and "depends_on" in configfile["xtype_model"].keys():
                configfile = {
                    "xtype_model": {
                        "robotname": configfile["xtype_model"]["robotname"],
                        "modelname": configfile["xtype_model"]["modelname"],
                        "join": configfile["xtype_model"]["join"],
                        "depends_on": configfile["xtype_model"]["depends_on"]
                    }
                }
            else:
                raise Exception("Invalid config file")
        if "modeltype" not in configfile["xtype_model"]:
            configfile["xtype_model"]["modeltype"] = "xtype"

        super().__init__(configfile, pipeline, processed_model_exists, super_call=True)

        assert "xtype_model" in self.cfg.keys()
        assert DEIMOS_AVAILABLE

        self.basedir = os.path.join(self.tempdir, "xtype_model")
        self.basefile = os.path.join(self.basedir, "urdf", "combined_model.urdf")

        self.deimos = Deimos(config={"temp_path": os.path.join(self.tempdir, "parts"), "only_deimos": True})

        if self.processed_model_exists:
            self._load_robot()

        print("Finished reading config and joining models to base model", configfile, flush=True)

    def _join_to_basefile(self):
        if hasattr(self, "model"):
            self.deimos.load(self.model["modelname"], self.model["modelversion"])
            self.deimos.convertParts()
            kwargs = self.deimos.generateURDFAssemblyCfg()["xtype_model"]
            with open(self.tempdir+"/join_cfg.yml", "w") as f:
                f.write(dump_json(kwargs))
            for (k, v) in kwargs.items():
                setattr(self, k, v)
            super()._join_to_basefile()
        elif hasattr(self, "join") and hasattr(self, "depends_on"):
            super()._join_to_basefile()
        else:
            raise Exception("This model definition is not a valid definition of a xtype model!\n"
                            "It lacks the 'join' and the 'depends_on' definition or a model definition")

    def _load_robot(self):
        if not self.processed_model_exists:
            if os.path.exists(os.path.join(self.basedir, "smurf", "combined_model.smurf")):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                        smurffile=os.path.join(self.basedir, "smurf", "combined_model.smurf"))
            else:
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                        xmlfile=self.basefile)
            return
        else:
            if not os.path.isfile(self.exporturdf):
                raise Exception('Preprocessed file {} not found!'.format(self.exporturdf))
            if os.path.exists(os.path.join(self.exportdir, "smurf", self.robotname + ".smurf")):
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                        smurffile=os.path.join(self.exportdir, "smurf", self.robotname + ".smurf"))
            else:
                self.robot = Robot(name=self.robotname if self.robotname else None,
                                        xmlfile=self.exporturdf)
