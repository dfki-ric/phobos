import os

from ..utils import misc
from .base_model import BaseModel
from ..utils.commandline_logging import get_logger
log = get_logger(__name__)

SUBMECHS_VIA_ASSEMBLIES = False


class Model(BaseModel):
    def __init__(self, configfile, pipeline, processed_model_exists=True):
        super().__init__(configfile, pipeline, processed_model_exists)

        assert "model" in self.cfg.keys()

        if hasattr(self, "basefile"):
            self.basefile = os.path.abspath(os.path.join(self.pipeline.configdir, self.basefile))
        elif hasattr(self, "derived_base"):
            self.basefile = self.get_exported_model_path(self.pipeline,
                                                         os.path.join(self.pipeline.configdir, self.derived_base))
        else:
            raise Exception("Neither a basefile or a derived_base entry given."
                            "So we don't now from which model we should derive your model!")

        if self.processed_model_exists:
            self._load_robot()

        log.debug(f"Finished reading config {configfile}")

    def process(self):
        misc.recreate_dir(self.pipeline, self.tempdir)
        misc.recreate_dir(self.pipeline, self.exportdir)

        self._load_robot()

        if not hasattr(self, 'robot') or not hasattr(self, 'pipeline'):
            raise Exception('Model is not initialized!')

        super()._process()

