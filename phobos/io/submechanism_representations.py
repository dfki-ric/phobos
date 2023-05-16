

from .base import Representation
from .smurf_reflection import SmurfBase


class Submechanism(Representation, SmurfBase):
    type_dict = {} # TODO ??
    def __init__(self, name: str = None, link=None, submechtype=None,
                 rate=None, always_on=True, visualize=False, topic=None, enable_metrics=False,
                 _sdf_type=None, _blender_type=None, **kwargs):
        if link is not None:
            if type(link) != str:
                link = link.name
            kwargs["link"] = link
        SmurfBase.__init__(self, name=name,
                           rate=rate, always_on=always_on, visualize=visualize, topic=topic,
                           enable_metrics=enable_metrics,
                           returns=["type", "rate",], **kwargs)
        self.type = submechtype
        self._sdf_type = _sdf_type
        self._blender_type = _blender_type
        self.excludes += ["_blender_type", "_sdf_type"]

    @property
    def blender_type(self):
        return self._blender_type