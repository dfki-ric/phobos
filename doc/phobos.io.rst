phobos io module
----------------

The io module of phobos takes care of dealing with several model formats:
- SMURF (YAML)
- URDF (XML)
- SDF (XML)
- (more are under development)

This is the backend to phobos.core.Robot.

While "base", "xml_factory", "yaml_reflection", "smurf_reflection" provide the basic classes for the formats, further
files ("representation", "sensor_representations", "poses", "hyrodyn") define the basic representation classes for all
robot's entities.
"parser" holds the code to parse xml-files to the Robot class.
"xmlrobot" and "smurfrobot" define the basic classes for robots in URDF/SDF respectively smurf format; they hold all the
information models in the respective formats can hold.


Submodules
----------

phobos.io.base module
`````````````````````

.. automodule:: phobos.blender.phobosgui
    :members:
    :undoc-members:
    :show-inheritance:

phobos.io.xml_factory module
````````````````````````````
xml_factory depends on data/xml_formats.json. This file defines how XML formats like URDF and SDF are defined, and where
to obtain the corresponding information.

.. automodule:: phobos.blender.phobosgui
    :members:
    :undoc-members:
    :show-inheritance:

phobos.io.yaml_reflection module
````````````````````````````````

.. automodule:: phobos.blender.phobosgui
    :members:
    :undoc-members:
    :show-inheritance:

phobos.io.smurf_reflection module
`````````````````````````````````

.. automodule:: phobos.blender.phobosgui
    :members:
    :undoc-members:
    :show-inheritance:
