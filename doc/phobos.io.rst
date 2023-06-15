phobos io module
================

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

Representation
--------------

Nearly all representation classes base on the Linkable class.
As python offers no direct access to memory management things like shared pointers can't directly be used.
To be memory-efficient and efficient regarding changing of existing instances.
The robot representations hold the instances of representations like Links, Joints, Materials, Submechanisms etc.
Those refer to other instances internally (e.g. a Joint holds a reference to the parent link and a child link) by name.
When ever we change e.g. a link name it would be necessary to go through all other instances to check whether the reference has be updated as well.

The Linkable class no allows to link instance with a robot instance.
Whenever we link an instance to a robot all its references to other linkables are replaced by a python-reference.
This way an update (change of properties) of an instance takes effect every where where it's referenced.
Unlinking replaces this python-reference again by the string-type name, hence we'd have to take care of the properties again manually.

It is therefore necessary that all Linkables have a unique name inside a robot.
When the unique identifyer is needed, make sure to use str() on the object, that way you'll get the unique name, no matter if its currently linked to a robot or not. (See __str__() of Linkables)


XML-Representation
``````````````````

To represent XML-robot-formats like SDF and URDF, we defined the format in a separate JSON file in data/xml_formats.json.
This file defines for each format for each class in io.representation and sensor.representation which properties are
read/written from/to the XML-file.
For each class the following properties are defined therefore:
- attributes: define the xml attributes of the element
- children: child-elements that are classes there-selves keyed by there xml tag
- attribute_children: are properties that are stored as attributes of a child-element with a defined tag
- value_children: are properties that are stored as text of child-elements with a defined tag

XMLDefinition class provides access to these definitions and takes care of the (de-)serialization.
XMLFactory associates each io.representation class with its corresponding XMLDefinition.
By using class_factory() on all io.representation classes in io.__init__.py this association is performed and each class
is extended with methods to serialize to and from xml and all available xml-dialects (URDF etc.).

As sensor elements in URDF and SDF are a bit more complex, an SensorFactory class was necessary to parse and write the Sensors to those formats.

io.xmlrobot fuses all this to the pure representation of URDF and SDF.

SMURF-Representation
`````````````````````

The SMURF representation happens by inheritance of the SMURFReflection class.
It defines which properties will be exported using `returns` and `excludes`.


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
