Import and Export
=================

## Importing a robot model

## Exporting a robot model

Model data can be exported with Phobos in a number of formats: YAML, URDF, SMURF.

If you're done with editing, you can export the model by again selecting all relevant layers (which in this case will be the first five), then select all objects (hitting 'A' twice will do the trick) and then click on the "Export Robot Model" button on the very bottom of the tools panel on the left. Make sure to check the options that you need, that is, whether or not the meshes should be exported and if yes, in which format. It's not necessary to export the meshes every time you export the robot (just make sure that if you change the type of the meshes - which will be used to write the file names into URDF - is not changed without exporting the meshes in that type, otherwise URDF will not find the meshes).

## Custom property handling

When exporting a model to smurf, it is not intrinsically obvious what to do with all the custom properties defined in the model's objects. This is why we introduced a 'category system' in the names of custom properties.

Generally speaking, custom properties with simple names such as 'level' or 'left_side' are simply ignored by the export operator. This is so you may define custom properties to be used only in Blender, for instance in custom-made scripts. There are a few exceptions to this rule, like the predefined 'phobostype' property, which is used throughout Phobos' code, or some properties that we have not yet changed for reasons of backwards-compatibility. However, in general, plain simple properties are simply annotations to objects in Blender only.
The second category of custom properties is *type-specific* information. The simplest example occurs in armature objects representing links. As the object represents multiple output objects, type-specific information is encoded with the prefix `type/`. For instance, joint-specific information such as the maximum effort and velocity of the joint get encoded as `joint/maxeffort` and `joint/maxvelocity` respectively. This information will only be written to the joint, but not to the link. Similarly, attaching a motor to a joint works by creating the motor's properties as custom properties of the armature with the prefix `motor/`.
Finally, there is *custom information*. Any custom property named in the format `category/property` where category is not one of the phobostypes associated with an object, will get written to a custom YAML file of the SMURF output named 'category.yml'. This file will contain a list of dictionaries called 'category', with every dictionary having 2+n items: *name* and *type* of the object, as well as n entries named after the specified properties, containing the respective values. Thus custom properties of the same category defined in any object of a robot model will end up in the same YAML file. 

## Quirks
 
As always when dealing with complicated constructs, there are numerous special cases that need to be covered, at least for providing some form of workaround to avoid problems. Thus the following list of features.
 
### URDF formatting and naming

A common use case for Phobos in the ROS world might be to import an existing URDF, edit the model, and export it again. This works in principle, but you will find that Phobos will make a number of changes to your URDF file, as it is essentially digested upon import and created completely from scratch (i.e. from the Blender scene) upon export. As of version 0.5, we have not tested this use case extensively and there may be some changes that you don't want in your model. To name but two, visual and collision elements will always be given a name and it might be that some joint axes are defined explicitly which were previously defined implicitly via the orientation of the joint.
We have not invested a lot of time in dealing with such inconsistencies as for the moment, our standard use case is to create a robot in Blender and then export it in URDF, thus using URDF mostly downstream from Blender. Please report any issues arising from using Phobos to edit existing URDF files and we will try to take care of them ASAP.
 
### Mesh filenames

If the custom property `filename` exists in a visual or collision object and the object is of 'geometry/type' mesh, then that filename is written to the URDF output, but the mesh is not exported even if the other meshes are. The reason for this behavior is that is allows to import an existing URDF, make some changes and export it without overwriting existing meshes - consequently filenames of meshes are written in this custom property upon import. Thus if your particular desire is to edit the actual mesh and you for some reason would like to do so *in situ*, you have to delete this custom property before exporting your model.

### Object prefixes (URDF) <a name="#objectprefixes"></a>

It is perfectly acceptable in URDF to give a link, a joint and a sensor all the same name, 'camera' for instance. URDF does not run into conflicts here as objects of different types exist in their mutual exclusive type spaces. When trying to represent all these objects in Blender, the obvious problem occurs that suddenly several objects may be supposed to have the same name. Blender quite creatively deals with this by attaching '.xyz' to the name of an object whose designated name already existed when it is created ('xyz' being a running number). To avoid such rather awkward names (which would be exported, of course), duplicate objects receive a prefix upon import, for instance the camera sensor in the above example would be called 'sensor:camera'. Upon export, any prefixed in the shape of '*:' are sliced from the rest of the name. If you're not importing a model, this is something you have to do manually. Alternatively, it might be a good idea to avoid duplicate names alltogether.

### Joint names

Similar to the [Object prefixes](#objectprefixes), links and joints present a naming problem in the sense that they are two objects in URDF derived from one object in Blender. It is entirely possible to export them with the same name, however if you wish to have separate names created automatically, simply tick the 'typetags' option in the export operator. 
