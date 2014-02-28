Blender Transformations
-----------------------

== Rotation Matrices ==

In Blender, every object possesses four different transformation matrices to save it current location, rotation and scale in space. These are:

- matrix_basis
- matrix_local
- matrix_world
- matrix_parent_inverse

These matrices make the parent-child relationships possible that objects can be structurally ordered with in Blender. Thus very different behavior can be observed for these matrices depending on whether they belong to a *global* or *child* object. Global objects, i.e. objects which do not have a parent, their location, rotation and scale are fully defined in relation to the world, thus their matrix_basis, matrix_local and matrix_world are all equal. As they possess no parent, the parent inverse matrix is the identity transform. In objects which are children of other objects, i.e., which have a parent, the three matrices serve different functions, as outlined above.

=== matrix_basis ===

matrix_basis is what is displayed in Blender's "Transform" settings in the sidebar. This is often a point of confusion because it does *not* necessarily refer to the object's actual position and orientation in space. For global objects it does

=== matrix_local ===

The local matrix defines the transformation from the parent world transform to the child transform. This means that if the local matrix of a child is applied to its basis, it would reside at its parent's origin.

=== matrix_world ===

The absolute transform of the object in the world. This is the location, orientation and scale at which the object is displayed in blender.

=== matrix_parent_inverse ===

The parent inverse matrix is set at the time of parenting and *never changes afterwards*, no matter what transformations are applied to the parent or the child (in fact, it doesn't even change after a parent-child relationship is cancelled and is simply ignored by Blender until the object becomes a child again, in which case it is simply overwritten). It is the transformation which, if applied to the child, reverses the change of the origin of the child's coordinate system that resulted from establishing the parent-child relationship. It is thus equal to the inverse of the parent's *world* transform. This last point is important! As the parent could be a child of another object, the local and basis transforms would not reflect the change of origin of the parent, thus the world transform has to be use to derive the parent inverse.

=== Summary ===

- matrix_basis: the object's "own" transform in its own coordinate system
- matrix_local: the transform which brings an object from it's parent's origin to its position in the world
- matrix_world: the absolute transform of the object with respect to the world
- matrix_parent_inverse: the inverse of the parent's world transform at time of parenting


== Location & Transformation ==
