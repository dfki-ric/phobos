from .base import SmurfAnnotation


class Material(SmurfAnnotation):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


class Collision(SmurfAnnotation):
    def __init__(self, robot, link, collision, bitmask=None, noDataPackage=False, reducedDataPackage=False, ccfm=None):
        super().__init__(robot=robot, link=link)
        self.name = collision.name
        self.returns += ['name', 'link']
        self.bitmask = bitmask
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
        if ccfm is not None:
            self.ccfm = ccfm
        if bitmask is not None:
            self.returns += ['bitmask']


class Link(SmurfAnnotation):
    def __init__(self, robot, link, noDataPackage=False, reducedDataPackage=False):
        super().__init__(robot=robot, link=link)
        if type(link) is str:
            self.name = link
        else:
            self.name = link.name
        self.returns = ['name']
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage


class Joint(SmurfAnnotation):
    def __init__(self, robot, joint, noDataPackage=False, reducedDataPackage=False,
                 damping_const_constraint_axis1=None, springDamping=None, springStiffness=None,
                 spring_const_constraint_axis1=None):
        super().__init__(robot=robot, joint=joint)
        if type(joint) is str:
            self.name = joint
        else:
            self.name = joint.name
        self.returns = ['name']
        if noDataPackage is not None:
            self.noDataPackage = noDataPackage
        if reducedDataPackage is not None:
            self.reducedDataPackage = reducedDataPackage
        if damping_const_constraint_axis1 is not None:
            self.damping_const_constraint_axis1 = damping_const_constraint_axis1
        if springDamping is not None:
            self.springDamping = springDamping
        if springStiffness is not None:
            self.springStiffness = springStiffness
        if spring_const_constraint_axis1 is not None:
            self.spring_const_constraint_axis1 = spring_const_constraint_axis1
