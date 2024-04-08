from .basics import *
from .core import (
    SimpleElementType,
    start_namespace,
    add_type, Object,
    reflect,
    Attribute,
    Element,
    ValueType,
    FactoryType,
    on_error,
    AggregateElement,
    DuckTypedFactory,
    RawType,
    end_namespace,)

# Add a 'namespace' for names to avoid a conflict between URDF and SDF?
# A type registry? How to scope that? Just make a 'global' type pointer?
# Or just qualify names? urdf.geometric, sdf.geometric

start_namespace('urdf')

add_type('element_link', SimpleElementType('link', str))
add_type('element_xyz', SimpleElementType('xyz', 'vector3'))

verbose = True


class Pose(Object):
    def __init__(self, xyz=None, rpy=None):
        self.xyz = xyz
        self.rpy = rpy

    def check_valid(self):
        assert (self.xyz is None or len(self.xyz) == 3) and \
            (self.rpy is None or len(self.rpy) == 3)

    # Aliases for backwards compatibility
    @property
    def rotation(self): return self.rpy

    @rotation.setter
    def rotation(self, value): self.rpy = value

    @property
    def position(self): return self.xyz

    @position.setter
    def position(self, value): self.xyz = value


reflect(Pose, tag='origin', params=[
    Attribute('xyz', 'vector3', False, default=[0, 0, 0]),
    Attribute('rpy', 'vector3', False, default=[0, 0, 0])
])


# Common stuff
name_attribute = Attribute('name', str)
origin_element = Element('origin', Pose, False)


class Color(Object):
    def __init__(self, *args):
        # What about named colors?
        count = len(args)
        if count == 4 or count == 3:
            self.rgba = args
        elif count == 1:
            self.rgba = args[0]
        elif count == 0:
            self.rgba = None
        if self.rgba is not None:
            if len(self.rgba) == 3:
                self.rgba += [1.]
            if len(self.rgba) != 4:
                raise Exception('Invalid color argument count')


reflect(Color, tag='color', params=[
    Attribute('rgba', 'vector4')
])


class JointDynamics(Object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction


reflect(JointDynamics, tag='dynamics', params=[
    Attribute('damping', float, False),
    Attribute('friction', float, False)
])


class Box(Object):
    def __init__(self, size=None):
        self.size = size


reflect(Box, tag='box', params=[
    Attribute('size', 'vector3')
])


class Cylinder(Object):
    def __init__(self, radius=0.0, length=0.0):
        self.radius = radius
        self.length = length


reflect(Cylinder, tag='cylinder', params=[
    Attribute('radius', float),
    Attribute('length', float)
])


class Sphere(Object):
    def __init__(self, radius=0.0):
        self.radius = radius


reflect(Sphere, tag='sphere', params=[
    Attribute('radius', float)
])


class Mesh(Object):
    def __init__(self, filename=None, scale=None):
        self.filename = filename
        self.scale = scale


reflect(Mesh, tag='mesh', params=[
    Attribute('filename', str),
    Attribute('scale', 'vector3', required=False)
])


class GeometricType(ValueType):
    def __init__(self):
        self.factory = FactoryType('geometric', {
            'box': Box,
            'cylinder': Cylinder,
            'sphere': Sphere,
            'mesh': Mesh
        })

    def from_xml(self, node, path):
        children = xml_children(node)
        assert len(children) == 1, 'One element only for geometric'
        return self.factory.from_xml(children[0], path=path)

    def write_xml(self, node, obj):
        name = self.factory.get_name(obj)
        child = node_add(node, name)
        obj.write_xml(child)


add_type('geometric', GeometricType())


class Collision(Object):
    def __init__(self, geometry=None, origin=None):
        self.geometry = geometry
        self.origin = origin


reflect(Collision, tag='collision', params=[
    origin_element,
    Element('geometry', 'geometric')
])


class Texture(Object):
    def __init__(self, filename=None):
        self.filename = filename


reflect(Texture, tag='texture', params=[
    Attribute('filename', str)
])


class Material(Object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def check_valid(self):
        if self.color is None and self.texture is None:
            on_error("Material has neither a color nor texture.")


reflect(Material, tag='material', params=[
    name_attribute,
    Element('color', Color, False),
    Element('texture', Texture, False)
])


class LinkMaterial(Material):
    def check_valid(self):
        pass


class Visual(Object):
    def __init__(self, geometry=None, material=None, origin=None, name=None):
        self.geometry = geometry
        self.material = material
        self.name = name
        self.origin = origin


reflect(Visual, tag='visual', params=[
    Attribute('name', str, False),
    origin_element,
    Element('geometry', 'geometric'),
    Element('material', LinkMaterial, False)
])


class Inertia(Object):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz]]


reflect(Inertia, tag='inertia',
             params=[Attribute(key, float) for key in Inertia.KEYS])


class Inertial(Object):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin


reflect(Inertial, tag='inertial', params=[
    origin_element,
    Element('mass', 'element_value'),
    Element('inertia', Inertia, False)
])


# FIXME: we are missing the reference position here.
class JointCalibration(Object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling


reflect(JointCalibration, tag='calibration', params=[
    Attribute('rising', float, False, 0),
    Attribute('falling', float, False, 0)
])


class JointLimit(Object):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper


reflect(JointLimit, tag='limit', params=[
    Attribute('effort', float),
    Attribute('lower', float, False, 0),
    Attribute('upper', float, False, 0),
    Attribute('velocity', float)
])

# FIXME: we are missing __str__ here.


class JointMimic(Object):
    def __init__(self, joint_name=None, multiplier=None, offset=None):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset


reflect(JointMimic, tag='mimic', params=[
    Attribute('joint', str),
    Attribute('multiplier', float, False),
    Attribute('offset', float, False)
])


class SafetyController(Object):
    def __init__(self, velocity=None, position=None, lower=None, upper=None):
        self.k_velocity = velocity
        self.k_position = position
        self.soft_lower_limit = lower
        self.soft_upper_limit = upper


reflect(SafetyController, tag='safety_controller', params=[
    Attribute('k_velocity', float),
    Attribute('k_position', float, False, 0),
    Attribute('soft_lower_limit', float, False, 0),
    Attribute('soft_upper_limit', float, False, 0)
])


class Joint(Object):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    def check_valid(self):
        assert self.type in self.TYPES, "Invalid joint type: {}".format(self.type)  # noqa

    # Aliases
    @property
    def joint_type(self): return self.type

    @joint_type.setter
    def joint_type(self, value): self.type = value

reflect(Joint, tag='joint', params=[
    name_attribute,
    Attribute('type', str),
    origin_element,
    Element('axis', 'element_xyz', False),
    Element('parent', 'element_link'),
    Element('child', 'element_link'),
    Element('limit', JointLimit, False),
    Element('dynamics', JointDynamics, False),
    Element('safety_controller', SafetyController, False),
    Element('calibration', JointCalibration, False),
    Element('mimic', JointMimic, False),
])


class Link(Object):
    def __init__(self, name=None, visual=None, inertial=None, collision=None,
                 origin=None):
        self.aggregate_init()
        self.name = name
        self.visuals = []
        if visual:
            self.visual = visual
        self.inertial = inertial
        self.collisions = []
        if collision:
            self.collision = collision
        self.origin = origin

    def __get_visual(self):
        """Return the first visual or None."""
        if self.visuals:
            return self.visuals[0]

    def __set_visual(self, visual):
        """Set the first visual."""
        if self.visuals:
            self.visuals[0] = visual
        else:
            self.visuals.append(visual)
        if visual:
            self.add_aggregate('visual', visual)

    def __get_collision(self):
        """Return the first collision or None."""
        if self.collisions:
            return self.collisions[0]

    def __set_collision(self, collision):
        """Set the first collision."""
        if self.collisions:
            self.collisions[0] = collision
        else:
            self.collisions.append(collision)
        if collision:
            self.add_aggregate('collision', collision)

    # Properties for backwards compatibility
    visual = property(__get_visual, __set_visual)
    collision = property(__get_collision, __set_collision)


reflect(Link, tag='link', params=[
    name_attribute,
    origin_element,
    AggregateElement('visual', Visual),
    AggregateElement('collision', Collision),
    Element('inertial', Inertial, False),
])


class PR2Transmission(Object):
    def __init__(self, name=None, joint=None, actuator=None, type=None,
                 mechanicalReduction=1):
        self.name = name
        self.type = type
        self.joint = joint
        self.actuator = actuator
        self.mechanicalReduction = mechanicalReduction


reflect(PR2Transmission, tag='pr2_transmission', params=[
    name_attribute,
    Attribute('type', str),
    Element('joint', 'element_name'),
    Element('actuator', 'element_name'),
    Element('mechanicalReduction', float)
])


class Actuator(Object):
    def __init__(self, name=None, mechanicalReduction=1):
        self.name = name
        self.mechanicalReduction = None


reflect(Actuator, tag='actuator', params=[
    name_attribute,
    Element('mechanicalReduction', float, required=False)
])


class TransmissionJoint(Object):
    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.hardwareInterfaces = []

    def check_valid(self):
        assert len(self.hardwareInterfaces) > 0, "no hardwareInterface defined"


reflect(TransmissionJoint, tag='joint', params=[
    name_attribute,
    AggregateElement('hardwareInterface', str),
])


class Transmission(Object):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """

    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.joints = []
        self.actuators = []

    def check_valid(self):
        assert len(self.joints) > 0, "no joint defined"
        assert len(self.actuators) > 0, "no actuator defined"


reflect(Transmission, tag='new_transmission', params=[
    name_attribute,
    Element('type', str),
    AggregateElement('joint', TransmissionJoint),
    AggregateElement('actuator', Actuator)
])

add_type('transmission',
              DuckTypedFactory('transmission',
                                    [Transmission, PR2Transmission]))


class Robot(Object):
    def __init__(self, name=None, version=None):
        self.aggregate_init()

        self.name = name
        if version is not None:
            self.version = version
        self.joints = []
        self.links = []
        self.materials = []
        self.gazebos = []
        self.transmissions = []

        self.joint_map = {}
        self.link_map = {}

        self.parent_map = {}
        self.child_map = {}

    def add_aggregate(self, typeName, elem):
        Object.add_aggregate(self, typeName, elem)

        if typeName == 'joint':
            joint = elem
            self.joint_map[joint.name] = joint
            self.parent_map[joint.child] = (joint.name, joint.parent)
            if joint.parent in self.child_map:
                self.child_map[joint.parent].append((joint.name, joint.child))
            else:
                self.child_map[joint.parent] = [(joint.name, joint.child)]
        elif typeName == 'link':
            link = elem
            self.link_map[link.name] = link

    def add_link(self, link):
        self.add_aggregate('link', link)

    def add_joint(self, joint):
        self.add_aggregate('joint', joint)

    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            (joint, parent) = self.parent_map[link]
            if joints:
                if fixed or self.joint_map[joint].joint_type != 'fixed':
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    def get_root(self):
        root = None
        for link in self.link_map:
            if link not in self.parent_map:
                assert root is None, "Multiple roots detected, invalid URDF."
                root = link
        assert root is not None, "No roots detected, invalid URDF."
        return root

    def post_read_xml(self):
        if self.version is None:
            self.version = "1.0"

        split = self.version.split(".")
        if len(split) != 2:
            raise ValueError("The version attribute should be in the form 'x.y'")

        if split[0] == '' or split[1] == '':
            raise ValueError("Empty major or minor number is not allowed")

        if int(split[0]) < 0 or int(split[1]) < 0:
            raise ValueError("Version number must be positive")

        if self.version != "1.0":
            raise ValueError("Invalid version; only 1.0 is supported")

    @classmethod
    def from_parameter_server(cls, key='robot_description'):
        """
        Retrieve the robot model on the parameter server
        and parse it to create a URDF robot structure.

        Warning: this requires roscore to be running.
        """
        # Could move this into xml_reflection
        import rclpy
        return cls.from_xml_string(rclpy.get_param(key))


reflect(Robot, tag='robot', params=[
    Attribute('name', str),
    Attribute('version', str, False),
    AggregateElement('link', Link),
    AggregateElement('joint', Joint),
    AggregateElement('gazebo', RawType()),
    AggregateElement('transmission', 'transmission'),
    AggregateElement('material', Material)
])

# Make an alias
URDF = Robot

end_namespace()