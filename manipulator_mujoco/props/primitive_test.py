from dm_control import mjcf
import numpy as np


class Primitive_test(object):
    """
    A base class representing a primitive object in a simulation environment.
    """

    def __init__(self, name_site=None, size_site=None, pos_site=None,  # site
                 name_body=None, pos_body=None,  # body
                 type_joint=None, axis_joint=None,  # joint
                 diaginertia=None,  # inertia
                 condim=None, type_geom=None, name_geom=None, quat_geom=None,  # geom
                 mass_geom=None, size_geom=None, rgba_geom=None,  # geom
                 friction=None, solimp=None, solref=None, **kwargs):  # geom
        """
        Initialize the Primitive object.

        Args:
            **kwargs: Additional keyword arguments for configuring the primitive.
        """
        self._mjcf_model = mjcf.RootElement()


        #  Add a site element to the worldbody
        self._site = self._mjcf_model.worldbody.add("site", **kwargs)

        if name_site is not None:
            self._site.name = name_site

        if size_site is not None:
            self._site.size = size_site

        if pos_site is not None:
            self._site.pos = pos_site



        #  Add a body element to the worldbody
        self._body = self._mjcf_model.worldbody.add("body", **kwargs)

        if name_body is not None:
            self._body.name = name_body

        if pos_body is not None:
            self._body.pos = pos_body



        #  Add a joint element to the worldbody
        self._joint = self._mjcf_model.worldbody.body[0].add("joint", **kwargs)

        if type_joint is not None:
            self._joint.type = type_joint

        if axis_joint is not None:
            self._joint.axis = axis_joint




        #  Add a inertial element to the worldbody
        self._inertial = self._mjcf_model.worldbody.body[0].add("inertial", mass=0.08, pos="0 0 0", **kwargs)

        # Set diaginertia
        if diaginertia is not None:
            self._inertial.diaginertia = diaginertia

        # Add a geometric element to the worldbody
        self._geom = self._mjcf_model.worldbody.body[0].add("geom", **kwargs)

        # Set condim
        if condim is not None:
            self._geom.condim = condim

        # Set type
        if type_geom is not None:
            self._geom.type = type_geom

        # Set name
        if name_geom is not None:
            self._geom.name = name_geom

        # Set quat
        if quat_geom is not None:
            self._geom.quat = quat_geom

        # Set mass
        if mass_geom is not None:
            self._geom.mass = mass_geom

        # Set size
        if size_geom is not None:
            self._geom.size = size_geom

        # Set rgba
        if rgba_geom is not None:
            self._geom.rgba = rgba_geom

        # Set friction
        if friction is not None:
            self._geom.friction = friction

        # Set solimp
        if solimp is not None:
            self._geom.solimp = solimp

        # Set solref
        if solref is not None:
            self._geom.solimp = solref


    @property
    def site(self):
        return self._site

    @property
    def body(self):
        return self._body

    @property
    def joint(self):
        return self._joint

    @property
    def inertial(self):
        return self._inertial

    @property
    def geom(self):
        """Returns the primitive's geom, e.g., to change color or friction."""
        return self._geom

    @property
    def mjcf_model(self):
        """Returns the primitive's mjcf model."""
        return self._mjcf_model


