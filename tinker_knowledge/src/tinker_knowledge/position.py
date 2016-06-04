import dexml
from dexml import fields

from geometry_msgs import msg


class Point(dexml.Model):
    x = fields.Float()
    y = fields.Float()
    z = fields.Float()

    @staticmethod
    def X(m):
        return msg.Point(x=m.x, y=m.y, z=m.z)

    @staticmethod
    def R(m):
        return Point(x=m.x, y=m.y, z=m.z)


class Quaternion(dexml.Model):
    x = fields.Float()
    y = fields.Float()
    z = fields.Float()
    w = fields.Float()

    @staticmethod
    def X(m):
        return msg.Quaternion(x=m.x, y=m.y, z=m.z, w=m.w)

    @staticmethod
    def R(m):
        return Quaternion(x=m.x, y=m.y, z=m.z, w=m.w)


class Pose(dexml.Model):
    name = fields.String()
    position = fields.Model(Point)
    orientation = fields.Model(Quaternion)

    @staticmethod
    def X(m):
        return msg.Pose(position=Point.X(m.position), orientation=Quaternion.X(m.orientation))

    @staticmethod
    def R(m):
        return Pose(position=Point.R(m.position), orientation=Quaternion.R(m.orientation))


class PoseList(dexml.Model):
    name = fields.String()
    pose = fields.List(Pose)
