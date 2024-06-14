from xml.etree.ElementTree import Element, SubElement
import xml.etree.ElementTree as ET
import adsk, adsk.core, adsk.fusion, traceback
import numpy as np
from .urdf_utils import get_occurrence_tf, tf_to_rpy_str, tf_to_xyz_str

class Joint(Element):
    def __init__(self, name: str, joint: adsk.fusion.AsBuiltJoint):
        joint_type = self.convert_fusion_joint_type_to_URDF(joint=joint)
        super().__init__("joint", attrib={"name": f"{name}_joint", "type": joint_type})
        self.__origin = Element("origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
        self.__parent = Element("parent", attrib={"link": ""})
        self.__child = Element("child", attrib={"link": ""})
        self.append(self.__origin)
        self.append(self.__parent)
        self.append(self.__child)
        if(joint_type != 'fixed'):
            self.set_axis_value(joint=joint)

    def convert_fusion_joint_type_to_URDF(self, joint: adsk.fusion.AsBuiltJoint) -> str:
        joint_type = joint.jointMotion.jointType
        if(joint_type == adsk.fusion.JointTypes.RevoluteJointType):
            joint_motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
            if(joint_motion.rotationLimits.isMaximumValueEnabled == True) and (joint_motion.rotationLimits.isMinimumValueEnabled == True):
                return 'revolute'
            else:
                return 'continuous'
        else:
            return 'fixed'
        
    def set_axis_value(self, joint: adsk.fusion.AsBuiltJoint):
        axis_vector = joint.geometry.primaryAxisVector.asArray()
        self.__axis = Element("axis", attrib={"xyz": f"{round(axis_vector[0], 6)} {round(axis_vector[1], 6)} {round(axis_vector[2], 6)}"})
        self.append(self.__axis)

    def get_parent_value(self):
        return self.__parent.attrib["link"]

    def get_child_value(self):
        return self.__child.attrib["link"]

    def get_xyz_value(self):
        return self.__origin.attrib["xyz"]

    def get_rpy_value(self):
        return self.__origin.attrib["rpy"]

    def set_parent_value(self, parent_link: str):
        self.__parent.attrib["link"] = f'{parent_link}_link'

    def set_child_value(self, child_link: str):
        self.__child.attrib["link"] = f'{child_link}_link'

    def set_xyz_value(self, xyz):
        if type(xyz) == list:
            xyz = f"{round(xyz[0] /1000, 6)} {round(xyz[1] /1000, 6)} {round(xyz[2] /1000, 6)}"
        self.__origin.attrib["xyz"] = xyz

    def set_rpy_value(self, rpy):
        if type(rpy) == list:
            rpy = f"{round(rpy[0], 6)} {round(rpy[1], 6)} {round(rpy[2], 6)}"
        self.__origin.attrib["rpy"] = rpy

    def set_from_tf(self, tf: np.ndarray[(4,4), np.dtype[any]]):
        self.set_xyz_value(xyz=tf_to_xyz_str(tf=tf))
        self.set_rpy_value(rpy=tf_to_rpy_str(tf=tf))