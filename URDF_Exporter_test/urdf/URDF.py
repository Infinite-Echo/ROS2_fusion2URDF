from .link import Link
from .joint import Joint
from .urdf_utils import get_occurrence_tf, tf_to_rpy_str, tf_to_xyz_str, get_joint_child_occ, parse_occ_name, parse_name
from xml.etree.ElementTree import ElementTree, Element, SubElement
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
import adsk, adsk.core, adsk.fusion, traceback


class URDF(ElementTree):
    def __init__(self, robot_name: str, export_path: str):
        super().__init__(
            element=Element(
                "robot",
                {"name": robot_name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"},
            )
        )
        self.export_path = export_path
        self.robot_name = robot_name

    def create_base_link(self, base_link_occ: adsk.fusion.Occurrence):
        base_link = Link('base')
        tf = get_occurrence_tf(base_link_occ)
        tf[0:3, 3] = 0
        base_link.set_from_tf(tf=tf)
        physical_properties = base_link_occ.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        mass = physical_properties.mass
        moment_of_inertia_tuple = physical_properties.getXYZMomentsOfInertia()
        inertia_dict = {
            "xx":f'{moment_of_inertia_tuple[1]}',
            "yy":f'{moment_of_inertia_tuple[2]}',
            "zz":f'{moment_of_inertia_tuple[3]}',
            "xy":f'{moment_of_inertia_tuple[4]}',
            "yz":f'{moment_of_inertia_tuple[5]}',
            "xz":f'{moment_of_inertia_tuple[6]}'
        }
        base_link.set_inertial(mass=mass, xyz="0.0 0.0 0.0", inertia_dict=inertia_dict)
        self.append_link(base_link)

    def traverse_link(self, app: adsk.core.Application, parent_link: adsk.fusion.Occurrence, parent_joint: adsk.fusion.AsBuiltJoint = None):
        for joint in parent_link.asBuiltJoints:
            if (parent_joint != None) and (parent_joint.name == joint.name):
                continue
            child_link = get_joint_child_occ(parent=parent_link, joint=joint)
            self.create_link(child_link=child_link, child_joint=joint, parent_link=parent_link, parent_joint=parent_joint)
            if(child_link.asBuiltJoints.count >= 2):
                self.traverse_link(app=app, parent_link=child_link, parent_joint=joint)
            else:
                return
                

    def create_link(self, child_link: adsk.fusion.Occurrence, child_joint: adsk.fusion.AsBuiltJoint, parent_link: adsk.fusion.Occurrence, parent_joint: adsk.fusion.AsBuiltJoint = None):
        parent_link_tf = None
        child_link_tf = None
        child_joint_tf = None
        parent_joint_tf = None

        #create new link for child
        new_link = Link(parse_occ_name(child_link))
        new_joint = Joint(parse_name(child_joint.name), child_joint)
        new_joint.set_child_value(parse_occ_name(child_link))
        new_joint.set_parent_value(parse_occ_name(parent_link))
        
        
        '''
        Important Notes:
            -Fusion API gives access to global coordinates of components, URDF uses local coordinates
            -If a joint is rigid, the joint origin uses the child link's origin because rigid-asBuiltJoints do not have a geometry property. 
             In this case, the child link's offsets are set to zero inside the visual, collision, inertial properties in the URDF
            -If a joint is not rigid, the child link's properties must be offset because the joint's geometry origin is used instead of the child link's origin
                -Additionally, when a new joint references a link whose parent joint is non-rigid, the new joint's origin starts from the parent joints origin not the link's origin
        '''
        if(parent_joint != None) and (parent_joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType):
            parent_link_tf = get_occurrence_tf(parent_link)
            parent_joint_tf = parent_link_tf.copy()
            parent_joint_xyz = parent_joint.geometry.origin.asArray()
            parent_joint_tf[0:3, 3] = parent_joint_xyz
            child_link_tf = get_occurrence_tf(child_link)
            child_joint_tf = np.dot(np.linalg.inv(parent_joint_tf), child_link_tf)
        else:
            parent_link_tf = get_occurrence_tf(parent_link)
            child_link_tf = get_occurrence_tf(child_link)
            child_joint_tf = np.dot(np.linalg.inv(parent_link_tf), child_link_tf)
            
        new_joint.set_from_tf(child_joint_tf)
        new_link.set_from_tf(np.dot(np.linalg.inv(child_joint_tf), child_link_tf))
        physical_properties = child_link.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        mass = physical_properties.mass
        moment_of_inertia_tuple = physical_properties.getXYZMomentsOfInertia()
        inertia_dict = {
            "xx":f'{moment_of_inertia_tuple[1]}',
            "yy":f'{moment_of_inertia_tuple[2]}',
            "zz":f'{moment_of_inertia_tuple[3]}',
            "xy":f'{moment_of_inertia_tuple[4]}',
            "yz":f'{moment_of_inertia_tuple[5]}',
            "xz":f'{moment_of_inertia_tuple[6]}'
        }
        new_link.set_inertial(mass=mass, xyz="0.0 0.0 0.0", inertia_dict=inertia_dict)
        self.append_link(new_link)
        self.append_joint(new_joint)
        return

    def append_link(self, link: Element):
        self.getroot().append(link)

    def append_joint(self, joint: Element):
        self.getroot().append(joint)

    def export(self):
        ET.indent(self, "  ")
        self.write(f'{self.export_path}/{self.robot_name}.xacro')
