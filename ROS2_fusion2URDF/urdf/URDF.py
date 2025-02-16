from .link import Link
from .joint import Joint
from .urdf_utils import get_occurrence_tf, tf_to_rpy_str, tf_to_xyz_str, get_joint_child_occ, parse_occ_name, parse_name
from ..simulation.materials import Mats
from ..simulation.gazebo import GazeboXacro, GazeboReference, GazeboPlugin
from xml.etree.ElementTree import ElementTree, Element, SubElement
import xml.etree.ElementTree as ET
import xml.dom.minidom
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
import adsk, adsk.core, adsk.fusion, traceback
import os, sys
from . import xacro as X

class URDF(ElementTree):
    def __init__(self, robot_name: str, export_path: str, app: adsk.core.Application, inputs: adsk.core.CommandInputs):
        super().__init__(
            element=Element(
                "robot",
                {"name": robot_name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"},
            )
        )
        self.package_name = f'{robot_name}_description'
        self.robot_name = robot_name
        self.export_path = export_path
        self.app = app
        design = adsk.fusion.Design.cast(self.app.activeProduct)
        self._inputs = inputs
        self._material_table = adsk.core.TableCommandInput.cast(self._inputs.itemById('contact_coefficient_table'))
        self._materials = Mats(robot_name=robot_name, design=design, material_table=self._material_table, app=app)
        self._gazebo = GazeboXacro(robot_name=robot_name, design=design, app=app, cmd_inputs=inputs)
        self.stl_export_manager = design.exportManager
        self.getroot().append(X.Property('package_name', self.package_name))
        self.getroot().append(X.Include(f'$(find ${{package_name}})/src/urdf/materials.xacro'))
        self.getroot().append(X.Include(f'$(find ${{package_name}})/src/urdf/gazebo.xacro'))
        os.makedirs(f'{self.export_path}/{self.package_name}/src/meshes', exist_ok=True)

    def create_base_link(self, base_link_occ: adsk.fusion.Occurrence, base_footprint: adsk.fusion.ConstructionPoint = None):
        base_link_occ.component.name = 'base'
        base_link = Link('base', app=self.app, cmd_inputs=self._inputs)
        tf = get_occurrence_tf(base_link_occ)
        tf[0:3, 3] = 0.0
        base_link.set_from_tf(tf=tf)
        base_link.set_inertial(base_link_occ)
        base_link.visual.set_material(base_link_occ)
        self.append_link(base_link)
        if base_footprint != None:
            self.create_base_footprint(base_link_occ, base_footprint)
        self.export_stl(base_link_occ)

    def create_base_footprint(self, base_link_occ: adsk.fusion.Occurrence, base_footprint: adsk.fusion.ConstructionPoint):
        base_footprint_link = Element('link', attrib={"name": f"base_footprint"})
        base_joint = Joint('base', None, cmd_inputs=self._inputs, app=self.app)
        base_joint.set_parent_value(parent_link='base_footprint', override_suffix=True)
        base_joint.set_child_value(child_link='base')

        base_link_tf = get_occurrence_tf(base_link_occ)
        base_footprint_tf = np.identity(4, dtype=np.float32)
        base_footprint_tf[0:3,3] = base_footprint.geometry.asArray()
        tf = np.dot(np.linalg.inv(base_footprint_tf), base_link_tf)
        base_joint.set_from_tf(tf=tf)
        self.append_link(base_footprint_link)
        self.append_joint(base_joint)
        return

    def traverse_link(self, parent_link: adsk.fusion.Occurrence, parent_joint: adsk.fusion.AsBuiltJoint = None):
        design = adsk.fusion.Design.cast(self.app.activeProduct)
        root_component = design.rootComponent

        for joint in root_component.allAsBuiltJoints:
            if(joint.occurrenceOne == parent_link) or (joint.occurrenceTwo == parent_link):
                if (parent_joint != None) and (parent_joint.name == joint.name):
                    continue
                child_link = get_joint_child_occ(parent=parent_link, joint=joint)
                self.create_link(child_link=child_link, child_joint=joint, parent_link=parent_link, parent_joint=parent_joint)
                self.traverse_link(parent_link=child_link, parent_joint=joint)

    def create_link(self, child_link: adsk.fusion.Occurrence, child_joint: adsk.fusion.AsBuiltJoint, parent_link: adsk.fusion.Occurrence, parent_joint: adsk.fusion.AsBuiltJoint = None):
        parent_link_tf = None
        child_link_tf = None
        child_joint_tf = None

        #create new link for child
        new_link = Link(parse_occ_name(child_link), app=self.app, cmd_inputs=self._inputs) # Initializes all xyz, rpy values to 0
        new_joint = Joint(parse_name(child_joint.name), child_joint, cmd_inputs=self._inputs, app=self.app)
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
            parent_joint_xyz = parent_joint.geometry.origin.asArray()
            parent_link_tf[0:3, 3] = parent_joint_xyz
        else:
            parent_link_tf = get_occurrence_tf(parent_link)
            
        child_link_tf = get_occurrence_tf(child_link)
        if(child_joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType):
            tf = get_occurrence_tf(child_link)
            child_joint_xyz = child_joint.geometry.origin.asArray()
            tf[0:3, 3] = child_joint_xyz
            child_joint_tf = np.dot(np.linalg.inv(parent_link_tf), tf)
            new_link.set_xyz(tf_to_xyz_str(np.dot(np.linalg.inv(tf), child_link_tf))) # Set offsets in link since joint is non-rigid
        else:
            child_joint_tf = np.dot(np.linalg.inv(parent_link_tf), child_link_tf)

        new_joint.set_from_tf(child_joint_tf)
        new_link.set_inertial(child_link)
        new_link.set_materials(child_link)
        self.append_link(new_link)
        self.append_joint(new_joint)
        self.export_stl(child_link)
        return

    def append_link(self, link: Element):
        self.getroot().append(link)

    def append_joint(self, joint: Element):
        self.getroot().append(joint)

    def export_stl(self, occ: adsk.fusion.Occurrence):
        stl_options = self.stl_export_manager.createSTLExportOptions(occ)
        stl_options.filename = f'{self.export_path}/{self.package_name}/src/meshes/{parse_occ_name(occ=occ)}.stl'
        self.stl_export_manager.execute(stl_options)
        del(stl_options)

        # export collision meshes if low refinement used
        if(self._inputs.itemById('collision_mesh_refinement_input').value):
            stl_options = self.stl_export_manager.createSTLExportOptions(occ)
            stl_options.filename = f'{self.export_path}/{self.package_name}/src/meshes/{parse_occ_name(occ=occ)}_collision.stl'
            stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            self.stl_export_manager.execute(stl_options)
            del(stl_options)

    def export(self):
        os.makedirs(f'{self.export_path}/{self.package_name}/src/urdf', exist_ok=True)
        self.write_xml_to_file(self.getroot(), f'{self.export_path}/{self.package_name}/src/urdf/{self.robot_name}.xacro')
        self.write_xml_to_file(self._materials.getroot(), f'{self.export_path}/{self.package_name}/src/urdf/materials.xacro')
        self.write_xml_to_file(self._gazebo.getroot(), f'{self.export_path}/{self.package_name}/src/urdf/gazebo.xacro')
        
    def write_xml_to_file(self, root_element: Element, filepath: str):
        xml_string = ET.tostring(root_element, 'utf-8')
        formatted_xml = self.prettify_urdf(xml_string=xml_string)
        with open(filepath, 'w') as file:
            file.write(formatted_xml)

    def prettify_urdf(self, xml_string):
        """Return a pretty-printed XML string for the given URDF."""
        dom = xml.dom.minidom.parseString(xml_string)
        pretty_xml = dom.toprettyxml(indent="  ")

        # Add new lines between <link> and <joint> elements for better readability
        lines = pretty_xml.split('\n')
        new_lines = []
        for line in lines:
            if (line.strip() == '</link>') or (line.strip() == '</joint>') or (line.strip().startswith('</xacro')):
                new_lines.append(f'{line}\n')
            else:
                new_lines.append(line)
        
        return '\n'.join(new_lines)

    def get_input_value(self, input_name: str):
        pass