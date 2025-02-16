from xml.etree.ElementTree import ElementTree, Element, SubElement
import xml.etree.ElementTree as ET
import xml.dom.minidom
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
import adsk, adsk.core, adsk.fusion, traceback
import os, sys
import yaml

class GazeboXacro(ElementTree):
    def __init__(self, robot_name: str, design: adsk.fusion.Design, 
                 app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
        super().__init__(Element('robot', {"name": robot_name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"}))
        self.getroot().append(GazeboPlugin('ignition-gazebo-joint-state-publisher-system', 'ignition::gazebo::systems::JointStatePublisher'))
        self._design = design
        self._app = app
        self._cmd_inputs = cmd_inputs

class GazeboPlugin(Element):
    def __init__(self, filename: str, name: str):
        super().__init__('gazebo')
        self._plugin = Element('plugin', attrib={'filename':filename, 'name':name})
        self.append(self._plugin)

class JointPositionController(GazeboPlugin):
    def __init__(self, joint_name: str):
        super().__init__(filename='ignition-gazebo-joint-position-controller-system',
                         name='ignition::gazebo::systems::JointPositionController')
        

class GazeboReference(Element):
    def __init__(self, reference_name: str):
        super().__init__('gazebo', attrib={"reference":reference_name})
    
    def set_friction(self, mu1: float, mu2: float):
        self.__mu1 = Element('mu1')
        self.__mu1.text = f'{mu1}'
        self.__mu2 = Element('mu2')
        self.__mu2.text = f'{mu2}'
        self.append(self.__mu1)
        self.append(self.__mu2)

    def set_material(self, diffuse: str, ambient: str, specular: str):
        self.__material = Element('material')
        self.__diffuse = Element('diffuse')
        self.__diffuse.text = diffuse
        self.__ambient = Element('ambient')
        self.__ambient.text = ambient
        self.__specular = Element('specular')
        self.__specular.text = specular
        self.__material.append(self.__diffuse)
        self.__material.append(self.__ambient)
        self.__material.append(self.__specular)
        self.append(self.__material)