from xml.etree.ElementTree import ElementTree, Element, SubElement
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
import adsk, adsk.core, adsk.fusion, traceback
from .urdf_utils import get_occurrence_tf, tf_to_rpy_str, tf_to_xyz_str

class Link(Element):
    def __init__(self, name: str, app: adsk.core.Application) -> None:
        super().__init__("link", attrib={"name": f"{name}_link"})
        self.app = app
        self.inertial = Inertial()
        self.visual = Visual()
        self.collision = Collision()
        self.append(self.inertial)
        self.append(self.visual)
        self.append(self.collision)
        self.set_mesh_filepath(filepath=f'file://$(find ${{package_name}})/src/meshes/{name}.stl')

    def set_inertial(self, link_occ: adsk.fusion.Occurrence):
        physical_properties = link_occ.component.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        mass = physical_properties.mass
        self.inertial.set_mass_value(mass=mass)
        offset_xyz = [(float(i)) for i in self.inertial.get_xyz_value().split()]
        center_of_mass = [_/100.0 for _ in physical_properties.centerOfMass.asArray()] ## cm to m
        (_, xx, yy, zz, xy, yz, xz) = physical_properties.getXYZMomentsOfInertia()
        moment_inertia_world = [_ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz] ] ## kg / cm^2 -> kg/m^2
        x = center_of_mass[0]
        y = center_of_mass[1]
        z = center_of_mass[2]
        inertia_xyz = [x + offset_xyz[0], y + offset_xyz[1], z + offset_xyz[2]]
        self.inertial.set_xyz_value(f'{inertia_xyz[0]:.10e} {inertia_xyz[1]:.10e} {inertia_xyz[2]:.10e}')
        translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                            -x*y, -y*z, -x*z]
        inertia = [ (i - mass*t) for i, t in zip(moment_inertia_world, translation_matrix)]
        inertia_dict = {
            "ixx":f'{inertia[0]:.10e}',
            "iyy":f'{inertia[1]:.10e}',
            "izz":f'{inertia[2]:.10e}',
            "ixy":f'{inertia[3]:.10e}',
            "iyz":f'{inertia[4]:.10e}',
            "ixz":f'{inertia[5]:.10e}'
        }
        self.inertial.set_inertia_values(inertia_dict=inertia_dict)

    def set_xyz(self, xyz: str):
        self.inertial.set_xyz_value(xyz=xyz)
        self.visual.set_xyz_value(xyz=xyz)
        self.collision.set_xyz_value(xyz=xyz)

    def set_rpy(self, rpy: str):
        self.inertial.set_rpy_value(rpy=rpy)
        self.visual.set_rpy_value(rpy=rpy)
        self.collision.set_rpy_value(rpy=rpy)
    
    def set_mesh_filepath(self, filepath: str):
        self.visual.set_mesh_filepath(filepath=filepath)
        self.collision.set_mesh_filepath(filepath=filepath)

    def set_from_tf(self, tf: np.ndarray[(4,4), np.dtype[any]]):
        self.set_xyz(xyz=tf_to_xyz_str(tf=tf))
        self.set_rpy(rpy=tf_to_rpy_str(tf=tf))


class LinkElement(Element):
    def __init__(self, name: str):
        super().__init__(name)
        self.__origin = Element("origin", attrib={"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
        self.append(self.__origin)

    def get_xyz_value(self):
        return self.__origin.attrib["xyz"]

    def get_rpy_value(self):
        return self.__origin.attrib["rpy"]

    def set_xyz_value(self, xyz):
        if type(xyz) == list:
            xyz = f"{(xyz[0] /100):.10e} {(xyz[1] /100):.10e} {(xyz[2] /100):.10e}"
        self.__origin.attrib["xyz"] = xyz

    def set_rpy_value(self, rpy):
        if type(rpy) == list:
            rpy = f"{round(rpy[0], 6)} {round(rpy[1], 6)} {round(rpy[2], 6)}"
        self.__origin.attrib["rpy"] = rpy


class Visual(LinkElement):
    def __init__(self):
        super().__init__("visual")
        self.__geometry = Element("geometry")
        self.__mesh = Element("mesh", attrib={"filename": " ", "scale":"0.001 0.001 0.001"})
        self.__geometry.append(self.__mesh)
        self.append(self.__geometry)

    def get_mesh_filepath(self):
        return self.__mesh.attrib["filename"]

    def set_mesh_filepath(self, filepath: str):
        self.__mesh.attrib["filename"] = filepath
    
    def set_material(self, link_occ: adsk.fusion.Occurrence):
        material = link_occ.component.material
        color = adsk.core.ColorProperty.cast(material.appearance.appearanceProperties.itemByName('Color'))
        rgba_values = color.value.getColor()
        self.__material = Element("material", attrib={"name": f'{material.name.replace(" ", "_")}'})
        self.__color = Element("color", attrib={"rgba": f'{rgba_values[1]/255} {rgba_values[2]/255} {rgba_values[3]/255} {rgba_values[4]/255}'})
        self.__material.append(self.__color)
        self.append(self.__material)


class Collision(LinkElement):
    def __init__(self):
        super().__init__("collision")
        self.__geometry = Element("geometry")
        self.__mesh = Element("mesh", attrib={"filename": " ", "scale":"0.001 0.001 0.001"})
        self.__geometry.append(self.__mesh)
        self.append(self.__geometry)

    def get_mesh_filepath(self):
        return self.__mesh.attrib["filename"]

    def set_mesh_filepath(self, filepath: str):
        self.__mesh.attrib["filename"] = filepath


class Inertial(LinkElement):
    def __init__(self):
        super().__init__("inertial")
        self.__inertia_dict = {
            "ixx": "0.0",
            "iyy": "0.0",
            "izz": "0.0",
            "ixy": "0.0",
            "iyz": "0.0",
            "ixz": "0.0",
        }
        self.__mass = Element("mass", attrib={"value": "0.0"})
        self.__inertia = Element("inertia", attrib=self.__inertia_dict)
        self.append(self.__mass)
        self.append(self.__inertia)

    def get_inertia_values(self) -> dict[str, str]:
        return self.__inertia.attrib

    def set_inertia_values(self, inertia_dict: dict[str, str]) -> None:
        self.__inertia.attrib = inertia_dict
        return

    def get_mass_value(self) -> str:
        return self.__mass.attrib["value"]

    def set_mass_value(self, mass) -> None:
        if type(mass) != str:
            mass = f'{mass:.10e}'
        self.__mass.attrib["value"] = mass
