from xml.etree.ElementTree import ElementTree, Element
import adsk, adsk.core, adsk.fusion, traceback
import numpy as np
from ..urdf import xacro as X
import yaml

def col_to_attrib_name(col_num: int):
    if col_num == 1:
        return 'mu'
    if col_num == 2:
        return 'kp'
    if col_num == 3:
        return 'kd'

class Mats(ElementTree):
    def __init__(self, robot_name: str, design: adsk.fusion.Design, material_table: adsk.core.TableCommandInput, app: adsk.core.Application):
        super().__init__(Element('robot', {"name": robot_name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"}))
        self._design = design
        self._mat_table = material_table
        self._app = app
        self._existing_contact_coefficients = []
        materials_list = []
        for i in range(design.materials.count):
            material_name = design.materials.item(i).name
            if material_name not in materials_list:
                materials_list.append(material_name)
                self.add_new_material(design.materials.item(i))
        self.create_coefficients(material_table)

    def add_new_material(self, material: adsk.core.Material):
        color = adsk.core.ColorProperty.cast(material.appearance.appearanceProperties.itemByName('Color'))
        rgba_values = color.value.getColor()
        new_mat = Element('material', attrib={'name':f'{material.name.replace(" ", "_")}'})
        new_mat.append(Element('color', attrib={'rgba': f'{rgba_values[1]/255} {rgba_values[2]/255} {rgba_values[3]/255} {rgba_values[4]/255}'}))
        self.getroot().append(new_mat)

    def create_coefficients(self, material_table: adsk.core.TableCommandInput):
        for i in range(1, material_table.rowCount):
            attrib_dict = {}
            for j in range(1, material_table.numberOfColumns):
                table_entry = material_table.getInputAtPosition(i, j)
                table_entry = adsk.core.StringValueCommandInput.cast(table_entry)
                if table_entry.value == None:
                    continue
                elif float(table_entry.value) == 0.0:
                    continue
                else:
                    attrib_dict[col_to_attrib_name(j)] = table_entry.value
            if len(attrib_dict) > 0:
                name = f"{material_table.getInputAtPosition(i,0).value}_contact_coefficients"
                contact_coefs_prop = X.PropertyBlock(name)
                self._existing_contact_coefficients.append(name.removesuffix('_contact_coefficients'))
                contact_coefs = Element('contact_coefficients', attrib=attrib_dict)
                contact_coefs_prop.append(contact_coefs)
                self.getroot().append(contact_coefs_prop)
        existing_contact_coefficients_element = X.Property('existing_contact_coefficients', f"{self._existing_contact_coefficients}")
        self.getroot().append(existing_contact_coefficients_element)
