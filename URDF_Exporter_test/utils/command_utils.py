import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
import csv


def init_materials_table(design: adsk.fusion.Design, app: adsk.core.Application, 
                           table: adsk.core.TableCommandInput) -> adsk.core.TableCommandInput:
    materials_list = []
    add_materials_table_header_row(table=table)
    row_num = 1
    for i in range(design.materials.count):
        row_num = add_materials_table_row(material_name=design.materials.item(i).name,
                                materials_list=materials_list,
                                row_num=row_num,
                                table=table)
    return table

def add_materials_table_row(material_name: str, materials_list: list, row_num: int,
                            table: adsk.core.TableCommandInput) -> int:
    table_inputs = adsk.core.CommandInputs.cast(table.commandInputs)
    if material_name not in materials_list:
        materials_list.append(material_name)
        material_name_input = table_inputs.addStringValueInput('material_name_input_{}'.format(str(row_num)), 'Material Name', material_name.replace(" ", "_"))
        material_name_input.isReadOnly = True
        friction_coef_input = table_inputs.addStringValueInput('friction_coefficient_input_{}'.format(str(row_num)), 'Friction Coefficient', '0.0')
        friction_coef_input.tooltip = 'Friction Coefficient (float)'
        stiffness_coef_input = table_inputs.addStringValueInput('stiffness_coefficient_input_{}'.format(str(row_num)), 'Stiffness Coefficient', '0.0')
        stiffness_coef_input.tooltip = 'Stiffness Coefficient (float)'
        dampening_coef_input = table_inputs.addStringValueInput('dampening_coefficient_input_{}'.format(str(row_num)), 'Dampening Coefficient', '0.0')
        dampening_coef_input.tooltip = 'Dampening Coefficient (float)'
        table.addCommandInput(material_name_input, row_num, 0)
        table.addCommandInput(friction_coef_input, row_num, 1)
        table.addCommandInput(stiffness_coef_input, row_num, 2)
        table.addCommandInput(dampening_coef_input, row_num, 3)
        row_num += 1
    return row_num

def add_materials_table_header_row(table: adsk.core.TableCommandInput):
    table_inputs = adsk.core.CommandInputs.cast(table.commandInputs)
    material_name_input = table_inputs.addStringValueInput('material_name_input_header', 'Material Name', 'Material')
    friction_coef_input = table_inputs.addStringValueInput('friction_coefficient_input_header', 'Friction Coefficient', 'Mu')
    stiffness_coef_input = table_inputs.addStringValueInput('stiffness_coefficient_input_header', 'Stiffness Coefficient', 'Kp')
    dampening_coef_input = table_inputs.addStringValueInput('dampening_coefficient_input_header', 'Dampening Coefficient', 'Kd')
    material_name_input.isReadOnly = True
    friction_coef_input.isReadOnly = True
    stiffness_coef_input.isReadOnly = True
    dampening_coef_input.isReadOnly = True
    table.addCommandInput(material_name_input, 0, 0)
    table.addCommandInput(friction_coef_input, 0, 1)
    table.addCommandInput(stiffness_coef_input, 0, 2)
    table.addCommandInput(dampening_coef_input, 0, 3)