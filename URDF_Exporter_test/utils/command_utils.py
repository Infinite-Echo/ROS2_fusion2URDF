import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
import csv
import yaml

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

def init_joint_dynamics_table(design: adsk.fusion.Design, app: adsk.core.Application, 
                           table: adsk.core.TableCommandInput) -> adsk.core.TableCommandInput:
    joint_list = []
    add_joint_dynamics_table_header_row(table=table)
    row_num = 1
    for joint in design.rootComponent.allAsBuiltJoints:
        if joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType:
            joint_name = joint.name
            row_num = add_joint_dynamics_table_row(joint_name=joint_name,
                                    joint_list=joint_list,
                                    row_num=row_num,
                                    table=table)
    return table

def add_joint_dynamics_table_row(joint_name: str, joint_list: list, row_num: int,
                            table: adsk.core.TableCommandInput) -> int:
    table_inputs = adsk.core.CommandInputs.cast(table.commandInputs)
    if joint_name not in joint_list:
        joint_list.append(joint_name)
        joint_name_input = table_inputs.addStringValueInput('joint_dynamics_name_input{}'.format(str(row_num)), 'Joint Name', joint_name.replace(" ", "_"))
        joint_name_input.isReadOnly = True
        joint_friction_input = table_inputs.addStringValueInput('joint_dynamics_friction_input_{}'.format(str(row_num)), 'Joint Friction', '0.0')
        joint_damping_input = table_inputs.addStringValueInput('joint_dynamics_damping_input_{}'.format(str(row_num)), 'Joint Damping', '0.0')
        table.addCommandInput(joint_name_input, row_num, 0)
        table.addCommandInput(joint_friction_input, row_num, 1)
        table.addCommandInput(joint_damping_input, row_num, 2)
        row_num += 1
    return row_num

def add_joint_dynamics_table_header_row(table: adsk.core.TableCommandInput):
    table_inputs = adsk.core.CommandInputs.cast(table.commandInputs)
    joint_name_input = table_inputs.addStringValueInput('joint_dynamics_name_input_header', 'Joint Name', 'Joint')
    joint_friction_input = table_inputs.addStringValueInput('joint_dynamics_friction_input_header', 'Joint Friction Coefficient', 'Friction')
    joint_damping_input = table_inputs.addStringValueInput('joint_dynamics_damping_input_header', 'Joint Damping Coefficient', 'Damping')
    joint_name_input.isReadOnly = True
    joint_friction_input.isReadOnly = True
    joint_damping_input.isReadOnly = True
    table.addCommandInput(joint_name_input, 0, 0)
    table.addCommandInput(joint_friction_input, 0, 1)
    table.addCommandInput(joint_damping_input, 0, 2)

def get_file_from_dialog(ui: adsk.core.UserInterface, filter: str = 'All files (*.*)', save: bool = False):
    file_dialog = ui.createFileDialog()
    file_dialog.filter = filter
    file_dialog.isMultiSelectEnabled = False
    if save == False:
        results = file_dialog.showOpen()
    else:
        results = file_dialog.showSave()

    if results == adsk.core.DialogResults.DialogOK:
        return file_dialog.filename
    else:
        return None

def get_folderpath_from_dialog(ui: adsk.core.UserInterface):
    folder_dialog = ui.createFolderDialog()
    results = folder_dialog.showDialog()
    if results == adsk.core.DialogResults.DialogOK:
        return folder_dialog.folder
    else:
        if results == adsk.core.DialogResults.DialogCancel:
            return None
        elif results == adsk.core.DialogResults.DialogError:
            ui.messageBox('Unable To Load File')
            return None
    
def load_material_config(app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
    filepath = get_file_from_dialog(ui=app.userInterface, filter='(*.yaml;*.yml);;All files (*.*)')
    if filepath == None:
        return
    with open(filepath, 'r') as file:
        config = yaml.load(file, yaml.Loader)
    contact_coef_table = adsk.core.TableCommandInput.cast(cmd_inputs.itemById('contact_coefficient_table'))
    for i in range(1, contact_coef_table.rowCount):
        material_name = contact_coef_table.getInputAtPosition(i, 0).value
        if material_name in config.keys():
            material = config[material_name]
            contact_coef_table.getInputAtPosition(i, 1).value = material['Mu']
            contact_coef_table.getInputAtPosition(i, 2).value = material['Kp']
            contact_coef_table.getInputAtPosition(i, 3).value = material['Kd']

def save_material_config(app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
    filepath = get_file_from_dialog(ui=app.userInterface, 
                                    filter='(*.yaml;*.yml);;All files (*.*)', save=True)
    if filepath == None:
        return
    contact_coef_table = adsk.core.TableCommandInput.cast(cmd_inputs.itemById('contact_coefficient_table'))
    materials = {}
    for i in range(1, contact_coef_table.rowCount):
        materials[contact_coef_table.getInputAtPosition(i, 0).value] = {
            'Mu':contact_coef_table.getInputAtPosition(i, 1).value,
            'Kp':contact_coef_table.getInputAtPosition(i, 2).value,
            'Kd':contact_coef_table.getInputAtPosition(i, 3).value
        }
    
    with open(filepath, 'w') as file:
        yaml.dump(materials, file, yaml.Dumper)

def load_joint_dynamics_config(app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
    filepath = get_file_from_dialog(ui=app.userInterface, filter='(*.yaml;*.yml);;All files (*.*)')
    if filepath == None:
        return
    with open(filepath, 'r') as file:
        config = yaml.load(file, yaml.Loader)
    joint_dynamics_table = adsk.core.TableCommandInput.cast(cmd_inputs.itemById('joint_dynamics_table'))
    for i in range(1, joint_dynamics_table.rowCount):
        joint_name = joint_dynamics_table.getInputAtPosition(i, 0).value
        if joint_name in config.keys():
            joint = config[joint_name]
            joint_dynamics_table.getInputAtPosition(i, 1).value = joint['Friction']
            joint_dynamics_table.getInputAtPosition(i, 2).value = joint['Damping']

def save_joint_dynamics_config(app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
    filepath = get_file_from_dialog(ui=app.userInterface, 
                                    filter='(*.yaml;*.yml);;All files (*.*)', save=True)
    if filepath == None:
        return
    joint_dynamics_table = adsk.core.TableCommandInput.cast(cmd_inputs.itemById('joint_dynamics_table'))
    joints = {}
    for i in range(1, joint_dynamics_table.rowCount):
        joints[joint_dynamics_table.getInputAtPosition(i, 0).value] = {
            'Friction':joint_dynamics_table.getInputAtPosition(i, 1).value,
            'Damping':joint_dynamics_table.getInputAtPosition(i, 2).value
        }
    
    with open(filepath, 'w') as file:
        yaml.dump(joints, file, yaml.Dumper)

def load_gz_plugins_config(app: adsk.core.Application, cmd_inputs: adsk.core.CommandInputs):
    filepath = get_file_from_dialog(ui=app.userInterface, filter='(*.yaml;*.yml);;All files (*.*)')
    current_gz_plugins_filepath_input = adsk.core.TextBoxCommandInput.cast(cmd_inputs.itemById('gz_plugins_filepath_display'))
    if filepath == None:
        new_path = f'<div align="left">{None}</div>'
    else:
        new_path = f'<div align="left">{filepath}</div>'
    current_gz_plugins_filepath_input.formattedText = new_path
    return