#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils, test_utils
import csv
from .urdf.joint import Joint
from .urdf.link import Link
from .urdf.URDF import URDF
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
from .urdf.urdf_utils import get_joint_child_occ, get_occurrence_tf
from .ROS2.robot_description_pkg import generate_robot_description_pkg

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

_app = None
_ui  = None
_handlers = []

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

# Event handler that reacts to any changes the user makes to any of the command inputs.
class ExportUrdfCommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input

            if cmdInput.id == 'export_path_button_input':
                export_path_dialog = _ui.createFolderDialog()
                current_export_path_input = adsk.core.TextBoxCommandInput.cast(inputs.itemById('export_path_display'))
                results = export_path_dialog.showDialog()
                if results == adsk.core.DialogResults.DialogOK:
                    _app.log(f'Export Path Selected: {export_path_dialog.folder}')
                    new_path = f'<div align="left">{export_path_dialog.folder}</div>'
                    current_export_path_input.formattedText = new_path
            elif '_coefficient_' in cmdInput.id:
                coefficient_input = adsk.core.StringValueCommandInput.cast(cmdInput)
                if coefficient_input.value.isnumeric() or coefficient_input.value.replace('.', '', 1).isnumeric():
                    coefficient_input.isValueError = False
                else:
                    coefficient_input.isValueError = True
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# Event handler that reacts to when the command is destroyed. This terminates the script.            
class ExportUrdfCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # When the command is done, terminate the script
            # This will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler that reacts when the command definitio is executed which
# results in the command being created and this event being fired.
class ExportUrdfCommandCreaterHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # Get the command that was created.
            cmd = adsk.core.Command.cast(args.command)
            cmd.isPositionDependent = True

            # Connect to the command destroyed event.
            onDestroy = ExportUrdfCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)

            # Connect to the input changed event.           
            onInputChanged = ExportUrdfCommandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)    

            onExecute = ExportUrdfCommandExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

            # Get the CommandInputs collection associated with the command.
            inputs = cmd.commandInputs

            basic_tab = inputs.addTabCommandInput('basic_tab', 'Basic')
            basic_tab_inputs = basic_tab.children

            advanced_tab = inputs.addTabCommandInput('advanced_tab', 'Advanced')
            advanced_tab_inputs = advanced_tab.children

            ### Basic
            # Get Robot Name
            robot_name_input = basic_tab_inputs.addStringValueInput('robot_name_string_input', 'Robot Name', '')

            # Select Component to Use as Base Link
            base_link_selection_input = basic_tab_inputs.addSelectionInput('base_link_selection', 'Select Base Link', 'Select a component to be base_link')
            base_link_selection_input.setSelectionLimits(minimum=1, maximum=1)
            base_link_selection_input.addSelectionFilter("Occurrences")

            # Select the Export Path for the Package
            export_path_button_input = basic_tab_inputs.addBoolValueInput('export_path_button_input', 'Export Path', False, 'resources/EllipsisButton', True)  
            export_path_display = basic_tab_inputs.addTextBoxCommandInput('export_path_display', '', '', 1, True)
            
            ### Advanced - Optional
            # Select Point to Use as Base Footprint
            base_footprint_selection_input = advanced_tab_inputs.addSelectionInput('base_footprint_selection', 'Select Base Footprint', 'Select a point to be base_footprint')
            base_footprint_selection_input.setSelectionLimits(minimum=0, maximum=1)
            base_footprint_selection_input.addSelectionFilter("ConstructionPoints")

            # Use Material Contact Coefficients
            material_contact_coef_group = advanced_tab_inputs.addGroupCommandInput('material_contact_coefficient_group', 'Set Material Contact Coefficients')
            material_contact_coef_group_inputs = material_contact_coef_group.children
            contact_coef_table = material_contact_coef_group_inputs.addTableCommandInput('contact_coefficient_table', 'Material Contact Coefficients', 4, '1:1:1:1')
            contact_coef_table_inputs = adsk.core.CommandInputs.cast(contact_coef_table.commandInputs)

            design = adsk.fusion.Design.cast(_app.activeProduct)
            materials_list = []
            row_num = 0
            for i in range(design.materials.count):
                if design.materials.item(i).name not in materials_list:
                    materials_list.append(design.materials.item(i).name)
                    material_name_input = contact_coef_table_inputs.addStringValueInput('material_name_input_{}'.format(str(i)), 'Material Name', design.materials.item(i).name.replace(" ", "_"))
                    material_name_input.isReadOnly = True
                    friction_coef_input = contact_coef_table_inputs.addStringValueInput('friction_coefficient_input_{}'.format(str(i)), 'Friction Coefficient', '0.0')
                    friction_coef_input.tooltip = 'Friction Coefficient (float)'
                    stiffness_coef_input = contact_coef_table_inputs.addStringValueInput('stiffness_coefficient_input_{}'.format(str(i)), 'Stiffness Coefficient', '0.0')
                    stiffness_coef_input.tooltip = 'Stiffness Coefficient (float)'
                    dampening_coef_input = contact_coef_table_inputs.addStringValueInput('dampening_coefficient_input_{}'.format(str(i)), 'Dampening Coefficient', '0.0')
                    dampening_coef_input.tooltip = 'Dampening Coefficient (float)'
                    contact_coef_table.addCommandInput(material_name_input, row_num, 0)
                    contact_coef_table.addCommandInput(friction_coef_input, row_num, 1)
                    contact_coef_table.addCommandInput(stiffness_coef_input, row_num, 2)
                    contact_coef_table.addCommandInput(dampening_coef_input, row_num, 3)
                    row_num += 1

        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

class ExportUrdfCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # Cast the args to CommandEventArgs
            eventArgs = adsk.core.CommandEventArgs.cast(args)
            # Get the command
            cmd = eventArgs.command
            # Get the command inputs
            inputs = cmd.commandInputs

            base_footprint_selection_input = adsk.core.SelectionCommandInput.cast(inputs.itemById('base_footprint_selection'))
            if base_footprint_selection_input.selectionCount != 0:
                base_footprint = adsk.fusion.ConstructionPoint.cast(base_footprint_selection_input.selection(0).entity)
            else:
                base_footprint = None

            base_link_selection_input = adsk.core.SelectionCommandInput.cast(inputs.itemById('base_link_selection'))
            base_link = adsk.fusion.Occurrence.cast(base_link_selection_input.selection(0).entity)

            robot_name = adsk.core.StringValueCommandInput.cast(inputs.itemById('robot_name_string_input')).value
            export_path = adsk.core.TextBoxCommandInput.cast(inputs.itemById('export_path_display')).text
            
            generate_robot_description_pkg(export_path=export_path, robot_name=robot_name)
            urdf = URDF(robot_name=robot_name, export_path=export_path, app=_app, inputs=inputs)
            urdf.create_base_link(base_link_occ=base_link, base_footprint=base_footprint)
            urdf.traverse_link(parent_link=base_link, parent_joint=None)
            urdf.export()
            _app.log(f'Export Finished')

        except:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    try:
        global _app, _ui
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        # Get the existing command definition or create it if it doesn't already exist.
        cmdDef = _ui.commandDefinitions.itemById('cmd_export_urdf')
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition('cmd_export_urdf', 'Export URDF', 'Command for converting the active project into a URDF and exporting it')

        # Connect to the command created event.
        onCommandCreated = ExportUrdfCommandCreaterHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)

        # Execute the command definition.
        cmdDef.execute()

        # Prevent this module from being terminated when the script returns, because we are waiting for event handlers to fire.
        adsk.autoTerminate(False)
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))