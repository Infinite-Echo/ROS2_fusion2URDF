import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
import csv
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R

def print_joint_info(app: adsk.core.Application, joint: adsk.fusion.AsBuiltJoint):
    app.log(f'Joint Name: {joint.name}')
    
    if(joint.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType):
        app.log(f'Joint Type: Revolute')
        geometry = joint.geometry
        point = geometry.origin
        app.log(f'Joint Origin: {point.asArray()}')
        app.log(f'Joint Primary Axis {geometry.primaryAxisVector.asArray()}')
        app.log(f'Joint Secondary Axis {geometry.secondaryAxisVector.asArray()}')
        app.log(f'Joint Third Axis {geometry.thirdAxisVector.asArray()}')
    elif(joint.jointMotion.jointType == adsk.fusion.JointTypes.RigidJointType):
        app.log(f'Joint Type: Rigid')
    else:
        app.log(f'Joint Type: Unsupported By Script')
    app.log(f'Occurence One: {joint.occurrenceOne.name}')
    tf_matrix = get_occurrence_tf(joint.occurrenceOne)
    rotation_matrix = tf_matrix[0:3, 0:3]
    rotation = R.from_matrix(rotation_matrix)
    rotation = rotation.as_euler('xyz', degrees=True)
    app.log(f'xyz = {tf_matrix[0:3, 3]}')
    app.log(f'rpy = {rotation}')
    app.log(f'Occurence Two: {joint.occurrenceTwo.name}')
    tf_matrix = get_occurrence_tf(joint.occurrenceTwo)
    rotation_matrix = tf_matrix[0:3, 0:3]
    rotation = R.from_matrix(rotation_matrix)
    rotation = rotation.as_euler('xyz', degrees=True)
    app.log(f'xyz = {tf_matrix[0:3, 3]}')
    app.log(f'rpy = {rotation}')
    return

def get_occurrence_tf(occurrence: adsk.fusion.Occurrence) -> np.ndarray[(4,4), np.dtype[any]]:
    tf_mat = np.array(occurrence.transform2.asArray()).reshape((4,4))
    return tf_mat

def calc_parent_to_child_tf(parent: adsk.fusion.Occurrence, child: adsk.fusion.Occurrence) -> np.ndarray[(4,4), np.dtype[any]]:
    parent_tf = get_occurrence_tf(parent)
    child_tf = get_occurrence_tf(child)
    tf = parent_tf @ child_tf
    return tf

def test_urdf(app: adsk.core.Application, design: adsk.fusion.Design):
    app.log("Starting URDF Test")
    root = design.rootComponent
    joints = root.asBuiltJoints
    joint = joints.itemByName('Rigid 1')
    print_joint_info(app, joint=joint)
    tf = calc_parent_to_child_tf(joint.occurrenceTwo, joint.occurrenceOne)
    app.log(f'tf = \n{tf}')
    app.log(f'xyz = {tf[0:3, 3]}')
    parent_tf = get_occurrence_tf(joint.occurrenceTwo)
    child_tf = get_occurrence_tf(joint.occurrenceOne)

    origin, x_axis, y_axis, z_axis = joint.occurrenceOne.transform2.getAsCoordinateSystem()
    app.log(f'origin: {origin.asArray()}')
    xyz = child_tf[0:3, 3] - parent_tf[0:3, 3]
    app.log(f'{joint.occurrenceOne.name} xyz: {xyz}')
    rot = tf[0:3, 0:3]
    rot = R.from_matrix(rot)
    rot = rot.as_euler('xyz', degrees=False)
    app.log(f'rpy = {rot}')

    app.log(f'point = {joint.occurrenceTwo.component.name}')
    app.log(f'point 2 = {joint.occurrenceTwo.component.originConstructionPoint.geometry.getData()}')
    app.log(f'point 1 = {joint.occurrenceOne.component.originConstructionPoint.geometry.getData()}')

    # Must set assembly context properly to get component origin point in global frame
    comp = joint.occurrenceTwo.component
    origin = comp.originConstructionPoint
    originProxy = origin.createForAssemblyContext(joint.occurrenceTwo)
    point = originProxy.geometry
    app.log(f'X: {point.x}, Y: {point.y}, Z: {point.z}')

    # center_component(app=app, occ=joint.occurrenceTwo)

    app.log("Ending URDF Test")

def get_component_origin(occ: adsk.fusion.Occurrence) -> adsk.core.Point3D:
    comp = occ.component
    origin = comp.originConstructionPoint
    originProxy = origin.createForAssemblyContext(occ)
    return originProxy.geometry

def center_component(app: adsk.core.Application, occ: adsk.fusion.Occurrence):
    origin_point = get_component_origin(occ=occ)
    Tm = adsk.core.Matrix3D.create()
    # Tm = Tm.setWithArray([1, 0, 0, ])
    occ.transform2.invert()
    tf = origin_point.transformBy(occ)
    app.log(f'tf: X: {origin_point.x}, Y: {origin_point.y}, Z: {origin_point.z}')

def test_urdf_2(app: adsk.core.Application, design: adsk.fusion.Design):
    app.log("Starting URDF Test")
    root = design.rootComponent
    joints = root.asBuiltJoints
    joint = joints.itemByName('Rigid 1')
    print_joint_info(app, joint=joint)
    tf = calc_parent_to_child_tf(joint.occurrenceOne, joint.occurrenceTwo)
    app.log(f'tf = \n{tf}')
    app.log(f'xyz = {tf[0:3, 3]}')
    rotation = R.from_matrix(tf[0:3, 0:3])
    rot = rotation.as_euler('xyz', degrees=False)
    app.log(f'r: {round(rot[0], 6)}, p: {round(rot[1], 6)}, y: {round(rot[2], 6)}')
    app.log(f'X: {round((tf[0, 3] / 100), 6)}, Y: {round((tf[1, 3] / 100), 6)}, Z: {round((tf[2, 3] / 100), 6)}')

    #Front Left Shoulder
    joint = joints.itemByName('Rigid 45')
    print_joint_info(app, joint=joint)
    tf = calc_parent_to_child_tf_2(joint.occurrenceOne, joint.occurrenceTwo)
    app.log(f'tf = \n{tf}')
    app.log(f'xyz = {tf[0:3, 3]}')
    rotation = R.from_matrix(tf[0:3, 0:3])
    rot = rotation.as_euler('xyz', degrees=False)
    app.log(f'r: {round(rot[0], 6)}, p: {round(rot[1], 6)}, y: {round(rot[2], 6)}')
    app.log(f'X: {round((tf[0, 3] / 100), 6)}, Y: {round((tf[1, 3] / 100), 6)}, Z: {round((tf[2, 3] / 100), 6)}')
    parent_tf = get_occurrence_tf(joint.occurrenceOne)
    child_tf = get_occurrence_tf(joint.occurrenceTwo)

    joint = joints.itemByName('Revolute 46')
    print_joint_info(app, joint=joint)
    app.log(f'Vector 1 = {joint.geometry.primaryAxisVector.asArray()}')
    app.log(f'xyz = {joint.geometry.origin.asArray()}')
    tf = calc_parent_to_child_tf_3(joint.occurrenceTwo, joint.occurrenceOne, joint.geometry)
    app.log(f'tf = \n{tf}')
    app.log(f'xyz = {tf[0:3, 3]}')
    rotation = R.from_matrix(tf[0:3, 0:3])
    rot = rotation.as_euler('xyz', degrees=False)
    app.log(f'r: {round(rot[0], 6)}, p: {round(rot[1], 6)}, y: {round(rot[2], 6)}')
    app.log(f'X: {round((tf[0, 3] / 100), 6)}, Y: {round((tf[1, 3] / 100), 6)}, Z: {round((tf[2, 3] / 100), 6)}')

    child_tf = get_occurrence_tf(joint.occurrenceOne)
    child_xyz = joint.geometry.origin.asArray()
    child_tf[0, 3] = child_xyz[0]
    child_tf[1, 3] = child_xyz[1]
    child_tf[2, 3] = child_xyz[2]
    tf = np.dot(np.linalg.inv(child_tf), get_occurrence_tf(joint.occurrenceOne))
    app.log(f'tf = \n{tf}')
    app.log(f'xyz = {tf[0:3, 3]}')
    rotation = R.from_matrix(tf[0:3, 0:3])
    rot = rotation.as_euler('xyz', degrees=False)
    app.log(f'r: {round(rot[0], 6)}, p: {round(rot[1], 6)}, y: {round(rot[2], 6)}')
    app.log(f'X: {round((tf[0, 3] / 100), 6)}, Y: {round((tf[1, 3] / 100), 6)}, Z: {round((tf[2, 3] / 100), 6)}')

    app.log("Ending URDF Test")

def calc_parent_to_child_tf_2(parent: adsk.fusion.Occurrence, child: adsk.fusion.Occurrence) -> np.ndarray[(4,4), np.dtype[any]]:
    parent_tf = get_occurrence_tf(parent)
    child_tf = get_occurrence_tf(child)
    tf = np.dot(np.linalg.inv(parent_tf), child_tf)
    return tf

def calc_parent_to_child_tf_3(parent: adsk.fusion.Occurrence, child: adsk.fusion.Occurrence, joint: adsk.fusion.JointGeometry) -> np.ndarray[(4,4), np.dtype[any]]:
    parent_tf = get_occurrence_tf(parent)
    child_tf = get_occurrence_tf(child)
    child_xyz = joint.origin.asArray()
    child_tf[0, 3] = child_xyz[0]
    child_tf[1, 3] = child_xyz[1]
    child_tf[2, 3] = child_xyz[2]
    tf = np.dot(np.linalg.inv(parent_tf), child_tf)
    return tf