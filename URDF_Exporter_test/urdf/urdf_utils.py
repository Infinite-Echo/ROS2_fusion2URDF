from xml.etree.ElementTree import ElementTree, Element, SubElement
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
import adsk, adsk.core, adsk.fusion, traceback
import re

def get_occurrence_tf(occurrence: adsk.fusion.Occurrence) -> np.ndarray[(4,4), np.dtype[any]]:
    tf_mat = np.array(occurrence.transform2.asArray()).reshape((4,4))
    return tf_mat

def tf_to_xyz_str(tf: np.ndarray[(4,4), np.dtype[any]]) -> str:
    xyz_str = f'{round((tf[0, 3] / 100), 6)} {round((tf[1, 3] / 100), 6)} {round((tf[2, 3] / 100), 6)}'
    return xyz_str

def tf_to_rpy_str(tf: np.ndarray[(4,4), np.dtype[any]]) -> str:
    rotation = R.from_matrix(tf[0:3, 0:3])
    rpy = rotation.as_euler('xyz', degrees=False)
    rpy_str = f'{round(rpy[0], 6)} {round(rpy[1], 6)} {round(rpy[2], 6)}'
    return rpy_str

def get_joint_child_occ(parent: adsk.fusion.Occurrence, joint: adsk.fusion.AsBuiltJoint) -> adsk.fusion.Occurrence:
    if(parent.name == joint.occurrenceOne.name):
        return joint.occurrenceTwo
    else:
        return joint.occurrenceOne
    
def calc_parent_to_child_tf(parent: adsk.fusion.Occurrence, child: adsk.fusion.Occurrence) -> np.ndarray[(4,4), np.dtype[any]]:
    parent_tf = get_occurrence_tf(parent)
    child_tf = get_occurrence_tf(child)
    tf = np.dot(np.linalg.inv(parent_tf), child_tf)
    return tf

def parse_name(name: str) -> str:
    pattern = r'^(.*):[^:]*$|^(.*)$'
    match = re.match(pattern, name)
    if match:
        return match.group(1) or match.group(2)
    return None

def parse_occ_name(occ: adsk.fusion.Occurrence) -> str:
    return parse_name(occ.name)