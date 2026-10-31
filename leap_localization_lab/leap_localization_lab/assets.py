"""Prepare per-session copies; never edit the installed TurtleBot3 model."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_assets(model_path, world_path, directory):
    model_path, world_path, directory = map(Path, (model_path, world_path, directory))
    model_raw, world_raw = model_path.read_bytes(), world_path.read_bytes()
    model, world = ET.fromstring(model_raw), ET.fromstring(world_raw)
    drives = [p for p in model.iter('plugin')
              if p.get('filename') == 'libgazebo_ros_diff_drive.so']
    if len(drives) != 1 or world.find('world') is None:
        raise ValueError('Expected one differential-drive plugin and one world')
    source = drives[0].find('odometry_source')
    if source is None:
        source = ET.SubElement(drives[0], 'odometry_source')
    source.text = '0'  # Gazebo ROS 3.9.0: ENCODER, integrated simulated joint velocities.
    if any(p.get('filename') == 'libgazebo_ros_state.so' for p in world.iter('plugin')):
        raise ValueError('World already has a state plugin; inspect its namespace first')
    plugin = ET.SubElement(world.find('world'), 'plugin',
                          name='localization_lab_world_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/lab/gazebo'
    ET.SubElement(plugin, 'update_rate').text = '10.0'
    directory.mkdir(parents=True, exist_ok=False)
    # Humble spawn_entity reads Unicode text before lxml parsing; an encoding
    # declaration in that string is rejected even though the XML itself is valid.
    ET.ElementTree(model).write(directory/'burger_encoder.sdf', encoding='utf-8', xml_declaration=False)
    ET.ElementTree(world).write(directory/'lab.world', encoding='utf-8', xml_declaration=False)
    manifest = {
        'schema': 1, 'simulation_only': True,
        'original_model': str(model_path), 'original_world': str(world_path),
        'original_model_sha256': hashlib.sha256(model_raw).hexdigest(),
        'original_world_sha256': hashlib.sha256(world_raw).hexdigest(),
        'odometry_source': 'ENCODER (0); simulated wheel joint velocity integration',
        'truth_source': '/lab/gazebo/get_entity_state, burger in world',
        'truth_time': 'Gazebo response header simulation time, not callback arrival time',
        'start_world_pose': [-2.0, -0.5, 0.0],
        'alignment': 'Not applied to raw bag. Align once offline; never align each frame.',
        'limits': 'Encoder odom need not visibly drift in ideal simulation. No error metrics yet.',
    }
    for name in ('burger_encoder.sdf', 'lab.world'):
        manifest[name+'_sha256'] = hashlib.sha256((directory/name).read_bytes()).hexdigest()
    (directory/'session.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return directory
