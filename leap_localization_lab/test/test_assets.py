import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from leap_localization_lab.assets import prepare_assets


class AssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.model = self.root/'original.sdf'
        self.world = self.root/'original.world'
        self.model.write_text('<sdf version="1.6"><model name="burger"><link name="base_footprint"/>'
                              '<plugin name="drive" filename="libgazebo_ros_diff_drive.so">'
                              '<wheel_separation>0.16</wheel_separation><odometry_source>1</odometry_source>'
                              '</plugin></model></sdf>')
        self.world.write_text('<sdf version="1.6"><world name="default">'
                              '<include><uri>model://ground_plane</uri></include></world></sdf>')

    def test_encoder_copy_preserves_originals_and_physical_geometry(self):
        before = self.model.read_bytes(), self.world.read_bytes()
        directory = prepare_assets(self.model, self.world, self.root/'session')
        copied = ET.parse(directory/'burger_encoder.sdf')
        self.assertEqual(copied.findtext('.//odometry_source'), '0')
        self.assertEqual(copied.findtext('.//wheel_separation'), '0.16')
        self.assertIsNotNone(copied.find('.//link[@name="base_footprint"]'))
        self.assertFalse((directory/'burger_encoder.sdf').read_text().lstrip().startswith('<?xml'))
        self.assertEqual(before, (self.model.read_bytes(), self.world.read_bytes()))

    def test_model_text_is_accepted_by_spawn_entity_parser(self):
        try:
            from lxml import etree
        except ImportError:
            self.skipTest('lxml is supplied by the Ubuntu Gazebo environment')
        directory = prepare_assets(self.model, self.world, self.root/'session')
        parsed = etree.fromstring((directory/'burger_encoder.sdf').read_text())
        self.assertEqual(parsed.find('model').get('name'), 'burger')

    def test_new_source_setting_when_upstream_omits_it(self):
        self.model.write_text(self.model.read_text().replace('<odometry_source>1</odometry_source>', ''))
        directory = prepare_assets(self.model, self.world, self.root/'session')
        self.assertEqual(ET.parse(directory/'burger_encoder.sdf').findtext('.//odometry_source'), '0')

    def test_world_state_namespace_and_saved_provenance(self):
        directory = prepare_assets(self.model, self.world, self.root/'session')
        copied = ET.parse(directory/'lab.world')
        self.assertEqual(copied.findtext('.//include/uri'), 'model://ground_plane')
        self.assertEqual(copied.findtext('.//plugin/ros/namespace'), '/lab/gazebo')
        metadata = json.loads((directory/'session.json').read_text())
        self.assertEqual(metadata['original_model_sha256'], hashlib.sha256(self.model.read_bytes()).hexdigest())
        self.assertTrue(metadata['simulation_only'])

    def test_refuses_duplicate_world_source_and_preserves_existing_session(self):
        self.world.write_text('<sdf><world name="default"><plugin filename="libgazebo_ros_state.so"/></world></sdf>')
        with self.assertRaisesRegex(ValueError, 'already has a state plugin'):
            prepare_assets(self.model, self.world, self.root/'new')
        self.assertFalse((self.root/'new').exists())
        self.world.write_text('<sdf><world name="default"/></sdf>')
        directory = prepare_assets(self.model, self.world, self.root/'session')
        manifest = (directory/'session.json').read_bytes()
        with self.assertRaises(FileExistsError):
            prepare_assets(self.model, self.world, directory)
        self.assertEqual((directory/'session.json').read_bytes(), manifest)

    def test_rejects_unrecognized_model_or_world_without_writing(self):
        for model_text, world_text in (
                ('<sdf><model name="burger"/></sdf>', self.world.read_text()),
                (self.model.read_text(), '<sdf/>')):
            self.model.write_text(model_text)
            self.world.write_text(world_text)
            with self.assertRaisesRegex(ValueError, 'Expected one differential-drive'):
                prepare_assets(self.model, self.world, self.root/'invalid')
            self.assertFalse((self.root/'invalid').exists())


if __name__ == '__main__':
    unittest.main()
