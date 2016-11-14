#!/usr/bin/env python
PKG = 'test_agrodrone'
import unittest
from src.modes.modes import Modes


class TestModes(unittest.TestCase):

    def setUp(self):
        self.modes = Modes('vehicle')

    def tearDown(self):
        self.modes = None

    def testInitalMode(self):
        self.assertEqual(self.modes.current_mode.name, 'Inactive')

    def testInitialState(self):
        self.assertEqual(self.modes.current_mode.current_state.name, 'Pending')

    def testRunInactive(self):
        self.modes.current_mode.run()
        self.assertEqual(self.modes.current_mode.current_state.name, 'Pending')

    def testModeTransition(self):
        self.modes.current_mode.run()
        self.assertNotEqual(self.modes.current_mode.name, 'RTD')
        self.modes.to_RTD()
        self.assertEqual(self.modes.current_mode.name, 'RTD')

    # RTD transitions

    def testAboveToPosition(self):
        self.modes.to_RTD()
        self.assertEqual(self.modes.current_mode.current_state.name, 'MoveAboveDock')
        self.modes.current_mode.run()
        self.assertEqual(self.modes.current_mode.current_state.name, 'PositionAboveDock')

    def testPositionToControlled(self):
        self.modes.to_RTD()
        self.modes.current_mode.to_PositionAboveDock()
        self.modes.current_mode.run()
        self.assertEqual(self.modes.current_mode.current_state.name, 'ControlledDescend')

    def testControlledToLand(self):
        self.modes.to_RTD()
        self.modes.current_mode.to_ControlledDescend()
        self.modes.current_mode.run()
        self.assertEqual(self.modes.current_mode.current_state.name, 'LandOnDock')

    def testLandToPending(self):
        self.modes.to_RTD()
        self.modes.current_mode.to_LandOnDock()
        self.modes.current_mode.run()
        self.assertEqual(self.modes.current_mode.current_state.name, 'Pending')

# def RTDsuite(self):
#     """
#     Suite for all the tests for the RTD state transition tests
#     """
#     test_suite = unittest.TestSuite()


if __name__=="__main__":
    import rosunit
    # unittest.main()
    rosunit.unitrun(PKG, 'Test the modes', TestModes)