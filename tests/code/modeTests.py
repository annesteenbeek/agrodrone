#!/usr/bin/env python
PKG = 'test_agrodrone'
import unittest
import rospy
from src.modes.modes import Modes


class TestModes(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node')
        self.modes = Modes('vehicle')

    def tearDown(self):
        self.modes.set_mode_service.shutdown()
        # self.modes.mode_pub.shutdown()
        self.modes = None

    def testInitalMode(self):
        self.assertEqual(self.modes.cur_mode.name, 'Inactive')

    def testInitialState(self):
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'Pending')

    def testRunInactive(self):
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'Pending')

    def testModeTransition(self):
        self.modes.cur_mode.run()
        self.assertNotEqual(self.modes.cur_mode.name, 'RTD')
        self.modes.to_RTD()
        self.assertEqual(self.modes.cur_mode.name, 'RTD')

    # RTD transitions

    def testOffboardToMove(self):
        self.modes.to_RTD()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'SetToOffboard')
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'MoveAboveDock')

    def testAboveToPosition(self):
        self.modes.to_RTD()
        self.modes.cur_mode.to_MoveAboveDock()
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'PositionAboveDock')

    def testPositionToControlled(self):
        self.modes.to_RTD()
        self.modes.cur_mode.to_PositionAboveDock()
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'ControlledDescend')

    def testControlledToLand(self):
        self.modes.to_RTD()
        self.modes.cur_mode.to_ControlledDescend()
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'LandOnDock')

    def testLandToPending(self):
        self.modes.to_RTD()
        self.modes.cur_mode.to_LandOnDock()
        self.modes.cur_mode.run()
        self.assertEqual(self.modes.cur_mode.cur_state.name, 'Pending')


# Service test
#
#     def testSwitchModeService(self):
#         #TODO create service test, check by messages
#         pass


# def RTDsuite(self):
#     """
#     Suite for all the tests for the RTD state transition tests
#     """
#     test_suite = unittest.TestSuite()


if __name__=="__main__":
    import rosunit
    # unittest.main()
    rosunit.unitrun(PKG, 'Test the modes', TestModes)