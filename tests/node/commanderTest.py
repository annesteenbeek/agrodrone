#!/usr/bin/env python
PKG = 'agrodrone'

import unittest
from src.nodes.commander import CommanderNode
from src.lib.vehicle import Vehicle


class TestCommander(unittest.TestCase):
    def test_commander(self):
        vehicle = Vehicle()
        commander = CommanderNode(vehicle)
        commander.run()
        self.assertEquals(1, 1, "1!=1")

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, 'test_commander', TestCommander)
