#!/usr/bin/env python

import unittest
from inspect import isclass


class TestImport(unittest.TestCase):

    def test_import(self):
        """
        just tests that the module can be correctly imported
        """
        from pygame_managed_player.pygame_player import PyGamePlayer

        self.assertTrue(isclass(PyGamePlayer))

if __name__ == '__main__':
    import rostest
    PKG = 'pygame_managed_player'
    rostest.rosrun(PKG, 'test_import', TestImport)
