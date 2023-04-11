#!/usr/bin/python3

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import sys
import unittest

try:
    import phobos.blender as phobos

    class TestPhobosAddon(unittest.TestCase):

        def test_addon_enabled(self):
            # test if addon got loaded correctly
            # every addon must provide the "bl_info" dict
            self.assertIsNotNone(phobos.bl_info)

    # we have to manually invoke the test runner here, as we cannot use the CLI
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestPhobosAddon)
    success = unittest.TextTestRunner().run(suite)

    if success.errors or success.failures:
        raise Exception
except Exception:
    sys.exit(1)
