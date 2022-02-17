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

    class TestInertiaModel(unittest.TestCase):

        def test_calculateBoxInertia(self):
            mass = 12
            size = [1, 2, 3]
            result = (13., 0., 0., 10., 0., 5.)

            self.assertTupleEqual(phobos.model.inertia.calculateBoxInertia(mass, size), result)

        def test_calculateCylinderInertia(self):
            mass = 12
            radius = 2
            height = 5
            result = (37., 0., 0., 37., 0., 24.)

            self.assertTupleEqual(
                phobos.model.inertia.calculateCylinderInertia(mass, radius, height), result)

        def test_calculateSphereInertia(self):
            mass = 12
            radius = 2
            target = [19.2, 0., 0., 19.2, 0., 19.2]

            # we need to round the result, as machine precision fails us here...
            result = phobos.model.inertia.calculateSphereInertia(mass, radius)
            result = [round(val, 1) for val in result]
            self.assertEqual(result, target)

        def test_calculateEllipsoidInertia(self):
            mass = 5
            size = [1, 2, 3]
            result = (13, 0, 0, 10, 0, 5)

            self.assertTupleEqual(phobos.model.inertia.calculateEllipsoidInertia(mass, size),
                                  result)

        def test_inertiaListToMatrix(self):
            inertialist = [1, 2, 3, 4, 5, 6]
            result = [[1., 2., 3.], [2., 4., 5.], [3., 5., 6.]]

            self.assertListEqual(
                list(list(elem) for elem in phobos.model.inertia.inertiaListToMatrix(inertialist)),
                result)

        def test_inertiaMatriToList(self):
            matrix = [[1, 2, 3], [2, 4, 5], [3, 5, 6]]
            result = [1, 2, 3, 4, 5, 6]

            self.assertListEqual(list(phobos.model.inertia.inertiaMatrixToList(matrix)), result)

            # TODO continue with joints

    # we have to manually invoke the test runner here, as we cannot use the CLI
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(TestInertiaModel)
    success = unittest.TextTestRunner().run(suite)

    if success.errors or success.failures:
        raise Exception

except Exception:
    sys.exit(1)
