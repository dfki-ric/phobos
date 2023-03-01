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
    import mathutils as mathutils
    import phobos.blender as phobos

    class TestBlenderUtils(unittest.TestCase):

        def test_compileEnumPropertyList(self):
            testlist = phobos.utils.blender.compileEnumPropertyList([1, 2, 3])
            self.assertTupleEqual(tuple(testlist), ((1, 1, 1), (2, 2, 2), (3, 3, 3)))

    class TestGeneralUtils(unittest.TestCase):

        def test_is_float(self):
            self.assertTrue(phobos.utils.general.is_float('12.3'))
            self.assertFalse(phobos.utils.general.is_float('hello'))
            self.assertFalse(phobos.utils.general.is_float('[12.3, 3.2]'))

        def test_is_int(self):
            self.assertTrue(phobos.utils.general.is_int('12'))
            self.assertFalse(phobos.utils.general.is_int('hello'))
            self.assertFalse(phobos.utils.general.is_int('12.3452234'))
            self.assertFalse(phobos.utils.general.is_int('[12, 3]'))

        def test_parse_number(self):
            number = phobos.utils.general.parse_number('12')
            self.assertEqual(type(number), type(1))
            number = phobos.utils.general.parse_number('12.124')
            self.assertEqual(type(number), type(12.124))
            number = phobos.utils.general.parse_number('hello')
            self.assertEqual(type(number), type('hello'))

        def test_only_contains_int(self):
            testlist = [1, 2, 3, 4, 5]
            self.assertTrue(phobos.utils.general.only_contains_int(testlist))
            testlist = [1, 2, 3, 4, 5, 12.3]
            self.assertTrue(phobos.utils.general.only_contains_int(testlist))
            testlist = [1, 2, 3, 4, 5, 'hello']
            self.assertFalse(phobos.utils.general.only_contains_int(testlist))

        def test_only_contains_float(self):
            testlist = [1.1, 2.1, 3.5, 4.7, 5.6]
            self.assertTrue(phobos.utils.general.only_contains_float(testlist))
            testlist = [1.1, 2.1, 3.5, 4.7, 5.6, 12]
            self.assertTrue(phobos.utils.general.only_contains_float(testlist))
            testlist = [1.1, 2.1, 3.5, 4.7, 5.6, 'hello']
            self.assertFalse(phobos.utils.general.only_contains_float(testlist))

        def test_parse_text(self):
            teststring = '1 2 3 4 5'
            self.assertListEqual(phobos.utils.general.parse_text(teststring), [1, 2, 3, 4, 5])
            teststring = '1.1 2.1 3.4 4.7 5.1'
            self.assertListEqual(phobos.utils.general.parse_text(teststring), [
                1.1, 2.1, 3.4, 4.7, 5.1])
            teststring = 'a bc de hello whatsoever'
            self.assertListEqual(phobos.utils.general.parse_text(teststring), [
                'a', 'bc', 'de', 'hello', 'whatsoever'])
            teststring = '12.3'
            self.assertEqual(phobos.utils.general.parse_text(teststring), 12.3)
            teststring = '12'
            self.assertEqual(phobos.utils.general.parse_text(teststring), 12)

        def test_calcBoundingBoxCenter(self):
            corners = [
                [0., 0., 0.],
                [1., 0., 0.],
                [1., 0., 1.],
                [0., 0., 1.],
                [0., 1., 0.],
                [1., 1., 0.],
                [1., 1., 1.],
                [0., 1., 1.]
            ]
            self.assertEqual(phobos.utils.general.calcBoundingBoxCenter(corners),
                             mathutils.Vector([0.5, 0.5, 0.5]))

            corners = [
                [-1., -1., -1.],
                [1., -1., -1.],
                [1., -1., 1.],
                [-1., -1., 1.],
                [-1., 1., -1.],
                [1., 1., -1.],
                [1., 1., 1.],
                [-1., 1., 1.]
            ]
            self.assertEqual(phobos.utils.general.calcBoundingBoxCenter(corners),
                             mathutils.Vector([0., 0., 0.]))

        def test_sortListsInDict(self):
            testdict = {'a': 1, 'b': [15, 12, 4, -3], 'c': ['delta', 'gamma', 'yota', 'omega'],
                        'd': 'hello', 'e': [12.3, 14, 'meh']}
            targetdict = {'a': 1, 'b': [15, 12, 4, -3], 'c': ['delta', 'gamma', 'omega', 'yota'],
                          'd': 'hello', 'e': [12.3, 14, 'meh']}
            self.assertEqual(phobos.utils.general.sortListsInDict(testdict), targetdict)
            testdict = ['omega', 'delta', 'alpha', 'eta']
            targetdict = ['alpha', 'delta', 'eta', 'omega']
            self.assertListEqual(phobos.utils.general.sortListsInDict(testdict), targetdict)
            testdict = ['omega', 'delta', 'alpha', 'eta']
            targetdict = ['omega', 'eta', 'delta', 'alpha']
            self.assertListEqual(phobos.utils.general.sortListsInDict(testdict, reverse=True),
                                 targetdict)
            # TODO adjust docstring to include string only?

        def test_roundFloatsInDict(self):
            testdict = {'a': 1, 'b': [12.545, -3.111, -3.894, 15.25, -0.111],
                        'c': ['delta', 'alpha', 'gamma']}
            targetdict = {'a': 1, 'b': [13., -3., -4., 15., 0.], 'c': ['delta', 'alpha', 'gamma']}
            self.assertEqual(phobos.utils.general.roundFloatsInDict(testdict, 0), targetdict)
            targetdict = {'a': 1, 'b': [12.5, -3.1, -3.9, 15.2, -0.1],
                          'c': ['delta', 'alpha', 'gamma']}
            self.assertEqual(phobos.utils.general.roundFloatsInDict(testdict, 1), targetdict)
            targetdict = {'a': 1, 'b': [12.54, -3.11, -3.89, 15.25, -0.11],
                          'c': ['delta', 'alpha', 'gamma']}
            self.assertEqual(phobos.utils.general.roundFloatsInDict(testdict, 2), targetdict)

        def test_datetimeFromIso(self):
            # TODO add test case for ISO time
            pass

        def test_outerProduct(self):
            a = [0, 1, 2]
            b = [3, 4, 5]
            result = [[0., 0., 0.], [3., 4., 5.], [6., 8., 10.]]
            self.assertListEqual(list(list(elem) for elem in
                                      phobos.utils.general.outerProduct(a, b)), result)

    class TestNamingUtils(unittest.TestCase):

        def test_getUniqueName(self):
            testname = 'bert'
            testlist = ['gunther', 'bert', 'walter']
            target = 'bert.000'
            self.assertEqual(phobos.utils.naming.getUniqueName(testname, testlist), target)

            testname = 'b' * 70
            testlist = ['gunther', 'b' * 70, 'walter']
            target = 'b' * 59 + '.000'
            self.assertEqual(phobos.utils.naming.getUniqueName(testname, testlist), target)

        def test_isValidModelname(self):
            testname = 'helloworld'
            self.assertTrue(phobos.utils.naming.isValidModelname(testname))
            testname = 'asnd_123_ppja-'
            self.assertTrue(phobos.utils.naming.isValidModelname(testname))
            testname = '124%£DJFLL:'
            self.assertFalse(phobos.utils.naming.isValidModelname(testname))
            testname = ''
            self.assertFalse(phobos.utils.naming.isValidModelname(testname))
            testname = ' '
            self.assertFalse(phobos.utils.naming.isValidModelname(testname))

        def test_addNamespaceToName(self):
            testname = 'bert'
            namespace = 'robot'
            target = 'robot::bert'
            self.assertEqual(phobos.utils.naming.addNamespaceToName(testname, namespace), target)
            testname = 'robot::bert'
            namespace = 'model'
            target = 'model::robot::bert'
            self.assertEqual(phobos.utils.naming.addNamespaceToName(testname, namespace), target)

        def test_stripNamespaceFromName(self):
            testname = 'robot::bert'
            target = 'bert'
            self.assertEqual(phobos.utils.naming.stripNamespaceFromName(testname), target)
            testname = 'model::robot::bert'
            target = 'bert'
            self.assertEqual(phobos.utils.naming.stripNamespaceFromName(testname), target)
            testname = 'robot::'
            target = ''
            self.assertEqual(phobos.utils.naming.stripNamespaceFromName(testname), target)
            testname = '::'
            target = ''
            self.assertEqual(phobos.utils.naming.stripNamespaceFromName(testname), target)
            testname = ''
            target = ''
            self.assertEqual(phobos.utils.naming.stripNamespaceFromName(testname), target)

    # we have to manually invoke the test runner here, as we cannot use the CLI
    blenderutilstest = unittest.defaultTestLoader.loadTestsFromTestCase(TestBlenderUtils)
    generalutilstest = unittest.defaultTestLoader.loadTestsFromTestCase(TestGeneralUtils)
    namingutilstest = unittest.defaultTestLoader.loadTestsFromTestCase(TestNamingUtils)

    results = []
    results.append(unittest.TextTestRunner().run(blenderutilstest))
    results.append(unittest.TextTestRunner().run(generalutilstest))
    results.append(unittest.TextTestRunner().run(namingutilstest))

    for result in results:
        if result.errors or result.failures:
            raise Exception
except Exception:
    sys.exit(1)
