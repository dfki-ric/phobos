import sys
import unittest

try:
    import phobos

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
