import os
import sys
import unittest

import install_test_helper


class TestInstall(unittest.TestCase):
    def test_install(self):
        """
        Tests enumerated binaries in installation environment.
        """
        # Fail (on purpose) if not given exactly one command line argument.
        self.assertEqual(len(sys.argv), 2)
        with open(sys.argv[1], 'r') as f:
            lines = f.readlines()

        # Install into bazel read-only temporary directory. The temporary
        # directory does not need to be removed as bazel tests are run in a
        # scratch space.
        install_test_helper.install()
        install_dir = install_test_helper.get_install_dir()

        # Verify install directory content.
        self.assertTrue(os.path.isdir(install_dir))
        content = set(os.listdir(install_dir))
        self.assertSetEqual(set(['bin', 'include', 'lib', 'share']), content)

        # Execute the install actions.
        for cmd in lines:
            cmd = cmd.strip()
            print("+ {}".format(cmd))
            install_test_helper.check_call([os.path.join(os.getcwd(), cmd)])


if __name__ == '__main__':
    sys.stdout = sys.stderr
    unittest.main(argv=["TestInstall"])
