#!/usr/bin/python3

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import glob
import subprocess
import sys

blenderExecutable = 'blender'

# allow override of blender executable (important for CI!)
if len(sys.argv) > 1:
    blenderExecutable = sys.argv[1]

# run all tests before aborting build
testfailed = False

# iterate over each *.test.blend file in the "tests" directory
# and open up blender with the .test.blend file and the corresponding .test.py python script
for file in glob.glob('tests/**/*.test.blend'):
    print('#' * 100)
    print('Running {} tests...'.format(file))
    print('#' * 100)
    code = subprocess.call([blenderExecutable, '--addons', 'phobos', '--factory-startup',
                            '-noaudio', '-b', file, '--python', file.replace('.blend', '.py'),
                            '--python-exit-code', '1'])
    print('#' * 100)
    print("Exited with: ", code)
    print('#' * 100 + '\n\n\n')
    if code:
        testfailed = True

if testfailed:
    sys.exit(1)
