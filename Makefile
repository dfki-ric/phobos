# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2018 University of Bremen & DFKI GmbH Robotics Innovation Center

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------------

help:
		@echo	'Targets of Phobos:'
		@echo 	'  help      - Print this help information.'
		@echo 	'  init      - Install the Phobos requirements using pip.'
		@echo	'  install   - Install the Phobos code to your Blender installation.'
		@echo	'              This also sets up the configuration folder for Phobos.'
		@echo	'  clean     - Removes the installation configuration file for Phobos.'
		@echo	'              This does not remove the Phobos installation and configurations!'
		@echo 	'  format    - Formats the python code in the folder using the black code'
		@echo 	'              formatter (github.com/ambv/black).'
		@echo 	'  apidoc    - Generates the Sphinx API documentation and moves it to the gh-pages'
		@echo 	'              branch.'

init:
		pip install -r requirements.txt

install:
		python3 setup.py

clean:
		rm python_dist_packages.conf

format:
		# add docstrings and format to google style
		pyment -o google -w .
		# add TODO tag for empty docstrings
		grep -rl --include=\*.py '""" """' | xargs -r sed -i 's/""" """/"""TODO Missing documentation"""/g'
		# apply hard formatting rules
		black -l 100 -S .

apidoc:
		@echo 'Start sphinx doc generation...'
		rm phobos/__init__.py
		cd doc && make html
		git checkout -- phobos/__init__.py
		git checkout gh-pages
		rm -rf *.html *.js *.inv _sources _static
		mv doc/_build/html/* .
		@echo 'Please commit and push the changes to publish the new doc on https://dfki-ric.github.io/phobos'

.PHONY: init test install format apidoc help
