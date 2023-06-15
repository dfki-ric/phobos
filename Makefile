# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

help:
		@echo	'Targets of Phobos:'
		@echo 	'  apidoc    - Generates the Sphinx API documentation and moves'
		@echo	'              it to the gh-pages branch.'
		@echo	'  clean     - Removes the installation configuration file for Phobos.'
		@echo	'              This does not remove the Phobos installation and configurations!'
		@echo 	'  format    - Formats the python code in the folder using the black code'
		@echo 	'              formatter (github.com/ambv/black).'
		@echo 	'  help      - Print this help information.'
		@echo	'  install   - Install the Phobos code to your Blender installation.'
		@echo	'              This also sets up the configuration folder for Phobos.'
		@echo	'  version   - Prints some help relating to drafting a new version.'

install:
		pip3 install .

clean:
		rm installation.conf

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
		cd doc && make clean && make html
		cd doc/_build/html && grep -rl --include=\*.html 'Created using' | xargs -r sed -i '/Created using/ s/$$/\&nbsp\&nbsp\&nbsp<a\ href=\"https:\/\/help\.github.com\/articles\/github-privacy-statement\/\">Privacy<\/a>/'
		git checkout -- phobos/__init__.py
		git checkout gh-pages
		rm -rf *.html *.js *.inv _sources _static
		mv doc/_build/html/* .
		@echo 'Please commit and push the changes to publish the new doc on https://dfki-ric.github.io/phobos'

version:
		@echo 'Change version in:'
		@echo '  - codemeta.json'
		@echo '  - phobos/defs.py'
		@echo '  - phobos/__init__.py'
		@echo '  - doc/conf.py'
		@echo '  - doc/index.rst'

.PHONY: init test install format apidoc help
