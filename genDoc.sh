#!/bin/bash

echo "Start sphinx doc generation..."
rm phobos/__init__.py
cd doc
make html
cd ..
git checkout -- phobos/__init__.py
git checkout gh-pages
rm -rf *.html *.js *.inv _sources _static
mv doc/_build/html/* .
echo "Please commit and push the changes to publish the new doc on https://rock-simulation.github.io/phobos"

