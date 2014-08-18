Phobos Blender ADD-ON {#phobos}
===================

Phobos is an Add-On for Blender which simplifies editing robot models
and environments to be used in the MARS simulation. Phobos can also be used to visually create
urdf files for ROS. This document aims to provide an instruction to set up blender and install the phobos Add-On.

## Linux
The simplest way to install blender in a linux environment is to use 
your favourite package manager to install blender. In Ubuntu for example just type
`apt-get install blender` into a terminal with root permissions. If you want to use the newest and feature richest version of blender you can download a tarball
from the [Blender download page](http://www.blender.org/download/). Extract tis 
archive and place it where you want blender to be installed. Please 
notice that you have to have *glibc* installed on your system to run 
blender properly! 

To install phobos you have to clone the phobos git repository hosted 
on Github. For that you have to have [git](http://git-scm.com/) installed. Then just type `git 
clone https://github.com/rock-simulation/phobos.git` into your 
terminal and switch into the newly created directory named *phobos*. The 
final step is to run the shell script *install_phobos.sh* and follow 
the instructions on your terminal window. At this point it is really 
important to have [python3](https://www.python.org/) and the python module [yaml](http://www.yaml.org/) 
installed on your machine. For that purpose most of the major 
distributions offer packages for yaml (in Ubuntu for example the 
package is named `python3-yaml`). You can also use [pip](https://pypi.python.org/pypi/pip) to install 
yaml into your python environment. After this you have a fresh blender 
installation with the phobos plugin installed!

###Troubleshooting
####If you started the install script before yaml was installed...
you just have to go to `~/.config/blender/BLENDERVERSION/scripts/addons/phobos/`
and delete the yamlpath.conf. Then you can rerun the install script and 
everything will be fine.

## Windows
*will follow soon*

## Mac OS
*will follow soon*

