#! /bin/sh

#author: Kai von Szadkowski, Ole Schwiegert
#
#This script will install phobos into your user bound python addon repository.
#It also provides a way for blenders python to import your python3s yaml module.

#If you used the old install script you can delete the previous phobos folder
#You also have to delete the old installconfig.txt to run this script correctly.

file="installconfig.txt"
if [ -r $file ]
then
    echo "Found install config file."
    . ./$file
else
    echo "Enter your blender version (e.g. '2.71')"
    read blenderversion
    echo "blenderversion=$blenderversion" >>installconfig.txt
    echo "Please enter the command to run your python3 binary (e.g python3 or /usr/bin/python3)"
    read pythoncom
    alias pythoncom='$pythoncom'
    echo "alias pythoncom='$pythoncom'" >>installconfig.txt
    addonpath="$HOME/.config/blender/$blenderversion/scripts/addons"
    echo "addonpath=$addonpath" >>installconfig.txt
fi

#Phobos installation
echo "Check for phobos installation..."
phobospath="$addonpath/phobos"
if [ -d $phobospath ]
then
	echo "Phobos installation found and updated."
	cp *.py $phobospath
else
	echo "Phobos folder does not exist, create phobos folder in $addonpath ? (y/n)"
    read YN
    case $YN in
        y|Y )
            mkdir -p $phobospath
            cp *.py $phobospath
            echo "Copied phobos to $phobospath"
            ;;
        n|N ) echo "No folder for phobos created";;
    esac
fi

#YAML new version
#TODO: I think it doesn't work when there is no phobos folder
echo "Checking for yamlpath.conf"
yamlpath="$phobospath/yamlpath.conf"
if [ -r $yamlpath ]
then
	echo "yamlpath.conf found! Done."
else
	echo "Do you want to create your yamlpath.conf? (y/n)"
	read YN
	case $YN in
	y|Y )
	
	###BEGIN PYTHON SNIPPET###
		pythoncom << END
import sys
f = open("yamlpath.conf", "w")
f.truncate() #Empty the file
try:
	import yaml
except ImportError:
	f.write("i")
	f.close()
	print("There was no YAML module in the current python version. Please install it with your favorite package manager")
	exit(0)
	
fullpath = yaml.__file__
f.write(fullpath.split("yaml")[0])
f.close()
print("YAMl module found!")
exit(1)
END
###END PYTHON SNIPPET###
		#delete the bash alias for python3 binary
		unalias pythoncom
		cp yamlpath.conf $phobospath/yamlpath.conf
		echo "yamlpath.conf created and copied"
		;;
	n|N )
		echo "yamlpath.conf wasn't created.";;
	esac
fi
