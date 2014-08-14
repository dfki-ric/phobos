#! /bin/sh

file="installconfig.txt"
if [ -r $file ]
then
    echo "Found install config file."
    . ./$file
else
    echo "Enter path to your blender installation:"
    read blenderpath
    echo "Enter your blender version (e.g. '2.69')"
    read blenderversion
    echo "blenderpath=$blenderpath" >>installconfig.txt
    echo "blenderversion=$blenderversion" >>installconfig.txt
fi

#marstools
echo "Checking marstools installation..."
marsfolder="$blenderpath/$blenderversion/scripts/addons/marstools/"
if [ ! -d $marsfolder ]
then
    echo "marstools folder does not exist, create marstools folder in $marsfolder ? (y/n)"
    read YN
    case $YN in
        y|Y )
            mkdir $marsfolder 
            cp *.py $marsfolder
            echo "Copied marstools to $marsfolder"
            ;;
        n|N ) echo "No folder for marstools created";;
    esac
else
    cp *.py $marsfolder
    echo "Copied marstools to $marsfolder"
fi

#YAML
echo "Checking YAML installation..."
#TODO:Set python correct python folder for different blender versions
yamlfolder="$blenderpath/$blenderversion/python/lib/python3.4/yaml/"
#prevents the yaml folder in the yaml folder
pythonfolder="$blenderpath/$blenderversion/python/lib/python3.4/"
if [ ! -d $yamlfolder ]
then
    echo "YAML installation does not exist, link YAML installation to $yamlfolder ? (y/n)"
    read YN
    case $YN in
        y|Y )
			#TODO just works if python3 is in path..maybe ask for python binary??
			#TODO integrate external YAMLFinder.py into this script
			yamlpath=`python3 YAMLFinder.py | tail -n 1`
			echo $yamlpath
			if [ -d $yamlpath ]
			then
				cp -R $yamlpath $pythonfolder
				echo "Linked YAML to $yamlfolder"
			else
				echo "There was no YAML installation found or the python version is wrong"
			fi
            ;;
        n|N ) echo "No folder for YAML created";;
    esac
else
    echo "Found existing YAML folder, no files copied. Please check for latest version yourself."
fi

#TODO: add yaml package properly after checking if it needs any c binaries
#echo "Copying yaml package to $blenderpath/$blenderversion/python/"
#cp ./yaml/ $blenderpath/$blenderversion/python/

#the following lines are only kept for reference until the script is working on all systems
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/.blender/2.62/scripts/addons/marstools/
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/.blender/2.69/scripts/addons/marstools/
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/tools/blender-2.69/2.69/scripts/addons/marstools/



