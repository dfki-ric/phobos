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

#phobos
echo "Checking phobos installation..."
phobosfolder="$blenderpath/$blenderversion/scripts/addons/phobos/"
if [ ! -d $phobosfolder ]
then
    echo "phobos folder does not exist, create phobos folder in $phobosfolder ? (y/n)"
    read YN
    case $YN in
        y|Y )
            mkdir $phobosfolder 
            cp *.py $phobosfolder
            echo "Copied phobos to $phobosfolder"
            ;;
        n|N ) echo "No folder for phobos created";;
    esac
else
    cp *.py $phobosfolder
    echo "Copied phobos to $phobosfolder"
fi

#YAML new version
#TODO: I think it doesn't work when there is no phobos folder
echo "Checking for yamlpath.conf"
blub="$phobosfolder/yamlpath.conf"
if [ -r $blub ]
then
	echo "yamlpath found! Done."
else
	echo "Do you want to create your yamlpath.conf? (y/n)"
	read YN
	case $YN in
	y|Y )
		message=`python3 YAMLFinder.py | tail -n 1`
		echo $message
		cp yamlpath.conf $phobosfolder/yamlpath.conf
		echo "yamlpath.conf created and copied"
		;;
	n|N )
		echo "yamlpath.conf wasn't created.";;
	esac
fi

#YAML
#echo "Checking YAML installation..."
#TODO:Set python correct python folder for different blender versions
#yamlfolder="$blenderpath/$blenderversion/python/lib/python3.4/yaml/"
#prevents the yaml folder in the yaml folder
#pythonfolder="$blenderpath/$blenderversion/python/lib/python3.4/"
#if [ ! -d $yamlfolder ]
#then
#    echo "YAML installation does not exist, link YAML installation to $yamlfolder ? (y/n)"
#    read YN
#    case $YN in
#        y|Y )
			#TODO just works if python3 is in path..maybe ask for python binary??
			#TODO integrate external YAMLFinder.py into this script
#			yamlpath=`python3 YAMLFinder.py | tail -n 1`
#			echo $yamlpath
#			if [ -d $yamlpath ]
#			then
#				cp -R $yamlpath $pythonfolder
#				echo "Linked YAML to $yamlfolder"
#			else
#				echo "There was no YAML installation found or the python version is wrong"
#				echo "Do you want to install YAML with the old method an copy static yaml files into blenders python libs? (y/n)"
#				read YN
#				case $YN in
#					y|Y )
#						cp yamlbak $yamlfolder;;
#					n|N ) echo "YAML wasn't installed at all";;
#				esac
#			fi
 #           ;;
  #      n|N ) echo "No folder for YAML created";;
   # esac
#else
#    echo "Found existing YAML folder, no files copied. Please check for latest version yourself."
#fi

#TODO: add yaml package properly after checking if it needs any c binaries
#echo "Copying yaml package to $blenderpath/$blenderversion/python/"
#cp ./yaml/ $blenderpath/$blenderversion/python/

#the following lines are only kept for reference until the script is working on all systems
#cp ~/limes-dev/mars/scripts/blender/phobos/*.py ~/.blender/2.62/scripts/addons/phobos/
#cp ~/limes-dev/mars/scripts/blender/phobos/*.py ~/.blender/2.69/scripts/addons/phobos/
#cp ~/limes-dev/mars/scripts/blender/phobos/*.py ~/tools/blender-2.69/2.69/scripts/addons/phobos/



