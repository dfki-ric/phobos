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

echo "Copying marstools to $blenderpath/$blenderversion/scripts/addons/marstools/"
if [ ! -d "$blenderpath/$blenderversion/scripts/addons/marstools/" ]
then
    echo "marstools folder does not exist, create marstools folder in $blenderpath/$blenderversion/scripts/addons/marstools/ ? (y/n)"
    read YN
    case $YN in
        y|Y ) mkdir $blenderpath/$blenderversion/scripts/addons/marstools/ ;;
        n|N ) echo "No folder created";;
    esac
fi
if [ -d "$blenderpath/$blenderversion/scripts/addons/marstools/" ]
then
    cp *.py $blenderpath/$blenderversion/scripts/addons/marstools/
    echo "Marstools Installation completed."
else
    echo "Installation failed."
fi

#TODO: add yaml package properly after checking if it needs any c binaries
#echo "Copying yaml package to $blenderpath/$blenderversion/python/"
#cp ./yaml/ $blenderpath/$blenderversion/python/

#the following lines are only kept for reference until the script is working on all systems
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/.blender/2.62/scripts/addons/marstools/
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/.blender/2.69/scripts/addons/marstools/
#cp ~/limes-dev/mars/scripts/blender/marstools/*.py ~/tools/blender-2.69/2.69/scripts/addons/marstools/



