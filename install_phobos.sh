#! /bin/sh

#author: Kai von Szadkowski, Ole Schwiegert
#
#This script will install phobos into your user bound python addon repository.
#It also provides a way for blenders python to import your python3s yaml module.

#If you used the old install script you can delete the previous phobos folder
#You also have to delete the old installconfig.txt to run this script correctly.



###Config file name###
file="installconfig.txt"
###Phobos install flag###
installed=0

#####
writeConfig() {
    #Creates the config and writes it to installconfig.txt
    echo "Enter your blender version (e.g. '2.71')"
    read blenderversion
    echo "blenderversion=$blenderversion" >>installconfig.txt
    echo "Do you want to use your python3 yaml library or do you want to use the yaml version
    distributed with phobos? (y - use python3 library / n - use distributed package)"
    read YN
    case $YN in
        y|Y )
            useP='yes'
            echo "useP=$useP" >>installconfig.txt
            echo "Please enter the command to run your python3 binary (e.g python3 or /usr/bin/python3)"
            read pythoncom
            pythoncom=$pythoncom
            echo "pythoncom=$pythoncom" >>installconfig.txt
            ;;
        n|N )
            useP='no'
            echo "useP=$useP" >>installconfig.txt;;
    esac
    
    #Getting OS
    ###BEGIN PYTHON SNIPPET###
    python << END
import platform
f = open("installconfig.txt", 'a')
osys = platform.system()
if osys == "Linux":
    f.write("addonpath=$HOME/.config/blender/$blenderversion/scripts/addons")
elif osys == "Darwin":
    f.write("addonpath=$HOME/Library/Application\ Support/Blender/$blenderversion/scripts/addons")
f.close()
END
    ###END PYTHON SNIPPET###
}
#####

#####
initConfig() {
    if [ -r $file ]
    then
        echo "Found install config file."
        . ./$file
    else
        writeConfig
    fi
    . ./$file
}

installPhobos() {
    echo "Check for phobos installation..."
    phobospath="$addonpath/phobos"
    if [ -d $phobospath ]
    then
        echo "Phobos installation found and updated."
        cp *.py $phobospath
        installed=1
    else
        echo "Phobos folder does not exist, create phobos folder in $addonpath ? (y/n)"
        read YN
        case $YN in
            y|Y )
                mkdir -p $phobospath
                cp *.py $phobospath
                installed=1
                echo "Copied phobos to $phobospath"
                ;;
            n|N ) echo "No folder for phobos created";;
        esac
    fi
}
#####

#####
installYAML() {
    if [ "$useP" = "yes" ]
    then
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
                $pythoncom << END
import sys
import os
dirs = sys.path
yamlpath="i"
for directory in dirs:
    if os.path.isdir(directory):
        modules = os.listdir(directory)
        for module in modules:
            if module == "yaml":
                yamlpath=directory
                print ("YAML module found!")
f = open("yamlpath.conf", 'w')
f.truncate()#Empty the file
f.write(yamlpath)
f.close()
exit(1)
END
                ###END PYTHON SNIPPET###
                cp yamlpath.conf $phobospath/yamlpath.conf
                echo "yamlpath.conf created and copied"
                ;;
            n|N )
                echo "yamlpath.conf wasn't created.";;
            esac
        fi
    else
        cp -R yaml $phobospath
        echo "YAML package was copied into $phobospath."
    fi
}
#####

#####MAIN SCRIPT#####
initConfig
installPhobos
if [ "$installed" -eq "1" ]
then
    installYAML
else
    echo "Cannot install YAML, because Phobos wasn't installed."
fi
