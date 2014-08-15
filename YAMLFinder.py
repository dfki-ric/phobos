import sys
f = open("yamlpath.conf", "w")
f.truncate()#Empty the file
#if sys.version_info < (3,4):
#	f.write("v")
#	f.close()
#	print("You have to use python 3.4 or greater to install the correct version of yaml")
#	exit(0)

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
