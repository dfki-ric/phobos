import sys

if sys.version_info < (3,4):
	print ("v")
	exit(0)

try:
	import yaml
except ImportError:
	print("i")
	exit(0)
	
fullpath = yaml.__file__
print(fullpath.split("__")[0])
exit(1)
