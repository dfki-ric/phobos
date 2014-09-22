# Phobos Roadmap

## 1.0

This is going to be the first ever 'release', resulting in a newly created branch 'stable'.

| branch  | push date |
| ------------- | ------------- |
| next  | 24.09.2014  |
| stable | 01.10.2014  |

### Features still incomplete or missing

- import of MARS scenes
- urdf import:
    - set joint constraints
    - set axes correctly
    - set layers correctly
- documentation of basic features

### Issues

- installation has to work for local and global Blender, as well as for Blender import (YAML installation?)
- refactor import (classes?)
- calculate inertia of entire link without re-calculating individual inertias 
- check if decimal place rounding still works with inertia tensors (exception?) or should be different
- inertias are not exported if visuals are not selected
