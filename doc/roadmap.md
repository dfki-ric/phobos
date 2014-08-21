# Phobos Roadmap

## 0.5

This is going to be the first ever 'release', resulting in a newly created branch 'stable'.

| branch  | push date |
| ------------- | ------------- |
| next  | 15.09.2014  |
| stable | 18.09.2014  |

### Features still missing

- import of MARS scenes
- urdf import:
    - set joint constraints
    - set axes correctly
- documentation of basic features

### Issues

- installation has to work for local and global Blender, as well as for Blender import
- refactor import (classes?)
- calculate inertia of entire link without re-calculating individual inertias 
- check if decimal place rounding still works with inertia tensors (exception?) or should be different
- inertias are not exported if visuals are not selected
