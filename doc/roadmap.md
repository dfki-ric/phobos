# Phobos Roadmap

## 1.0

This is going to be the first ever 'release', resulting in a newly created branch 'stable'.

| branch  | designated push date |
| ------------- | ------------- |
| next  | 08.10.2014  |
| stable | 10.10.2014  |

### Features still incomplete or missing

- import of MARS scenes
- Documentation
    - general workflow
    - cheat sheet
    - example scene
    - internal dictionary structure

### Issues

- refactor import (classes?)
- calculate inertia of entire link without re-calculating individual inertias 
- check if decimal place rounding still works with inertia tensors (exception?) or should be different
- inertias are not exported if visuals are not selected (?)
