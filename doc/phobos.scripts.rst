phobos scripts
--------------

phobos comes with several command line utilities for commonly used tasks.

These can be used by simply calling them via the `phobos` command:

.. code-block:: rst

    phobos --help

    Usage:
    phobos COMMAND ARGUMENTS
    Commands:
        check_meshes            Checks whether all meshes are available.
        convert                 Converts the given input robot file to SDF/URDF/PDF/THUMBNAIL/SMURF/SMURFA/SMURFS.
        preprocess_cad_export   Preprocess CAD->URDF exports to use with pipeline.
        reduce_mesh             Reduces the given mesh's number of vertices by the given factor, while trying to maintain the geometry as similar as possible.
        run_pipeline            Process simulation models automatically.
        setup_git               Sets-up a git repository for a simulation/control model.
        smurfs_in_pybullet      Loads a "smurfs" file in pyBullet.
        test_model              Test the latest model using a CI-pipeline.
        check_hyrodyn           Checks whether the model can be loaded in Hyrodyn.

