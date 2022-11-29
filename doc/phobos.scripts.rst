phobos scripts
--------------

phobos comes with several command line utilities for commonly used tasks.

These can be used by simply calling them via the `phobos` command:

.. code-block:: rst
    phobos --help

    Usage:
    phobos COMMAND ARGUMENTS
    Commands:
           assemble_smurfa         Loads a SMURFS/SMURFA file and exports the urdf(s)/smurf of the assembly.
           assemble_xtype          Assemble the given model to a urdf.
           check_hyrodyn           Checks whether the model can be loaded in Hyrodyn.
           check_meshes            Checks whether all meshes are available.
           convert                 Converts the given input robot file to SDF/URDF/PDF/SMURF.
           generate_xtype          Creates the xtype database entry for the given model repository
           preprocess_cad_export   Preprocess CAD->URDF exports to use with pipeline.
           run_pipeline            Process simulation models automatically.
           smurfs_in_pybullet      Loads a "smurfs" file in pyBullet.
           test_model              Test the latest model using a CI-pipeline.

