.. Phobos documentation master file, created by
   sphinx-quickstart on Wed Feb 28 08:59:46 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Phobos API documentation
========================

Introduction
------------

Phobos is a tool to create, edit, maintain and evaluate 3D-models for robots, objects and environments.

It comes with several modules:

- An add-on for the open-source 3D modeling software Blender that enables the creation and modification of WYSIWYG robot models for use in robot frameworks like ROS and ROCK or in real-time simulations such as MARS or Gazebo. Phobos allows to export completed models in formats such as URDF, SMURF or SDF and meshes as STereoLithography (.stl), Wavefront (.obj) or Collada (.dae).
- A python API that can be used in scripts
- Pre-defined utilty scripts that can be used directly from command line for common operations
- A tool to run phobos in a CI-pipeline (Continuous Integration) to maintain your models and keep several versions of one robot model up-to-date and consistent with each other

Phobos was initiated and is currently developed at the Robotics Innovation Center of the German Research Center for Artificial Intelligence (DFKI) in Bremen, together with the Robotics Research Group of the University of Bremen.

User documentation can be found in the Phobos Wiki at https://github.com/dfki-ric/phobos/wiki .

Please contact `Henning Wiedemann <https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/hewi04.html>`_ for any inquiries, or any questions and feedback not suited for the `issues <https://github.com/dfki-ric/phobos/issues>`_ page.

This page provides the code documentation of Phobos.

Release "Perilled Pangolin"
------------------------------
Phobos version 2.0 is named after the tree pangolin (Phataginus tricuspis) to draw attention to one more endangered species.

.. image:: img/pangolin.jpeg
  :width: 240
  :alt: Image of a Saiga (source wikimedia.org)
(source wikimedia.org)

Module overview
---------------

Phobos is structured into some submodules which contain different functionality:

.. toctree::
   :maxdepth: 1
   :caption: Blender Add-on

   phobos.blender <phobos.blender.rst>
   phobos.blender.io <phobos.blender.io.rst>
   phobos.blender.model <phobos.blender.model.rst>
   phobos.blender.operators <phobos.blender.operators.rst>
   phobos.blender.utils <phobos.blender.utils.rst>

.. toctree::
   :maxdepth: 1
   :caption: API

   phobos.io <phobos.io.rst>
   phobos.core <phobos.core.rst>
   phobos.scenes <phobos.scenes.rst>

.. toctree::
   :maxdepth: 1
   :caption: Utils

   phobos.geometry <phobos.geometry.rst>
   phobos.utils <phobos.utils.rst>

.. toctree::
   :maxdepth: 1
   :caption: CI

   phobos.ci <phobos.ci.rst>

.. toctree::
   :maxdepth: 1
   :caption: Scripts

   phobos.scripts <phobos.scripts.rst>

Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

