# Contributing to Phobos 
Thanks for taking the time to consider contributing to Phobos! We know time is a scarce resource and we greatly appreciate your support.

Most likely you've found your way to this page because you're a robotics specialist, and you're missing a feature or found a problem in Phobos corresponding to your field of expertise. That's great, because we've been developing Phobos for years with our own requirements in mind, and you're surely thinking of something we haven't considered, yet. That makes your input very valuable.

Even before you start implementing new features on your fork of Phobos and send us [pull requests](https://github.com/dfki-ric/phobos/pulls), please feel free to create [issues](https://github.com/dfki-ric/phobos/issues) to report bugs or propose new features, or contribute to the ongoing discussions.

You may also consider to post a question or point of discussion on [StackExchange Robotics](https://robotics.stackexchange.com/) or [ROS Discourse](https://discourse.ros.org/).


## Documentation

We have two separate documentation pages. One is the user documentation, residing in the [Phobos wiki](https://github.com/dfki-ric/phobos/wiki). The code documentation, compiled using Sphinx, can be found at our [GitHub.io pages](http://dfki-ric.github.io/phobos).


## How to get started

As always on GitHub, go ahead and create your own fork of Phobos ([GitHub Help: Fork a repo]([https://help.github.com/articles/fork-a-repo/)) to create your own code playground. It facilitates sending us [pull requests](https://github.com/dfki-ric/phobos/pulls) later.

Phobos is based on Blender, so if you haven't familiarized yourself with [Blender Basics](https://github.com/dfki-ric/phobos/wiki/Blender%20Basics) yet, it's a good idea to start here and get to know the environment we work with. We rely on its functionality a lot in our code. For instance, some operations such as importing or creating new meshes use the same operators that a user can access from Blender's GUI. You can play around with Phobos a bit and check out the code of the operators you use to get an idea of how we handle the related data in Blender.

It also probably helps to have a basic understanding of [URDF](http://wiki.ros.org/urdf), as it largely served as a template for how we organize robot models in Phobos.


## Issues
Congratulations! You found a bug which slipped through our fingers or you came
up with a useful feature we haven't thought of, yet. Check out the existing
[issues](https://github.com/dfki-ric/phobos/issues) to see if the subject has
already been covered. If that is the case, consider joining the discussion!

Otherwise just add a new issue with the green button in the top right of the
screen. Make use of the provided template as far as it is helpful for you. The
more information you provide, the more likely it is that we can address it
quickly.


## Submitting changes

Go ahead and submit your contribution as a [pull request](https://help.github.com/articles/about-pull-requests/). We will gladly check it out and provide you with feedback. We may ask you to make a few changes, for instance, to [use `git rebase`](https://code.tutsplus.com/tutorials/rewriting-history-with-git-rebase--cms-23191) to clean and squash your commits and provide a linear history.


## Code Organization

Before you start coding, take a brief look at Phobos' repository and try to find your bearings, so you can add the code where it fits best. The core functionality of Phobos which makes up the Blender plugin proper is found in the `phobos` sub-folder. In there, we have a bunch of directories, which organize our code into the following parts:

Folder | Contents
------ | ----------
. | basic functionality such as logging and GUI
definitions | YAML files to be parsed upon startup by the `defs` module
io | modules dealing with various input- and output formats
model | functions to manipulate model representation in Blender and  as a dictionary
operators | GUI-accessible Blender operators
utils | utility functions in various categories

Before you start adding code, it makes sense to become familiar with similar functionality in Phobos first, as *utils* contains a lot of convenience functions that are used throughout the rest of the code. It is very likely that part of what you want to achieve is already provided there. Especially *operators* should ideally not contain too much functionality themselves, but rather use utility functions. In other words: Whenever something is done in an *operator* that could come in handy again somewhere else, put it in a function in *utils* and call it from the operator. We've been trying to make our code [drier](https://en.wikipedia.org/wiki/Don%27t_repeat_yourself) over time, and you could help us.

Also, if you simply want to provide a useful snippet of code to other users, you can always add it as a code template in the *templates_py* folder of the repository.


## Code format and style

We mostly adhere to [PEP8](http://pep8.org/), with the exceptions that we allow lines to be 100 characters long (those screens really have become wider, folks) and that we use camel case for functions for historical reasons (we probably wouldn't if we started out today). There are other exceptions when legibility trumps convention - you'll get the hang of it when you browse our code, but none of these are mission-critical.

We use [Google-style docstrings](http://www.sphinx-doc.org/en/master/ext/example_google.html). We like to keep them brief, as they otherwise tend to clutter the code. If you find yourself writing a really long docstring: a) Great for taking the time to write proper documentation! b) You sure this function shouldn't be [split into two or more](https://iwanttocode.wordpress.com/tag/god-function/)?

We use the [Black Code Formatter](https://github.com/ambv/black) to clean up our code. To have your code properly formatted and also your Docstrings updated and fixed:

    $ make format
    
before commiting.


## Git conventions

We'd really appreciate if you adhere to the [standard convention for Git commit messages](https://chris.beams.io/posts/git-commit/). It's like chopping veggies in the kitchen: You're gonna be doing this your entire life, might as well do it right. That being said, for small changes, we're completely fine with one-liners.

Also, have a look at this [blogpost](https://www.spreedly.com/2014/06/24/merge-pull-request-considered-harmful/) how we handle pull-requests. It might help you to understand our workflow...

The Phobos repository is based on the git structure you can find in the [wiki](https://github.com/dfki-ric/phobos/wiki/Installation#versions-and-branching). We decided to drop the GitFlow method as it was too cumbersome for us, and instead introduced our own Git method which relies on rebasing of code. Every maintainer should use his/her own fork to keep the feature branches, which allows for easier rebasing before a merge into the master branch.

---

Thanks for considering this lengthy advice,

*Kai von Szadkowski, Simon V. Reichel, your trustworthy maintainers*
