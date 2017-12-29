#!/usr/bin/python
# coding=utf-8

__author__ = 'oschwiegert'

import phobos


def register():
    print("Registering shader module")
    phobos.shader.trees.register()
    phobos.shader.sockets.register()
    phobos.shader.nodes.register()
    phobos.shader.gui.register()


def unregister():
    print("Unregistering shader module")
    phobos.shader.trees.unregister()
    phobos.shader.sockets.unregister()
    phobos.shader.nodes.unregister()
    phobos.shader.gui.unregister()
