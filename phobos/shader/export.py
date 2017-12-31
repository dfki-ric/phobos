# The functions in this module will have to be moved to the proper places in the phobos structure

import bpy


def count_links(sockets):
    """ Sums up all links attached to the list of given sockets

    :param sockets: The sockets to count the links for
    :return: The number of links
    """
    link_counter = 0
    for socket in sockets:
        link_counter = link_counter + 1 if socket.is_linked else link_counter
    return link_counter


def topological_sort(ntree):
    """ Performs a topological sorting after Kahn (https://en.wikipedia.org/wiki/Topological_sorting) on a NodeTree

    :param ntree: The NodeTree to perform the search on
    :return: A list of Nodes sorted in topological order. Empty if no such list exists
    """
    tree_copy = ntree.copy()  # For removing links from the tree without corrupting original Node Tree
    sorted_nodes = []
    no_incoming = set()
    if len(tree_copy.nodes) == 0:  # Cancel algorithm is node list is empty
        return sorted_nodes
    # Initialising no_incoming set with all nodes that have no incoming edges
    for node in tree_copy.nodes:
        if count_links(node.inputs) == 0:
            no_incoming.add(node)

    # Kahn's Algorithm
    while len(no_incoming) > 0:
        node = no_incoming.pop()
        sorted_nodes.append(node.get_clean_name())
        for output_socket in node.outputs:
            for link in output_socket.links:
                to_node = link.to_node  # Node we delete the link to
                tree_copy.links.remove(link)  # Remove link from node to any other node
                if count_links(to_node.inputs) == 0:
                    no_incoming.add(to_node)
    if len(tree_copy.links) > 0:  # Cyclic graph, so no topological sorting possible
        sorted_nodes = []
    bpy.data.node_groups.remove(tree_copy)  # Delete working copy of node tree
    return sorted_nodes


def export_shader(ntree):
    shader = dict(name="", type="", sort=[], nodes={})

    shader["name"] = ntree.name
    if ntree.bl_idname == "VertexShaderTree":
        shader["type"] = "vertex"
    elif ntree.bl_idname == "FragmentShaderTree":
        shader["type"] = "fragment"
    else:
        shader["type"] = "unknown"
    shader["sort"] = topological_sort(ntree)
    for node in ntree.nodes:
        shader["nodes"][node.get_clean_name()] = node.export()
    # TODO: Handle collection of all custom nodes for additional data export
    return shader
