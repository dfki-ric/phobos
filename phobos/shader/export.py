# The functions in this module will have to be moved to the proper places in the phobos structure

import bpy
import phobos.shader.nodes as nodes


def count_links(sockets):
    """ Sums up all links attached to the list of given sockets

    :param sockets: The sockets to count the links for
    :return: The number of links
    """
    link_counter = 0
    for socket in sockets:
        link_counter = link_counter + 1 if socket.is_linked else link_counter
    return link_counter


def validate_shader(ntree):
    """Iterates over all linked input sockets and checks if the incoming data matches the required data type.
    If not, an error dict is appended to the returned list, containing information about the connected nodes,
    sockets and data types

    :param ntree: The tree to check
    :return: List containing error dicts. Empty if no errors were found.
    """
    errors = []
    for node in ntree.nodes:
        for input_socket in node.inputs:
            if not input_socket.is_linked:
                break
            origin_type = input_socket.links[0].from_socket.bl_idname
            if not origin_type == input_socket.bl_idname:
                errors.append(dict(node=node.name, socket=input_socket.name,
                                   from_socket_type=input_socket.links[0].from_socket.bl_label,
                                   socket_type=input_socket.bl_label,
                                   from_socket=input_socket.links[0].from_socket.name,
                                   from_node=input_socket.links[0].from_node.name))
    return errors


def solve_errors(ntree, errors):
    """This function alters the given node tree by trying to add converting nodes into the graph to erase type
    incompatibilities. The returned list contains the errors that could not be solved.

    :param ntree: The node tree to alter. !The node tree will be altered!
    :param errors: The errors to solve
    :return: The remaining errors not solved
    """
    remains = []
    for error in errors:
        func_name = error["from_socket_type"] + "_to_" + error["socket_type"]
        if (func_name.lower(), func_name.lower(), func_name.lower()) in nodes.CustomNode.node_types:
            faulty_link = ntree.nodes[error["node"]].inputs[error["socket"]].links[0]
            from_socket = faulty_link.from_socket
            to_socket = faulty_link.to_socket
            # ntree.links.remove(faulty_link)  # Done automatically by creating new links
            converter_node = ntree.nodes.new("CustomNode")
            converter_node.node_type = func_name.lower()
            ntree.links.new(converter_node.inputs[0], from_socket)
            ntree.links.new(to_socket, converter_node.outputs[0])
        else:
            remains.append(error)
    return remains


def topological_sort(ntree):
    """ Performs a topological sorting after Kahn (https://en.wikipedia.org/wiki/Topological_sorting) on a NodeTree

    :param ntree: The NodeTree to perform the search on
    :return: A list of Nodes sorted in topological order. Empty if no such list exists
    """
    tree_copy = ntree.copy()  # For removing links from the tree without corrupting original Node Tree
    sorted_nodes = []
    no_incoming = set()
    if len(tree_copy.nodes) == 0:  # Cancel algorithm if node list is empty
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
    shader = dict(name="", type="", sort=[], nodes={}, custom=set())

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
        if node.bl_idname == "CustomNode" or node.bl_idname == "CustomNodeFragment" or node.bl_idname == "CustomNodeVertex":
            shader["custom"].add(node.node_type)
    shader["custom"] = list(shader["custom"])
    return shader
