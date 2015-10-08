#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib.packages
import catkin.find_in_workspaces

from catkin.workspace import get_source_paths, get_workspaces
from catkin_pkg.packages import find_packages


def _get_source_path_to_packages(workspaces=None,
                                 workspace_to_source_spaces=None):
    """Return dictionay with mappings from source paths to packages.

    :param workspaces: (optional, used for unit tests),
        the list of workspaces to use.
    :param workspace_to_source_spaces: the dictionary is populated
        with mappings from workspaces to source paths,
        pass in the same dictionary to avoid repeated reading
        of the catkin marker file.
    """
    if workspaces is None:
        workspaces = get_workspaces()
    if workspace_to_source_spaces is None:
        workspace_to_source_spaces = {}

    source_path_to_packages = {}
    for ws in workspaces:
        if ws not in workspace_to_source_spaces:
            workspace_to_source_spaces[ws] = get_source_paths(ws)
        for src_path in workspace_to_source_spaces[ws]:
            if src_path not in source_path_to_packages:
                source_path_to_packages[src_path] = find_packages(src_path)
    return source_path_to_packages


# dictionary with mappings from source paths to packages
_source_path_to_packages = None


def _use_cached_resource(func):
    def __use_cached_resource(*args, **kwargs):
        global _source_path_to_packages
        if _source_path_to_packages is None:
            _source_path_to_packages = _get_source_path_to_packages()
        return func(*args, **kwargs)
    return __use_cached_resource


@_use_cached_resource
def find_node(*args, **kwargs):
    kwargs['source_path_to_packages'] = _source_path_to_packages
    return roslib.packages.find_node(*args, **kwargs)


@_use_cached_resource
def find_resource(*args, **kwargs):
    kwargs['source_path_to_packages'] = _source_path_to_packages
    return roslib.packages.find_resource(*args, **kwargs)


@_use_cached_resource
def find_in_workspaces(*args, **kwargs):
    kwargs['source_path_to_packages'] = _source_path_to_packages
    return catkin.find_in_workspaces.find_in_workspaces(*args, **kwargs)
