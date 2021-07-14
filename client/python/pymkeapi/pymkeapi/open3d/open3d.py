# -*- coding: utf-8 -*-

"""
MkE API to Open3D conversions
"""

import open3d

__copyright__ = "Copyright (c) 2017-2010, Magik-Eye Inc."

def update_point_cloud(pc, frame): 
    pc.points = open3d.utility.Vector3dVector(frame.pts3d)

