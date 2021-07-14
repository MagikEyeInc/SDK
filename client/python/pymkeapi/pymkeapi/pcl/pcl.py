# -*- coding: utf-8 -*-

"""
MkE API to PCL conversions
"""

#import pcl

__copyright__ = "Copyright (c) 2017-2010, Magik-Eye Inc."

def update_point_cloud(pc, frame):       
    pc.from_array(frame.pts3d.astype('float32'))

