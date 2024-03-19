"lib3d declarations"
#   240318  PLH     Updated version

import open3d as o3d
from .registration import get_transformations

def get_transformation(pcl_org: o3d.geometry.PointCloud,
                       pcl_test: o3d.geometry.PointCloud) -> tuple:
    "get the transformation to get test to match org"
    test_pcl, test_transformation = get_transformations(pcl_org, pcl_test)
    return test_pcl, test_transformation
