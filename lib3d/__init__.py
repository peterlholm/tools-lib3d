#lib3d/__init__.py
"""
lib3d provide severel 3d orientated functions:
"""
#   240318  PLH     Updated version

import open3d as o3d
from .registration import get_transformations

def get_transformation(pcl_org: o3d.geometry.PointCloud,
                       pcl_test: o3d.geometry.PointCloud) -> tuple:
    """
    Get the tranformation matrix for converting pcl_test to the pcl_org

    Args:
        pcl_org(PointCloud): Original reference pointcloud.
        pcl_test(PointCloud): The test pointcloud to be matched against the reference pointcloud
    
    Returns:
        (test_pcl, TransformationMatrix)

    """

    test_pcl, test_transformation = get_transformations(pcl_org, pcl_test)
    return test_pcl, test_transformation
