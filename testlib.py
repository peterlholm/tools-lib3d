"Test the library works"
from pathlib import Path
import open3d as o3d
from lib3d.registration import draw_registration_result,get_transformations






fil1 = Path("testdata/testobject/ph_obj/test_object.stl")
fil1a = Path("testdata/testobject/ph_obj/distort/test_object_p1M.ply")
fil2 = Path("testdata/testobject/ph_obj/distort/dist_0.001.ply")
art1 = Path("testdata/arti/file01.ply")
art2 = Path("testdata/arti/file04.ply")

TRANS = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
#mesh1 = o3d.io.read_triangle_mesh(str(fil1))
pcl1 = o3d.io.read_point_cloud(str(art1))
pcl2 = o3d.io.read_point_cloud(str(art2))
t_mesh,transform = get_transformations(pcl1, pcl2)
print(transform)
draw_registration_result(pcl1, pcl2, transformation=TRANS)
