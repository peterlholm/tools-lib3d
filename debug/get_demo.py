import open3d as o3d

demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
o3d.io.write_point_cloud("testdata/demo/cloud1.ply", source)
o3d.io.write_point_cloud("testdata/demo/cloud2.ply", target)

