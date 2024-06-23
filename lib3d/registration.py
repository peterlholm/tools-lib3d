'pointcloud registrations'

#   240617  PLH     Change to work in mm
#   240620  PLH     New default param

from time import perf_counter
import copy
import open3d as o3d
#import numpy as np

# pyrightx: reportMissingTypeStubs=true

_DEBUG = False
_SHOW = False
_TIMING = False

# original
GLOBAL_FITNESS = 0.5    # percent to overlap
GLOBAL_RMSE = 0.001
GLOBAL_FITNESS = 0.2   # percent to overlap
GLOBAL_RMSE = 0.001

MIN_FITNESS = 0.2
#original
LOCAL_FITNESS = 0.2
LOCAL_RMSE = 0.001

UNIT_MM = True

if UNIT_MM:
    VOXEL_SIZE = 0.2
    GLOBAL_RMSE = 0.3      # mm
    LOCAL_RMSE = 0.1       # mm
else:
    VOXEL_SIZE = 0.0005
    GLOBAL_RMSE = 0.001     # m
    LOCAL_RMSE = 0.001      # m

def draw_registration_result(reference: o3d.geometry.PointCloud, # pylint: disable=too-many-arguments
                             test_source: o3d.geometry.PointCloud,
                             transformation=None, axis=False, window_name="registration result", color=True):
    "Debug draw registration result"
    #print(type(reference), type(test_source))
    reference_temp = copy.deepcopy(reference)
    test_temp = copy.deepcopy(test_source)
    if color:
        reference_temp.paint_uniform_color([0.0, 0.9, 0.1])   # green
        test_temp.paint_uniform_color([0.9, 0.1, 0.1])        # red

    if not transformation is None:
        #test_temp.transform(transformation)
        reference_temp.transform(transformation)

    pointclouds =[ reference_temp, test_temp]

    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])
        pointclouds.append(axis_pcd)
    o3d.visualization.draw_geometries(pointclouds, window_name=window_name, width=500, height=500, point_show_normal=False) #mesh_show_back_face=False


def evaluate_registration(source_pcl, target_pcl, transformation):
    "Calculate the errors in the given transformation of source to target"
    max_distance = 0.1
    if _DEBUG:
        print(f"Source {len(source_pcl.points)} Points Target: {len(target_pcl.points)}")
    evaluation = o3d.pipelines.registration.evaluate_registration(source_pcl, target_pcl, max_distance, transformation)
    #print(evaluation)
    return evaluation

# def compute_normal(pcd):
#     "creates normalts that all point in same (wrong) direction (due to low radius)"
#     pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
#               radius=0.1, max_nn=30))
#     normals_load = np.asarray(pcd.normals) * -1  # Flip normals.
#     pcd.normals = o3d.utility.Vector3dVector(normals_load)
#     # Get new and correctly orientated normals.
#     pcd.estimate_normals()

def preprocess_point_cloud(pcd, voxel_size):
    "prepare point cloud by down sample and compute features"
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    #creates normalts that all point in same (wrong) direction (due to low radius)"
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    if _DEBUG:
        print(f"Number Points {len(pcd_down.points)}, Number features: {pcd_fpfh.num()}")
    return pcd_down, pcd_fpfh

def execute_global_registration(reference_down, target_down,        # pylint: disable=too-many-arguments
                                reference_fpfh, target_fpfh,
                                voxel_size, dist_thres_scalar=1.5,
                                scale=False, edge_length_thres=0.9, # org 0.99
                                converge_itr=(100000),       # org 10**8  demo 100000
                                converge_certainty=0.999): # org 0.9999 demo 0.999
    "find global registation"
    #     voxel_size, dist_thres_scalar=1.5,
    # scale=False, edge_length_thres=0.99,
    # converge_itr=(10**8),
    # converge_certainty=0.9999):

    if _DEBUG:
        print("Global registration start")
    if _TIMING:
        st = perf_counter()
    distance_threshold = voxel_size * dist_thres_scalar
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        reference_down, target_down, reference_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(scale),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                edge_length_thres),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(
            converge_itr, converge_certainty))
    if _TIMING:
        print(f"Global registation time: {perf_counter()-st} sec")
    # result
    return result

def execute_local_registration(reference_down, target_down, voxel_size,
                               init_transformation, converge_max_itr=30):
    "Find local registration"
    conver_crit = o3d.pipelines.registration.ICPConvergenceCriteria()
    conver_crit.max_iteration = converge_max_itr
    result_icp = o3d.pipelines.registration.registration_icp(
                    reference_down, target_down,  voxel_size, init_transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=conver_crit)
    return result_icp

# def prepare_dataset(ref, test_target, voxel_size):
#     "prepare data set "
#     # if _DEBUG:
#     #     print("Preprocessing reference")
#     ref_down, ref_fpfh = preprocess_point_cloud(ref, voxel_size)
#     # if _DEBUG:
#     #     print("Preprocession test target")
#     test_target_down, test_target_fpfh = preprocess_point_cloud(test_target, voxel_size)
#     return ref_down, test_target_down, ref_fpfh, test_target_fpfh

def get_transformations(ref: o3d.geometry.PointCloud|o3d.geometry.TriangleMesh,  # pylint: disable=too-many-locals
                        test_target: o3d.geometry.PointCloud|o3d.geometry.TriangleMesh,
                        voxel_size: float=VOXEL_SIZE, min_fitness=MIN_FITNESS, verbose=False) -> tuple:
    "get transformations from pointclouds"
    start_time = perf_counter()
    if _SHOW:
        draw_registration_result(ref, test_target, window_name="original pointclouds")
    ref_down, ref_fpfh = preprocess_point_cloud(ref, voxel_size)
    test_down, test_fpfh = preprocess_point_cloud(test_target, voxel_size)
    #ref_down, test_down, ref_fpfh, test_fpfh = prepare_dataset(ref, test_target, voxel_size)
    prepare_time = perf_counter()
    # if _DEBUG:
    #     #print("get_transformations, voxel size", voxel_size)
    #     o3d.io.write_point_cloud("/tmp/ref_vox.ply", ref_down)
    #     o3d.io.write_point_cloud("/tmp/test_vox.ply", test_down)
    if _SHOW:
        o3d.visualization.draw_geometries([ref_down, test_down], window_name="downsample", height=500, width=500)
        # GLOBAL registration
    result_ransac = execute_global_registration(ref_down, test_down, ref_fpfh, test_fpfh, voxel_size,
                                                converge_itr=(100000), converge_certainty=0.999, dist_thres_scalar=1.5)
    global_time = perf_counter()
    if _DEBUG or verbose:
        print(f"Global Registration result: Fittnes {result_ransac.fitness:.2} Rms {result_ransac.inlier_rmse:.3}")
        #print("Global transformation matrix\n", result_ransac.transformation)
        if _SHOW:
            draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")

    if result_ransac.fitness < min_fitness or result_ransac.inlier_rmse > GLOBAL_RMSE:
        print(f"BAD GLOBAL REGISTRATION Fittnes {result_ransac.fitness:.2} < {min_fitness} Rms {result_ransac.inlier_rmse:.3} > {GLOBAL_RMSE}")
        #print(result_ransac.transformation)
        if _SHOW:
            #print("global transformation matrix", result_ransac, result_ransac.transformation)
            draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="BAD Global registration")
        return None, None
    if _DEBUG:
        print("Local registration")
    local_start = perf_counter()
    result_icp = execute_local_registration(ref_down, test_down, voxel_size, result_ransac.transformation)
    local_end = perf_counter()
    if _TIMING:
        print("Timing (sec)")
        print("Prepare", prepare_time-start_time, "Global", global_time-prepare_time, "Local", local_end-local_start)
    if _DEBUG:
        print(f"Localregistration: Fitness {result_icp.fitness:.2} RMSE: {result_icp.inlier_rmse:.3}")
        #print("Local transformation matrix\n", result_icp.transformation)
        evalu = evaluate_registration(ref_down, test_down, result_icp.transformation)
        print("Local eval", evalu)
    if result_icp.fitness < min_fitness or result_icp.inlier_rmse >LOCAL_RMSE:
        print(f"BAD LOCAL REGISTRATION Fittnes {result_icp.fitness} < {min_fitness} Rms {result_icp.inlier_rmse} > {LOCAL_RMSE}")
        if _SHOW:
            print("Local transformation matrix", result_icp, result_icp.transformation)
            draw_registration_result(ref_down, test_down, result_icp.transformation, window_name="Local registration")
            print("global transformation matrix", result_ransac, result_ransac.transformation)
            draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")
        return None, None

    return test_target, result_icp #, inf_matrix


if __name__ == "__main__":
    from pathlib import Path
    fil1 = Path("testdata/testobject/ph_obj/test_object.stl")
    fil1a = Path("testdata/testobject/ph_obj/distort/test_object_p1M.ply")
    fil2 = Path("testdata/testobject/ph_obj/distort/dist_0.001.ply")
    art1 = Path("testdata/arti/file01.ply")
    art2 = Path("testdata/arti/file04.ply")

    TRANS = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    #mesh1 = o3d.io.read_triangle_mesh(str(fil1))
    pcl1 = o3d.io.read_point_cloud(str(art1))
    pcl2 = o3d.io.read_point_cloud(str(art2))
    #t_mesh,transform = get_transformations(pcl1, pcl2)
    #print(transform)
    draw_registration_result(pcl1, pcl2, transformation=TRANS)
