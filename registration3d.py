#!/bin/python
"Test registration part"
from time import perf_counter
from pathlib import Path
import argparse
import sys
import open3d as o3d

try:
    from lib3d.registration import get_transformations, draw_registration_result #, evaluate_registration
except ModuleNotFoundError:
    sys.path.append("/usr/local/lib")
    from lib3d.registration import get_transformations, draw_registration_result #, evaluate_registration

#import numpy as np

def surface_to_pcl(mesh, alg="poisson", number_of_points=100000, init_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=number_of_points, )
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=number_of_points, init_factor=init_factor)
    return pcl

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='registration3d', description='Registration of test object, giving translation and errors')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', required=False, help="Show pointclouds", action='store_true' )
    #parser.add_argument('-o', '--output', required=False, type=Path, help="Output file", metavar="Outputfile")
    parser.add_argument('reference_file', type=Path, help="The original mesh or pointcloud")
    parser.add_argument('test_file', type=Path, help="The pointcloud to be stitched")

    args = parser.parse_args()

    _DEBUG = args.d
    _VERBOSE = args.v
    _SHOW =args.s

    TRANS_MATRIX = [[ 1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],[0, 0, 0, 1]]

    if not args.reference_file.exists() and not args.reference_file.suffix in (".ply", ".stl"):
        print("input file(s) does not exist or is invalid type")
        sys.exit(1)
    # check file types
    if args.reference_file.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(args.reference_file))
        ref_pcl = surface_to_pcl(inmesh)
    elif args.reference_file.suffix=='.ply':
        ref_pcl = o3d.io.read_point_cloud(str(args.reference_file))
        if not ref_pcl.has_colors():
            if _DEBUG:
                print("Apply white color to reference pcl")
            ref_pcl.paint_uniform_color((1,1,1))
    else:
        print("Input file type error")
        sys.exit(1)
    # check 2. file or folder
    if not args.test_file.exists():
        print("input file/folder does not exist")
        sys.exit(1)
    if not args.test_file.suffix in ['.ply']:
        print("Input file type error")
        sys.exit(1)

    # start ---------------------------------

    # register input in_pcl
    t_pcl = o3d.io.read_point_cloud(str(args.test_file))
    if _DEBUG:
        print("pointcloud size", t_pcl.get_axis_aligned_bounding_box())
    vol = t_pcl.get_axis_aligned_bounding_box().volume()
    vox = vol ** (1./ 3.) / 100
    if _DEBUG:
        print("Voxel estimate",vox)
    if _VERBOSE:
        print(f"Number points:  {len(ref_pcl.points)},  {len(t_pcl.points)}")
    start_time = perf_counter()
    target, transformation = get_transformations(ref_pcl, t_pcl, voxel_size=0.1)
    #print(transformation)
    stop_time = perf_counter()
    if _VERBOSE and not _DEBUG:
        print(f"Registration time: {stop_time-start_time:.2f} sec" )
    if transformation is None:
        print(f"-------- Registration of {str(args.test_file)} unsuccessfull ---------------")
        sys.exit(2)
    else:
        print("Final Transformation\n", transformation.transformation)
        print(f"Fitness: {transformation.fitness*100:.1f}% RMSE: {transformation.inlier_rmse:.3f}")
    #   EVAL = evaluate_registration(ref_pcl, t_pcl, transformation)
    #   print(EVAL)
    #   print(f"Fittnes: {EVAL.fitness*100:.2f}% RMSE: {EVAL.inlier_rmse:.3f} CorrespondenceSet: {len(EVAL.correspondence_set)}")

    if _SHOW:
        draw_registration_result(ref_pcl, t_pcl, transformation=transformation.transformation, axis=True)

    # if args.output:
    #     print(args.output.parent.absolute())
    #     if not args.output.parent.exists():
    #         print("Outputfolder does not exist")
    #         sys.exit(2)
    #     new_pcl = t_pcl.transform(transformation)
    #     rms, mmin, mmax, mean = cmp2pcl(in_pcl, new_pcl)
    #     print(f"RMS error: {rms*1000:.3f} mm")
    #     col_pcl = concatenate_pcl(in_pcl, new_pcl)
    #     print(f"New pointcloud with {len(col_pcl.points)} points")
    #     filename = Path(f).with_suffix('.new.ply')
    #     in_pcl = col_pcl
    #     o3d.io.write_point_cloud(str(filename), col_pcl)
