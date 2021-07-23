import open3d as o3d
import numpy as np
import rospy
import std_msgs
from std_msgs.msg import Float64MultiArray
import roslib
import copy

voxel_size = 0.001 #default 0.02
already_cropped = True #If already cropped PCD availabe, set to True. Else False
generate_mesh = False #If STL model is STL file, set to True. If it has been converted to PCD before, set to False
location_not_cropped = "../140721.pcd" #Location of the PCD before it is cropped. Can be left empty if you have a cropped one
location_cropped = "../140721.ply" #Location of the already cropped PCD.
location_stl = "../Skeleton_Target.stl" #Location of the STL-Model
stl_in_pcd = "../Skeleton_Target.pcd" #If the STL-Model has been stored as PCD already, enter the location here

#Publishers to publish all the Points after Picking
pub_target = rospy.Publisher("/registration/target", Float64MultiArray, queue_size=10)
pub_support1 = rospy.Publisher("/registration/support1", Float64MultiArray, queue_size=10)
pub_support2 = rospy.Publisher("/registration/support2", Float64MultiArray, queue_size=10)
pub_support3 = rospy.Publisher("/registration/support3", Float64MultiArray, queue_size=10)

#Loading the PCD and calling the function to crop if necessary
def load_point_cloud():
    #When the PCD is raw from Azure Kinect, it needs to be cropped. If it has been cropped already, this step will be skipped.
    if already_cropped:
        target = o3d.io.read_point_cloud(location_cropped)
        o3d.visualization.draw_geometries([target])
    else:
        pcd = o3d.io.read_point_cloud(location_not_cropped)
    	target = crop_geometry(pcd)
    return target

#Function to manually crop the PCD taken by Azure Kinect Camera
def crop_geometry(pcd):
    print("Demo for manual geometry cropping")
    print("1) Press 'Y' twice to align geometry with negative direction of y-axis")
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    o3d.visualization.draw_geometries_with_editing(pcd)
    
    pcd_ret = o3d.io.read_point_cloud(location_cropped)
    o3d.visualization.draw_geometries([pcd_ret])
    return pcd_ret


#Prepares the PCD and STL, applies initial transformation
def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    target = load_point_cloud()

    #Check, whether the STL model should be loaded and transformed into PCD or it is already as PCD
    if generate_mesh:
    	source = mesh_generation()
    else:
        source = o3d.io.read_point_cloud(stl_in_pcd)
        o3d.visualization.draw_geometries([source])
        
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) #Initial transform to get the STL model better aligned
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

#Downsamples PCD, calculates FPFH features and estimates normals for Point to Plane PCD
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)

    #The FPFH feature is a 33-dimensional vector that describes the local geometric property of a point. 
    #A nearest neighbor query in the 33-dimensinal space can return points with similar local geometric 
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


#RANSAC based global registration. Can be used instead of Fast Global Registration
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 100 #voxel_size * 100 has been found experimentally
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(True), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 5000))
    return result

#Fast Global Registration Algorithm based on Q.-Y. Zhou, J. Park, and V. Koltun, Fast Global Registration, ECCV, 2016.
def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 100 #voxel_size * 100 has been found experimentally
    print(":: Apply fast global registration with distance threshold %.3f" \
          % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

#Function that displays the two PCDs with transformation already applied. This way, transformation is not directly applied to PCDs and registration can be rerun if bad
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #To see it easier, both PCDs can be painted in uniform color
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

#Created the PCD from STL model and transforms the dimenstion from mm to m
def mesh_generation():
    mesh = o3d.io.read_triangle_mesh(location_stl)
    point_cloud = mesh.sample_points_poisson_disk(80000)

    xzy = np.asarray(point_cloud.points)
    xzy = xzy/1000 #Transform from mm to m


    point_pcd = o3d.geometry.PointCloud()
    point_pcd.points = o3d.utility.Vector3dVector(xzy)
    point_pcd.estimate_normals()
    
    o3d.visualization.draw_geometries([point_pcd])
    o3d.io.write_point_cloud(stl_in_pcd, point_pcd)
    return point_pcd


#Point to Plane ICP based on Y.Chen and G. G. Medioni, Object modelling by registration of multiple range images, Image and Vision Computing, 10(3), 1992.
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 10
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_fast_global.transformation,
        o3d.registration.TransformationEstimationPointToPlane(), o3d.registration.ICPConvergenceCriteria(max_iteration=1000))
    #Outputs: Fitness: measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.
    #Outputs: RMSE: measures the RMSE of all inlier correspondences. The lower the better
    return result

#Function that shows the window to manually pick the points
def pick_points(pcd):
    print("")
    print("1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

#Transforms the PCD from Camera Coordinates to Robot Base Coordinates
def final_point_trans(coord):
    #Joint State Position where PCD was taken: [-0.10029397922745889, -1.2911458184120457, 0.7049271191044858, -3.05011957601859, 0.5050431246492588, 3.4907807440270098, 0.6923895804799265]
    #Forward Kinematics Matrix at position where PCD was taken (EE to Camera)
    fk_mat = np.asarray([[-0.104846281938938, 0.08594019278569172, 0.9907681567488626, 0.205107895661851],
    [-0.49049111132961404, -0.871125032192827, 0.023656880472417995, 0.1371478793849342],
    [0.8651160193119887, -0.48348263831393584, 0.1334871213966907, 0.5630145053833304],
    [0.0, 0.0, 0.0, 1.0]])

    #XMat from Hand-Eye Calibration (Base to EE)
    cam_end = np.asarray([[-2.33208078e-03 ,  3.26338090e-03 ,-4.03153535e-03, -3.70308947e-01],
    [ 2.21462804e-03, 4.17180971e-03, -3.70308947e-01, 3.07329256e-01],
    [-6.92904642e-04  , 3.91066199e-04, 3.07329256e-01, -1.17410629e-01],
    [0.0, 0.0, 0.0, 1.0]]) 

    final_mat = np.matmul(fk_mat,cam_end)
    final = np.matmul(final_mat, coord)
    return final

def publish_target_points(target, support1, support2, support3):
    target = target.flatten()
    support1 = support1.flatten()
    support2 = support2.flatten()
    support3 = support3.flatten()

    target_data = Float64MultiArray()
    support1_data = Float64MultiArray()
    support2_data = Float64MultiArray()
    support3_data = Float64MultiArray()

    target_data.data = target
    support1_data.data = support1
    support2_data.data = support2
    support3_data.data = support3

    pub_target.publish(target_data)
    pub_support1.publish(support1_data)
    pub_support2.publish(support2_data)
    pub_support3.publish(support3_data)

if __name__ == "__main__":

    rospy.init_node('registration', anonymous=True)

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug) #Show Debug Information

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

    #Do Global registration
    while True:
    	result_fast_global = execute_fast_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    	print(result_fast_global)
    	draw_registration_result(source_down, target_down,
                             	result_fast_global.transformation)
        good_results = raw_input("Where the results good? [Y|n]: ")
	#If user decides that registration result was not good, redo the Fast Global Registration
	if not good_results:
		good_results = "Y"
        if good_results == "n" or good_results == "N":
        	pass
        else:
		break
	

    source.transform(result_fast_global.transformation)

    #Refine registration by running ICP 
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     voxel_size)
    print(result_icp)
    draw_registration_result(source_down, target_down, result_icp.transformation)
    source_trans= source_down.transform(result_icp.transformation)
    
    #Combine source and target to get a single PointCloud with both
    pcd_combined = o3d.geometry.PointCloud()
    pcd_combined = target_down + source_trans

    points = pick_points(pcd_combined)

    o3d.io.write_point_cloud("/home/rnm/multiway_registration_140721.pcd", pcd_combined) #Store the combined PCD as Backup

    pcd_array = np.asarray(pcd_combined.points)

    #Make the points homogeneous
    target_point = np.append(pcd_array[points[0]],1)
    point1 = np.append(pcd_array[points[1]],1)
    point2 = np.append(pcd_array[points[2]],1)
    point3 = np.append(pcd_array[points[3]],1)


    #Transform the points to base coordinate system and print
    target_point_base = final_point_trans(target_point)
    point1_base = final_point_trans(point1)
    point2_base = final_point_trans(point2)
    point3_base = final_point_trans(point3)
    rospy.logwarn("TARGET: " + str(target_point_base))
    rospy.logwarn("SUPPORT POINT 1: " + str(point1_base))
    rospy.logwarn("SUPPORT POINT 2: " +  str(point2_base))
    rospy.logwarn("SUPPORT POINT 3: " + str(point3_base))

    #Publish all the points to rostopics
    publish_target_points(target_point_base, point1_base, point2_base, point3_base)



