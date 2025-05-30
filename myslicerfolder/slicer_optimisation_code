

#the four functions below form the optimisation  part of this slicer code. this is also the 3rd vector filtration step. 

#we use this to create a cKDTree of venctricles node and vessels node, for all voxels in the volume
def voxel_positions(labelmapnode):

    dims = labelmapnode.GetImageData().GetDimensions()
    voxel_positions = []

    #in these loops, for every i value, every j value corresponsing to that i value, and every k value corresponding to thay j value is looped
    #over, so every voxel is considered in our code. GetScalarComponentAsDouble gives us the scalar intensity value for the voxel at that IJK 
    #position. If it is above 0, that means it is in our labelmap region of interest, and it gets added to the empty voxel_positions list. 
    for i in range(dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                if labelmapnode.GetImageData().GetScalarComponentAsDouble(i, j, k, 0) > 0:
                    pointinIJK = (i, j, k)
                    voxel_positions.append(pointinIJK)

    return np.array(voxel_positions)

#create a cKDTree, but in IJK
ventriclestree = cKDTree(voxel_positions(ventriclesnode))
vesselstree = cKDTree(voxel_positions(vesselsnode))



#similar core logic to is_trajectory_valid_intensity 
def get_closest_distance(labelmapnodetree, labelmapnode, entry, target):
    stepsize = 0.5
    vector = np.array(target) - np.array(entry)
    steps = int(np.linalg.norm(vector) / stepsize)
    minimum_distance = float('inf')  

    for i in range(steps + 1):
        point = np.array(entry) + i * stepsize * (vector / np.linalg.norm(vector))

        pointinIJK = convert_ras_to_ijk(labelmapnode, point)
        
        #query[0] from the tree table holds minimum distances. so we are querying, for this intermediate IJK point, what is the closest point on the labelmap?
        distance = labelmapnodetree.query(pointinIJK)[0]  
        
        #we will do this for all intermediate points on a single vector, and overall, we want to get the minimum distance of a vector from the given labelmap, at any point along its trajectory
        minimum_distance = min(minimum_distance, distance)

    return minimum_distance


#objective_function and run_optimisation are closely linked. in itself, objective function is not called. It is more akin to a for loop where params[0] acts as an index for all the vectors, as specified in the 
#run_optimisation bounds. objective function is called on each vector, it then calls get_closest_distance for both vessels and venctricles, and returns the combined weighted distance. Of course in our task we give 
#both labelmaps equal importance, but this allows us to consider their distance avoidance as a whole.
def objective_function(params):
    vector_index = int(params[0])
    entry, target = valid_vectors[vector_index]
    
    distance_vessels = get_closest_distance(vesselstree, vesselsnode, entry, target)
    distance_ventricles = get_closest_distance(ventriclestree, ventriclesnode, entry, target)
    

    weight_vessels = 0.5 
    weight_ventricles = 0.5
    combined_distance = weight_vessels * distance_vessels + weight_ventricles * distance_ventricles
    
    #importantly, we are returning -combined distance. Negative distance is chosen as the optimisation function itself performs a minimisation – minimising the smallest distance is counter intuitive to our task. 
    return -combined_distance  

def run_optimization():
    num_vectors = len(valid_vectors)
    bounds = [(0, num_vectors - 1)]
    result = differential_evolution(objective_function, bounds, maxiter= 30, popsize = 200)

    best_entry, best_target = valid_vectors[int(result.x[0])]
    print("population based best trajectory is - ", best_entry, "to", best_target)

    return best_entry, best_target


best_entry, best_target = run_optimization()
