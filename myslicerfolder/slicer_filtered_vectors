entrypoints = []
for i in range(entrynode.GetNumberOfControlPoints()):
    pos = [0, 0, 0] 
    entrynode.GetNthControlPointPosition(i, pos)
    entrypoints.append(pos)
#simply creating a list of entry points in RAS in entrypoints. This list is used below to create entry-target pairs 
vectors = [(entry, target) for entry in entrypoints for target in targets_in_hippocampus]


#first vector filtration step
def is_trajectory_valid_intensity(entry, target, stepsize=0.5, int_threshold=0):

    #create vector by subtracting the two coordinates. Then, based on how long each step should be, we can calculate the number of intermediate points that will be generated. 
    vector = np.array(target) - np.array(entry)
    steps = int(np.linalg.norm(vector) / stepsize)

   
    for i in range(steps + 1):
        #for a vector, i = which step is it? so if i = 10th intermediate point, we have moved 10*0.5 = 5 units from the entry. align this to the vector by multiplying with the normalised vector.
        intermediatepoint = np.array(entry) + i * stepsize * (vector / np.linalg.norm(vector))
        
        #convert point in RAS to IJK
        pointinIJK = convert_ras_to_ijk(ventriclesnode, intermediatepoint)
        
        #get voxel intensity of that intermediate point for both labelmaps 
        intensity_ventricles = ventriclesnode.GetImageData().GetScalarComponentAsDouble(pointinIJK[0], pointinIJK[1], pointinIJK[2], 0)
        intensity_vessels = vesselsnode.GetImageData().GetScalarComponentAsDouble(pointinIJK[0], pointinIJK[1], pointinIJK[2], 0)

        #if any intensity is non-zero, remove the vector 
        if intensity_ventricles > int_threshold or intensity_vessels > int_threshold:
            return False  

    return True  


filtered_vectors = [(e, t) for e, t in vectors if is_trajectory_valid_intensity(e, t)]

print("number of vectors-", len(filtered_vectors))
for entry, target in filtered_vectors:
    print("valid vector- entry point-", entry, "target-", target)


#the code block below is for the angle filtration condition. this is the second vector filtration step
#create a surface mesh from the modelnode created manually. 
polydata = modelnode.GetMesh()


pointnumber = polydata.GetNumberOfPoints()
points = np.array([polydata.GetPoint(i) for i in range(pointnumber)])
#get surface normals for all points on the mesh 
normals = np.array([polydata.GetPointData().GetNormals().GetTuple(i) for i in range(pointnumber)])

#create a cKDTree of the points. this is a condensed form of the KDTree, a method that divides a space into k-dimensions for quicker distance lookup
kd_tree = cKDTree(points)


#given a vector, we need to find the first point that it intersects on the surface mesh to know the surface normal
def find_first_intersection(entry, target, cortexnode, stepsize=0.5):

    vector = np.array(target) - np.array(entry)
    steps = int(np.linalg.norm(vector) / stepsize)
    
    #combines logic from is_point_in_labelmap and is_trajectory_valid_intensity
    for i in range(steps + 1):
        point = np.array(entry) + i * stepsize * (vector / np.linalg.norm(vector))
        pointinIJK = convert_ras_to_ijk(cortexnode, point)
        
        if cortexnode.GetImageData().GetScalarComponentAsDouble(pointinIJK[0], pointinIJK[1], pointinIJK[2], 0) > 0:
            #this is important - remember, in this function, the intersection is calculated with the cortex labelmap, not the modelnode. so it is possible (due to the stepsize) that the point 
            #in question is not on the surface, but beneath it. due to this, we need to use the KDTree we created and find the index of the surface mesh point that is closest to the point in
            #cortex node, and then get its surface normal
            closestindex = kd_tree.query(point)[1]
            surfacenormal = normals[closestindex]
            return point, surfacenormal
    
    return None, None  

valid_vectors = []
count_below_35 = 0

for entry, target in filtered_vectors:
    intersection, surfacenormal = find_first_intersection(entry, target, cortexnode)
    if intersection is None:
        continue  
    
    vector = np.array(target) - np.array(entry)
    vector_normalized = vector / np.linalg.norm(vector)
    normal_normalized = surfacenormal / np.linalg.norm(surfacenormal)
    
    #simple dot product between the two vectors. Had to clip it-  without this, values were very odd and not within range of 0-90 degrees.
    cos_theta = np.dot(vector_normalized, normal_normalized)
    angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
    
    print(f"vector- {vector}, angle- {angle:.2f}")
    
    #if the angle between the surface normal and vector is less than 35, append to valid_vectors.
    if angle <= 35:
        valid_vectors.append((entry, target))
        count_below_35 += 1

print("the number of valid vectors (angle < 35 degrees)-", count_below_35)
