import slicer # type: ignore
import numpy as np # type: ignore
import vtk # type: ignore
from scipy.spatial import cKDTree # type: ignore #Octree was not available in python, only in c++
from scipy.optimize import differential_evolution, minimize # type: ignore

#creating slicer nodes for the datasets
ventriclesnode = slicer.mrmlScene.GetFirstNodeByName("ventricles")
vesselsnode = slicer.mrmlScene.GetFirstNodeByName("vessels")
hippocampusnode = slicer.mrmlScene.GetFirstNodeByName("r_hippo")
entrynode = slicer.mrmlScene.GetFirstNodeByName("entries") 
targetnode = slicer.mrmlScene.GetFirstNodeByName("targets")  
modelnode = slicer.mrmlScene.GetFirstNodeByName('seg')
cortexnode = slicer.mrmlScene.GetFirstNodeByName('cortex')


def convert_ras_to_ijk(volumenode, RASpoint):

    #this initialises a 4x4 transformation matrix 
    rasToIjkMatrix = vtk.vtkMatrix4x4()
    
    volumenode.GetRASToIJKMatrix(rasToIjkMatrix)

    #converting the array to 4x4 instead of the current RAS 3x3, as IJK is a homogenous coordinate system and RAS is a cartesian system
    completepointinRAS = np.append(RASpoint, 1) 

    pointinIJK = [0, 0, 0, 0]

    #multiplying RAS point with transformation matrix to obtain a 4x4 IJK point with IJK and a translational 4th scaling component
    rasToIjkMatrix.MultiplyPoint(completepointinRAS, pointinIJK)  

    #removing the 4th component as it is not relevant to us, we only want IJK, which is the voxel position. Not adding an int was throwing 
    #errors as without the int datatype returned in a float
    return (int(pointinIJK[0]), int(pointinIJK[1]), int(pointinIJK[2]))


def points_in_labelmap(labelmapnode, fiducialnode):
    
    points_inside_region = []
    #for each point inside the fiducial node file 
    for i in range(fiducialnode.GetNumberOfControlPoints()):
        RASpoint = [0.0, 0.0, 0.0]
        #get the RAS position of the point i 
        fiducialnode.GetNthControlPointPosition(i, RASpoint)
        pointinIJK = convert_ras_to_ijk(labelmapnode, RASpoint)
        #GetScalarComponentAsDouble extracts the voxel intensity at that point. a non-zero intensity means the voxel is not black, adn thus belongs inside the labelmap
        if labelmapnode.GetImageData().GetScalarComponentAsDouble(pointinIJK[0], pointinIJK[1], pointinIJK[2], 0) > 0:
            points_inside_region.append(RASpoint)
    return points_inside_region

#filter targets fiducial to obtain the final set of points inside the hippocampus
targets_in_hippocampus = points_in_labelmap(hippocampusnode, targetnode)
