import slicer
import unittest
import numpy as np
from slicer.ScriptedLoadableModule import ScriptedLoadableModuleTest
from slicer.util import loadVolume


class testtrialfunctions(ScriptedLoadableModuleTest):
    #testtrialfunctions inherits from ScriptedLoadableModuleTest
    def setUp(self):
        #setUp is a typical function used to load it all relevant data into the scene 
        slicer.mrmlScene.Clear()
        #remove all current data 
        self.delayDisplay('Loading testing dataset......... hippocampus')

        #path to file - modify as needed
        path = 'C:/Users/rajat/Downloads/TestSet'
        #the two volume nodes we will be needing in unit tests
        volumeNode = slicer.util.loadVolume(path + '/r_hippoTest.nii.gz')
        volumeNode1 = slicer.util.loadVolume(path + '/r_cortexTest.nii.gz')


        if not volumeNode:
            self.delayDisplay('Unable to load file')
            return
        #condition to confirm unit test works 
        if not volumeNode1:
            self.delayDisplay('Unable to load file')
            return
        
        self.delayDisplay('File loaded successfully')


        #converting the loaded volume node into a labelmap. for some reason, when included in setUp, this doesn't work. Still trying to figure
        #out why 
        labelmapNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLabelMapVolumeNode")
        slicer.modules.volumes.logic().CreateLabelVolumeFromVolume(slicer.mrmlScene, labelmapNode, volumeNode)
        hipponode = slicer.mrmlScene.GetFirstNodeByName('LabelMapVolume')


        #creating a dummy fiducial node with a known RAS point - 1, 1, 1
        Fiducialnode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        Fiducialnode = slicer.mrmlScene.GetFirstNodeByName('MarkupsFiducial')
        Fiducialnode.AddControlPoint(1, 1, 1)

        labelmapNode_1 = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLabelMapVolumeNode")
        slicer.modules.volumes.logic().CreateLabelVolumeFromVolume(slicer.mrmlScene, labelmapNode_1, volumeNode1)
        cortexNode = slicer.mrmlScene.GetFirstNodeByName('LabelMapVolume_1')


    #the first unit test, to check points_in_labelmap and whether it filters out the known RAS point given the hippocampus node
    def checkpointinhippo(self):
        hipponode = slicer.mrmlScene.GetFirstNodeByName('LabelMapVolume')
        Fiducialnode = slicer.mrmlScene.GetFirstNodeByName('MarkupsFiducial')
        hippopoints = points_in_labelmap(hipponode, Fiducialnode)
        if hippopoints  == []:
            print('no points found!!')

        if hippopoints > []:
            print('points found!!')

    #unit test number II. This is used to test convert_ras_to_ijk by checking the converted IJK value of the known RAS point. 
    #1, 1, 1 in RAS = 0, 0, 0 in IJK
    def checkifIJKiscorrect(self):

        hipponode = slicer.mrmlScene.GetFirstNodeByName('LabelMapVolume')
        raspoint  = Fiducialnode.GetNthControlPointPosition(0)
        checkpoint = convert_ras_to_ijk(hipponode, raspoint)
        print(checkpoint)
        if checkpoint == (0, 0, 0):
            print('pointiscorrect!!')
        else: 
            print('point is not correct!!')

    
    #unit test III - Checking the insertion angle logic
    def checkinsertionangle(self):


        #four vectors, 2 of which are known to be valid vectors, and two contstructed manually at very acute angles
        testFilteredVectors = [([200.2784, 169.2371, 122.0490], [187.804, 102.3299, 122.0490]), ([185.536, 13.8763, 130.0490], [179.8660, 121.6082, 130.0490]), ([207.525, 123.882, 126.768], [146.0, 114.0, 133.0]), ([206.867, 126.505, 130.049], [166.0, 74.0, 138.0])]
        test_valid_vectors = []
        test_count_below_35 = 0
        cortexNode = slicer.mrmlScene.GetFirstNodeByName('LabelMapVolume_1')

        for entry, target in testFilteredVectors:
            # call find_first_intersection. this should find the first point of intersection for our entry-target vector pair on cortexNode
            intersection, surface_normal = find_first_intersection(entry, target, cortexNode)
            if intersection is None:
                continue

            vector = np.array(target) - np.array(entry)
            vector_normalized = vector / np.linalg.norm(vector)
            normal_normalized = surface_normal / np.linalg.norm(surface_normal)
            #dot product, clipped to prevent values above pi/2
            cos_theta = np.dot(vector_normalized, normal_normalized)
            angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
    
            print(f"Vector: {vector}, Angle: {angle:.2f}")
            #update count to print which vector, the angle and how many were accepted
            if angle <= 35:
                test_valid_vectors.append((entry, target))
                test_count_below_35 += 1

        print("Total valid vectors (angle <= 35°):", test_count_below_35)



    def runTest(self):
        self.setUp()
        self.checkpointinhippo()
        self.checkifIJKiscorrect()
        self.checkinsertionangle()
