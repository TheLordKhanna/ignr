Welcome to this slicer readme

You will need - 
1) 3DSlicer
2) Data (CT scan volumes in our case + entry and target points). When loading into scene, select 'label map' for volumes 
3) OpenIGTLinkIF installed from extension wizard
4) python console (accessible from top right corner)

Although there are three different code blocks (slicer_target_filtration, slicer_filtered_vectors and slicer_optimisation_code), all three can be implemented one after each other in console. Please implement them in this order. Additionally, take care to rename your created nodes in slicer_target_filtration according to the name of your data. 

Generally speaking - 
slicer_target_filtration creates nodes for data, writes a function to convert RAS points (in fiducial nodes) to IJK points and a function to check whether that point is in a given labelmap. Our task was to only use those targets within the hippocampus, so the result of this code block is a set of targets that are valid. 

slicer_filtered_vectors implements our hard constraints. If a vector (entry-target pair) intersects either or vessel and ventricle labelmap, it is removed (since it is unsafe). Additonally, if a vector has an insertion angle more than 35 degrees with the surface normal, it is removed. 

slicer_optimisation_code implements our soft constraint- maximising clearance from vessels and ventricles to output a single, ideal vector. It does this by using a condensed KDTree to perform position lookup and differential evolution from scipy.optimise to find the best fit vector using a population based approach. 

A fourth code block, slicer_unit_tests, implements unit tests for functions in slicer_target_filtration and the angle condition in slicer_filtered_vectors. To execute it, you must paste the functions that are being tested into the console, clear the scene and then paste the unit tests. This will not automatically call the unit tests - you must paste -

tester = testtrialfunctions()
tester.runTest()

into the console to run the tests. 

Individual functions for all of the above are explained in more detail in comments in their code blocks 


Common problems - 
1) Object NoneType - usually due to slicer not being able to find your node. check the names to see if you have declared them properly 
2) dimensions mismatch - make sure you are appending 1 to RAS, and standardising the use of IJK all over the code
