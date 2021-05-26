# Elliot's notes

## We added

* Multi-crop detection
* Cached results (so can change options whilst debugging without re-detecting all markers)
* Better handling of winged markers

## Comparison vs Marker Mapper 1.0.15 (2021-05)

They have:

* Newer version of Eigen
* Some differences in visualisation?
* exception properties have been removed from function signatures
* Seems like no changes to actual algorithm

# Prior readme

INTRODUCTION
This projects implements the research explained at: http://arxiv.org/abs/1606.00151

The project allows to determine the 3D location of a set of images of squared planar markers placed in unknown locations, along with the camera trajectory from which the images where taken.

Thus, the project allows to create a cost-effective localization system: you only need a set of marker you can print on your own house.

The library depends on Aruco for the marker detection. It also depends on OpenCv and eigen3 (the later is included in the 3rdparty directory).

The library works with OpenCv 2.4 and 3.x


ACKNOWLEDGEMENTS

If you use this work, you must reference the following papers:

@article{MUNOZSALINAS2018158,
title = "Mapping and localization from planar markers",
journal = "Pattern Recognition",
volume = "73",
number = "Supplement C",
pages = "158 - 171",
year = "2018",
issn = "0031-3203",
doi = "https://doi.org/10.1016/j.patcog.2017.08.010",
url = "http://www.sciencedirect.com/science/article/pii/S0031320317303151",
author = "Rafael Muñoz-Salinas and Manuel J. Marín-Jimenez and Enrique Yeguas-Bolivar and R. Medina-Carnicer",
keywords = "Fiducial markers, Marker mapping, SLAM, SfM"
}

@article{Aruco2014,
title = "Automatic generation and detection of highly reliable fiducial markers under occlusion ",
journal = "Pattern Recognition ",
volume = "47",
number = "6",
pages = "2280 - 2292",
year = "2014",
issn = "0031-3203",
doi = "http://dx.doi.org/10.1016/j.patcog.2014.01.005",
url = "http://www.sciencedirect.com/science/article/pii/S0031320314000235",
author = "S. Garrido-Jurado and R. Mu\~noz-Salinas and F.J. Madrid-Cuevas and M.J. Mar\'in-Jim\'enez"
}
and

@article{GarridoJurado2015,
 title = "Generation of fiducial marker dictionaries using mixed integer linear programming ",
 journal = "Pattern Recognition ",
 volume = "51",
 number = "",
 pages = "481 - 491",
 year = "2016",
 issn = "0031-3203",
 doi = "http://dx.doi.org/10.1016/j.patcog.2015.09.023",
 url = "http://www.sciencedirect.com/science/article/pii/S0031320315003544",
 author = "S. Garrido-Jurado and R. Mu\~noz-Salinas and F.J. Madrid-Cuevas and R. Medina-Carnicer"
}


INSTALATION

NOTE: This package requires aruco to be installed first.
REQUIREMENTS:
   OpenCv >=2.4 (works with 3.X as well)
   Aruco  >=2.0.19
Both can be obtained from sourceforge.


Linux:
cd path; mkdir build;cd build;cmake ..;make ;make install;

Windows:
Use QtCreator if you have never use cmake. Open the project with it, and compile. You must indicate the option -DOpenCV_DIR where you have OpenCv installed.


HOW TO USE

Use the aruco library to create a set of markers. Download aruco, http://www.uco.es/investiga/grupos/ava/node/26
compile and then, and use utils/aruco_print_marker or utils/aruco_print_dictionary programs.

Take your camera and calibrate it. You can calibrate either using ARUCO or OpenCV. Read in http://www.uco.es/investiga/grupos/ava/node/26 for more details on camera calibration.

Print the markers and use a tape measurer to know their real size.

Place the markers in the environment you want.

IMPORTANT:Markers are added to the map when they are seen in the same image with other markers already in the map.
In other words, isolated markers will not be added to the map.
Let us say you have 4 markers {A,B,C,D}. In one image, you have {A,B} and in other image {C,D}. Then ,the map will not be connected since the elements
of the first set have not been connected to the second set. You may need an image showing {A,C} or {B,C} o {A,C,D}. Whatever connection between the elements.
IMPORTANT 2: All markers must have the same size

Record a video of the markers of take several pictures of them. Make sure several markers appear in the images to create the marker graph.

If you recorded a video (e.g., video.avi) use the following program of the library
./utils/mapper_from_video video.avi camera.yml 0.1 -out markerset -d dict

The first parameter is the video file to be used.
camera.yml is the camera calibration file.
0.1 is the marker size. If you do not know the size set it to 1.
-o map: base name of the files that will be generated as result of this
-d <dict> indicates the dictionary of markers employed. Since ARUCO can detect different type of marker dictionaries (ARUCO,APRILTAGS,ARTOOLKIT+,...), you must indicate which one you use. If not specified, then ARUCO will be employed.
The possible values are printed if you run the program without arguments.

When the program runs, you'll see the video frames and in red the detected markers. If markers are enclosed in a red line, then they are not detected. You may have set an incorrect dictionary.
When all frames are processed, the optimization start. At the end, the following files will be generated:

markerset.yml: the marker map with the 3D location of the markers. This file can be used by the Aruco library for camera localization purpouses.
    Take a look at Aruco. You can run the Aruco  program > aruco_path/utils_markermap/aruco_test_markermap video.avi map.yml -c camera.yml -s 0.01  to test it.
markerset.pcd: a point cloud file (Point Cloud Library PCL) showing the markers and the frames employed to calculate their location. You can visualize the with the pcl_viewer program of the PCL library.
markerset.log: a file indicating the 3d locations of the views employed for reconstruction, i.e., the camera locations. It is in csv format the same way than RGB-TUM dataset:
 #frame_id tx ty tz qx qy qz qw
markerset-cam.yml: in the final step of the optimization, the camera parameters initially provided are optized using the video sequence employed. This file is the optimized camera
parameters that best first the video data. You can use it  with aruco for tracking pourposes instead of the original one. However, if the video footage you provided has very few images, the optimized camera parameter might not
be general enough and not provide good results. Then, forget about it. In general, the original cam.yml will do a good service in the tracking with ArUCO.



By default, the program will process all images. You can skip frames by setting the parameter -fi X, so the program will only process frames X,2X,3X, ... etc

By default, the program will process until the end of the video. If you want to stop before, just press ESC and the optimization will start.

When the optimization finishes, the program will show a 3D representation of the map generated (map.yml).  You can visualize the result whenever you want using the programm mapper_viewer passing the markerset.yml file as argument.




You can also process instead of a video, a image set using mapper_from_images.



DATA SET:

There is a data set available for testing you can download at https://sourceforge.net/projects/markermapper/

KNOWN BUGS:

If the graph contains more than one components (i.e., there are markers unconnected from others), the program will have a Seg Fault.

REPORT  ISSUES



If you find bugs, please put the video and camera parameters somewhere I can download it. Also, indicate me the console output of the program and the dictionary employed.
Send an email to ucoava@gmail.es with the subject MARKERMAPPER BUG

For any other additional information or queries, please send an email to the same email address.




