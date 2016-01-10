# TopViewImageToPointCloud
## Overview
- Input RGB image will be converted to point cloud according to the user-specified recipes
 - The image is regarded as a top-view image
 - 3D viewpoint position and other detailed information must be specified by user
- Point cloud is saved as a PLY file

## Requirements
- CMake 2.8 or more
- Point Cloud Library 1.7.2 or more
- OpenCV 2.x or 3.x

## How to build
1. Install the above-mentioned libraries
2. Edit OpenCV paths information in CMakeLists.txt
3. Do cmake

## How to use
### Recipes
- Recipe describes how to convert 2D colorized area to 3D point cloud in one line separated by comma
- Recipes must be written in a CSV file
- Each line describes one 2D-to-3D conversion
- Format of one line is:
  `R, G, B, num_of_pt, min_height, max_height, horizontal_noise_half_width`
- if first character is sharp (#) of a line, it will be ignored (regarded as a comment)

### Command line arguments
|arg|value type|description|
|---|---|---|
|-i|string|input image path (image must be readable by `cv::imread`)|
|-r|string|input recipes path (CSV file)|
|-o|string|output point cloud path (PLY file)|
|-vp|double,double|viewpoint position on 2D image (separated by comma like *100,100*)|
|-mmp|double|meter per pixel|

## Example
- Command line
 - `-i ../input/sample1.png -r ../input/recipes.csv -o ../output/cloud.ply -vp 150,230 -h 1.0 -mmp 0.01`
- Results
 - Input  
  ![Input](https://github.com/kenkenjlab/TopViewImageToPointCloud/blob/master/input/sample1.png)
 - Output  
  ![Output](https://github.com/kenkenjlab/TopViewImageToPointCloud/blob/master/output/results_exmaple.png)
