# SonarImage_to_Occupancygrid

## Filter
The filter takes in the flsdata and publishes an image after every step of the filter algorithm. 

The first step is removing the gain. If you choose to not remove any gain, the publisher will just publish the image received from the fls, otherwise it will publish the fls image with the VGA removed. The image is then published under the topic "gain_removed"

The second step is using an image buffer to compare consecutive images and remove random noise. The image is then published under the topic "fused"

The third step is linear contrast stretching which make the white parts of the image whiter and the black parts placker. The image is then published under the topic "fused_and_stretched"

The fourth step is a simple median filter to remove salt and pepper noise that may have survived thus far and to smoothen out the image for the next step. The image is then published under the topic "median_filtered"

The fifth step is called otsu filtering. It turns the image into a binary image by finding a threshold that has the maximum variance between the dark parts and the bright parts of the image. The image is then published under the topic "otsu"

The sixth and last step is mathematical morphological operations. Here we dilate and then erode, this will connect detections that are close and smoothen out edges of the obstacles.
The image is then published under the topic "morphed"

## Grid
This node receives the "morphed" image and transforms the pixels to points on an occupancygrid. The occupancy grid is 2D and is perpendicular with the FLS. Basically you see everything the sonar sees from a side view like so:
    _______________________________________
    |                                      |
    |  =                                   |
    |                                      |
    |    _____                             |            THE EQUALSIGN IS BASE_LINK
    |          _________ ________          |
    |______________________________________|

The grid is made by first extracting the ones in the "morphed". Their position in fls link is then calculated before transforming the points into map frame. The points are then put into their respective cells in the occupancygrid. The value of the cell depends on how many times the same cells has been seen. For example, if in an iteration, an obstacle is seen in a cell, the new value of said cell will be its current value + fade_factor (the limit is 100). The fade_factor can be chosen in the launch file. The same goes for when there is no obstacle in a cell, its new value will be its current value - fade_factor (the limit is 0). This means that if an obstacle was randomly seen in one iteration but immedieatly dissappears in the next, the "fake" obstacle will fade out immediatly. However if an obstacle is seen for a long time, then randomly dissappears in just one iteration, the obstacle will still be shown in the grade but with a lighter shade. This can prove useful if you want to use som obstacle avoidance since the cells value represents some kind of confidence. 

Another thing to know is that if for example the sonar travels a longer distance than the grid-resolution before a new image has been received, the occupancy grid will move the cells accordingly so the correct cells have their corresponding confidence. Otherwise what would happen is that as you move forward, obstacles would "fade" towards you instead of keeping their confidence constantly.

Keep in mind right now the transform between the fls_link and base_link has to be manually set. There are two images that show an example of how the transform should be set if the sonar was pointing down with an angle of around 45 degrees. Also depending on how the image looks, you have to flip "morphed" so that the sea bottom starts from top left. There are two images in this catkin package where one is an example where flipping is not needed and the other is an example of when it is needed. Both the images also show how the frames are defined in the sonar image. Setting the transform and flipping the image can be done in the launch file.


## launching filter and grid

Both of the nodes are launched by writing "roslaunch detection detection.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)".

The second part of the command (2> >(grep -v TF_REPEATED_DATA buffer_core)) is used to remove annoying warnings about ignoring frames that have been published in a too high rate. These warning occur when launching the grid node but i can't seem to find the source of the warning.


