# SDP2015

#### How to install OpenCV on computers that support camera feed: ####

1. Download OpenCV: http://sourceforge.net/projects/opencvlibrary/files/

2. Extract the contents into anywhere on your disk

3. Navigate to /disk/scratch/sdp/ and create a directory, call it anything you like.

4. Copy extracted contents into this directory.

5. Once the copying is finished, stay where you are (root of OpenCV contents), and create a directory called "build". Navigate inside it.

6. Execute: cmake -D CMAKE_INSTALL_PREFIX=~/.local ..

7. Then execute: make

8. You'll very likely get an error at around 64% of the installation. 
	It'll look like this:
	CMake Error at /disk/scratch/sdp/ocv/cmake/cl2cpp.cmake:50 (string):
	So just simply comment out that line in the specified file, and save it.
	Once that's done, execute "make" again just like before. This time it'll run faster.

9. Then execute: make install

10. Run ipython and do import cv2, if all executes fine then you're set.)
