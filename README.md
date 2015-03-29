# SDP2015

![Super-DH 3000 NG Ultimate](https://github.com/danrr/SDP2015/blob/master/logo/Logo.png)

###Setup

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


### Git ####

####Workflow:

* always keep master in a working state, do not commit any broken code to master
* small commits can go directly on the master branch if you are 100% that they're OK (do relevant testing, make sure it's working with everything else)
* larger pieces of work should be done on a branch with a descriptive name
* before merging a branch back in, make sure that your code will work with the code currently on master:
  * try rebasing your branch off of the current master: 
     * `git checkout <my-amazing-branch>`
     * `git rebase master`
  * if this gives you get a merge conflict, abort the rebase:
     * `git rebase --abort`
  * and just do a regular merge of master into your branch:
     * `git merge master`
  * if you still get a merge conflict, suck it up and fix it (using a graphical merge tool is strongly encouraged)
  * now you should be at a state where you can test your code with the latest master (having someone else review the code at this stage might catch some errors as well)
  * if everything is alright merge the code into master by doing:
    * `git checkout master`
    * `git merge <my-amazing-branch> --no-ff`
  * the `--no-ff` makes git not fast-forward master even if it can which is useful in case all the changes on a branch need to be reverted at once
  * delete the old branch:
     * `git push origin :<my-amazing-branch>` 

####Random pointers for git:

* when pulling after local commit have been made use `git pull --rebase` which will rebase your local commits on top of the master on origin (makes the tree  look less of a mess)
* use branch names that describe what you are doing; using tokens might be useful if there are a lot of branches: vision/tweaking-calibrations
* run `git remote prune origin` to stop tracking branches that are no longer on origin


###Vision

 * Open terminal and run ./camera_setup.sh.x 
   * This resets the hardware settings for the video feed, manual controller for this underway
 * Run python calibrate.py 0 (or 1 depending on the pitch)
   * Follow the instrutions in terminal, to exit immedietly press q
   * First specify by 8 dots the outline of the pitch, the dots go to black corners, including little of the black part of the pitch is Ok, slightly prefered, this just specifies for the camera where to look. Press q
   * By 6 dots specify the left defender, including the white lines and half of the white line between defender and attacker, specifing the zone for the defender. Press q
   * By 4 dots specify the right attacker, including white lines. Press q
   * Follow this process
   * This calibrates the table, specifing the boundries the table and the zones
 * Now you can run the vision code, controller.py
   * To exit press ESC this saves the current settings
   * move the video out of the way to see windows with title mask plate, or to get to this screen press p - for plate
     * Here Adjust values to see all for plates on the screen, plates in white, background black and the i's inside the plates also black.
     * This specifies the bounding square for plates, this improves jitterness
   * Press d for dot
       * Adjust values to see 4 white dots on screen, everything else in black
       * This identifies the direction of the robot
   * Press r for red
       * Adjust values to see just the red ball in white, everything else in black
   * Tinker with these values and plate positions on the pitch, as the lighting may be different at different parts of pitch
   * Press ESC to exit and save these values.
   
   
####Vision - manual calibrations - all the sliders
 * For best results turn calibrate option on in the main gui (slider above the feed). This captures a short loop of the video and displays it.
 * When varying the values the amound of jitters displayed in the top right part of the screen is helpful. The goal is to have have them as small as possible.
  
 
 * Adjusting the main vision sliders manually from scratch, the settings for D - dot, R - red and P - plate
   * Open vision and the window mask plate
   * Adjust plate settings by pressing P
     * Set CT to 100 
     * BL to small value +- 4
     * All upper bounds UH,US,UV to max value
     * vary lower bounds LH,LS,LV around 60 to get rid of noise
     * Now move UH to around 100 to get rid of the red ball
   * Adjust Dot by pressing D
     * Set CT to 100
     * BL to 0
     * Set UH,US to max value and LV to 0
     * Vary UV around 170. Vary LH and LS around 30 to get rid of noise
   * Adjust Red by pressing R
     * Set CT to 0
     * BL to small value +-8
     * Set US,UV to max value. LH to 0
     * Vary UH around 10. Vary LS and LV to get rid of noise
     * Alternatively, if the ball sometimes disappears with the above settings, try setting CT to around 70 and set the following settings: http://i.imgur.com/XxpfwbA.jpg This should make the ball appear more reliably.
     
###Money
* Holonomic wheels: £11 * 3
* Compass: £19.99
* Extra motors: £1.50 * 3
* Servo: £12
* PCB: £??
* TOTAL: £69.49
