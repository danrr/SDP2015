# SDP2015

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