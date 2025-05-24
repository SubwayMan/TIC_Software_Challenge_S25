# Toyota Software Innovation Challenge – Vehicle Automation

  
## Code Documentation
The documentation for the start code can be found here: 

https://uofwaterloo.sharepoint.com/:w:/s/tm-eng-engineeringideasclinic/EViGUSkADoRAo0SAcYJBfS0B4q_ESYHth7W70o-UvChApw?e=BzTRPD

Google Drive can be found here:

https://drive.google.com/drive/folders/1U0qSvJUjqUBoHz40CjrrwecNmML-Dhgb?usp=sharing

## Getting Set Up
Controlling the TurtleBots requires ROS2, which only works on Linux devices. To get around this on Windows or Mac systems you can install a Virtual Machine to emulate a Linux system. If you are already running Linux or have a VM installed, skip to the **Environment Setup** section.

### Installing a Virtual Machine
**Note: Make sure you have at least 50GB of free space on your computer before going through this section. If this is an issue, please inform one of the Ideas Clinic staff.**
1. Download and install VMWare Workstation 17 Player from [here](https://www.techspot.com/downloads/1969-vmware-player.html). <br><br> Note: if you have a Mac that runs with an Intel processor, use [this link](https://www.techspot.com/downloads/2755-vmware-fusion-mac.html) to download and install VMWare Fusion 13.5.2 (the Mac equivalent for Workstation).
  - Just click next throughout the installation

2. Download this Ubuntu 22.04 ISO file (the 64-Bit Desktop Image) or download and extract the ubuntu-22.04.4-desktop-amd64.iso zip file located in the Sharepoint folder.

3. Select “Create a new Virtual Machine”.

4. Select “Installer disc image file (iso)” and select the downloaded Ubuntu 22.04 ISO file. (VMWare should automatically detect Ubuntu 22.04 as the operating system).  

5. Follow the on-screen prompts to progress through the installation (including choosing an install location and creating a Linux account). Do not change any options from the defaults, except setting your disk capacity to 50GB.

6. Before clicking “Finish”, click Customize Hardware. Set the memory to the maximum recommended amount (you can do so by clicking on the blue arrow), and set the number of processors being used to 4.  

7. The virtual machine will then “power on” to continue installation. Follow the on-screen prompts when required, leaving the default options. Note: please also install the VMWare Tools for Linux to complete the Easy Install. 

8. Once the installation is fully complete, you will be asked to restart the “computer”. This will not affect your system, only the virtual machine.

9. Once the restart is complete, use the username and password you created as part of step 5 to sign in to your Linux account. You are now ready for the next step: setting up your environment for this challenge.

### Environment Setup
1. To set up your environment, you will have to first clone this repository. This will require git to be installed. Open up the terminal and enter the command: ```sudo apt install git```.
2. Clone this repository into your home directory by running the command: ```git clone https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25.git``` in your home directory. <br>**Note:** If you are prompted for a username and password, you will need to create a personal access token to sign in with. Find instructions for doing that [here](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic). Copy the token you make and use that as your password when you sign in.
3. Once this is complete, you are ready to set up your Docker environment. Instructions for this can be found in the [turtlebot4_sim_docker](https://github.com/IdeasClinicUWaterloo/TIC_Software_Challenge_S25/tree/main/turtlebot4_sim_docker) folder. The README in this folder has all the instructions you need for setting up your simulation and connecting to an actual turtlebot.

## Coding Your Solution
Once the challenge is live, you will be able to see the challenge description for what your solution should look like. We have provided you a space to put your solution inside the solution.py file, though you are able to change the provided files or create your own to best suit your needs. There are six parts to the challenge, but you do not have to complete them all - focus on coming up with the best solutions you can for the whole challenge. 
### Remember that you are not just marked on how far you get into the challenge, but your presentation and design choices as well.
