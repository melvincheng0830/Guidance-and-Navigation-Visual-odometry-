# Guidance and Navigation Visual odometry

This repository contains multiple MATLAB program that is related to StructureFromMotion and VisualSLAM. You can find a brief introduction to visual odometry and a step-by-step tutorial below on how to use GitHub and MATLAB together. You can also find tutorial videos at the end of this page for more guidance.

## Sample Images

You can download the sample images here: <strong>[Download Link](https://drive.google.com/file/d/1kzwPI4-icW6AZy3HvoAMiHtdBScn7AKa/view)</strong>

## About Visual Odometry

Tutorial slide for Visual Odometry: <a href="Week 9 [Tutorial 2 Visual Odometry] Dr. Wen-V3.pdf"><strong>Link</strong></a>

Visual odometry in robotics and computer vision is a process of analyzing camera images in order to determine the position of a robot. This technology is widely adapted in various fields of applications, including exploration rovers used in planatory explorations.

<p align="center">
  <img width="512pix" src="https://scitechdaily.com/images/NASA-Mars-Perseverance-Rover-Driving.gif">
</p>

In unmanned aerial robotics, visual odometry can be utilized to achieve multiple purposes, such as surveying & mapping. Visual odometry can also be used to estimate the location of a UAV, recovering relative translation and rotation (Motion) by tracking image features over time using a calibrated camera model.

<p align="center">
  <img width="512pix" src="https://www.researchgate.net/profile/Vitor-Guizilini/publication/224252287/figure/fig1/AS:669965199880222@1536743559940/3D-localization-estimates-obtained-from-the-proposed-method-with-six-Single-GPs_Q320.jpg">
</p>

## MATLAB and Visual Odometry

Link to MATLAB Visual Odometry: <a href="https://www.mathworks.com/help/vision/ug/monocular-visual-odometry.html"><strong>MATLAB Visual Odometry</strong></a>

## MATLAB and GitHub

Introductory slide for GitHub and MATLAB collaborative coding: <a href="Week 13 Intro GitHub and MATLAB.pdf"><strong>Link</strong></a>

<p align="center">
  <img width="712pix" src="Images/Workflow.png">
</p>

MATLAB and GitHub can be used together to better achieve a collaborative coding and developing environment between developers or programmers. The setup of using MATLAB and Git would only take a few steps:

### 1. Install both MATLAB and Git in the local Machine

You can find the installation link for Git here: <a href="https://git-scm.com/downloads"><strong>Link</strong></a>

MATLAB Official Website: <a href="https://www.mathworks.com/products/matlab.html"><strong>Link</strong></a>

You will also need to have a GitHub account for this. You can create one in the GitHub webpage: <a href="https://github.com/"><strong>Link</strong></a>

### 2. Create a Repository / Join a repository where you have editing permission

You can create a repository by clicking on the "New" button on the left panel.

Alternatively, repository owners can invite collaborators to a repository in "Settings" --> "Collaborators".

### 3. Copy the repository link

You can copy the repository link at the front page of a repository by clicking the green "Code" button.

<p align="center">
  <img width="712pix" src="Images/Copy Repo link.png">
</p>


### 4. Link MATLAB and the GitHub repository

Open MATLAB and right click on the left empty space. You will see the option "Source Control". Click on it and paste the GitHub repository link that you just copied and click "Retrieve". Wait until the repository has been pulled to the local machine when you see the files appear on the left hand side.

<p align="center">
  <img width="512pix" src="Images/SC matlab.png">
</p>

<p align="center">
  <img width="712pix" src="Images/Retrieve.png">
</p>

### 5. Start Editing!

After the local repository is created by retrieving from the cloud repository (Step 4 above), you can start editing the codes!

<p align="center">
  <img width="712pix" src="Images/SC Options.png">
</p>

There are multiple source control functions as shown above. You can know more about them through this tutorial: <a href="https://medium.com/mindorks/what-is-git-commit-push-pull-log-aliases-fetch-config-clone-56bc52a3601c"><strong>Link</strong></a>

### 6. Pushing changes back to GitHub

Remember to use the "Push" function to sync everything back to the remtoe repository, otherwise everything will just stay in the local machine!

## MATLAB and GitHub Tutorial Videos

Here are some tutorial videos for your to follow:

### Offical MATLAB & Git Tutorial: <a href="https://www.youtube.com/watch?v=O7A27uMduo0&ab_channel=MATLAB"><strong>Link</strong></a>

### A Step-by-step version: <a href="https://www.youtube.com/watch?v=g_P2ulWBVlE&ab_channel=WeisongWen"><strong>Link</strong></a>

## Acknowledgment

We wish to show our appreciation for the code from the MATLAB official community (https://ww2.mathworks.cn/help/vision/ug/structure-from-motion-from-multiple-views.html) for SFM and https://ww2.mathworks.cn/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html for visual SLAM. We use the dataset provided by the TUM (https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz). Thank you very much for the help from Runzhi Hu and Xi Zheng.
