# Traffic Heatmap Generation on Top of Mapping

# Introduction
This is the coding part of the traffic heatmap. This project is to create a traffic heatmap based on the radar dataset.



![Screenshot Capture - 2021-04-14 - 12-39-45](/home/kardel/Downloads/Screenshot Capture - 2021-04-14 - 12-39-45.png)

# Membership
Yufeng Chen
Nitish Sanghi
Tingjun Li
Hyoensu Do
Ruochen Hou

# Basic workflow

```flow
st=>start: Dataset Collection
op1=>operation: Ground Segmentation
op2=>operation: Clustering
op3=>operation: Boxing
op4=>operation: Tracking
op5=>operation: Visualization
e=>end

st->op1->op2->op3->op4->op5->e

```
# Instruction

## Environment

System: Ubuntu 18.04 LTS

Python 3.6 (System Interpretor)

MATLAB R2021a

bash: /bin/bash

## Prerequisite

python-pcl

Computer Vision Toolbox

Image Processing Toolbox

Sensor Fusion and Tracking Toolbox

## Execution

It can be runned by run.sh

`/bin/bash run.sh`