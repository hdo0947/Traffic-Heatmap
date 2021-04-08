# Ground Segmentation
## Introduction
This is a ground segmentation tool in python

## Usage
It can be used by 
``python main.py python3 main.py --input_filename [input pcd/bin file] --output_ground_filename [output ground file in .pcd format] --output_notground_filename [output not-ground file in .pcd format]
``

## Requirement 

It requires
- System python3 
- python-pcl installed by
```shell
sudo add-apt-repository ppa:sweptlaser/python3-pcl
sudo apt update
sudo apt install python3-pcl
```
- open 3d 
- Numpy 