# ROS2BAG TO PCD CONVERTER

Simple package that converts point cloud from ros2bag pcd.

## How to run
Docker and docker compose are required. When docker is installed then run following commands in the repository folder
```bash
mkdir bag pcd
```
Copy your ros2bag to bag folder and add your topic name to .env file and after that run:
```bash
docker compose -f compose.pcl_bag_to_pcd.yaml up --build
```
Results will be stored in pcd folder.