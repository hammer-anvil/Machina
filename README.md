# Machina Metrology Task
A ROS 2 package for hand-eye calibration and 3D scanning, implementing:

 - Eye-in-hand calibration using two different methods.

 - Point cloud processing for noise filtering and alignment of overlapping scans.

## 1. Setup and Build

### Build the Docker Image

This package is designed to run in a Docker environment with Ubuntu 24 and ROS 2 Jazzy.

To build the Docker image, run the following command in the directory containing the Makefile:

```bash
make build
```

### Run the Docker Container

Once the image is built, launch the container:

```bash
make run 
```

### Open a New Terminal in the Running Container

For additional terminals within the container:

```bash
make bash 
```

## 2. Running the ROS 2 Nodes

### Launch the Calibration & Point Cloud Nodes

To start all required nodes:

```bash
ros2 launch machina_metrology_task machina_task.launch.xml
```

## 3. Eye-in-Hand Calibration

### Trigger Calibration

To start the calibration process:
```bash
ros2 service call /trigger_calibration std_srvs/srv/Trigger
```
The service call will trigger calibration based on the data provided and outputs the results on the console. 
The result will also be saved in the output directory within the package for further analysis. 

## 4. Point Cloud Filtering & Alignment

### Trigger Point Cloud Processing

To start filtering and alignment:
```bash
ros2 service call /filter_and_align std_srvs/srv/Trigger
```

This service call initiates the filtering and alignment process of the raw point clouds, which will be visualized in RViz. 
The processed point cloud data, aligned_scan.csv, will be saved in the output directory within the package for further use. 


## 5. Data Generation

Pre-generated data are available in the data directory within the package.

### Calibration Data

To generate flange and scanner poses:

```bash
ros2 run machina_metrology_task data_generator_calibration
```
### Point Cloud Data

To generate two raw point cloud data:

```bash
ros2 run machina_metrology_task data_generator_pcd
```