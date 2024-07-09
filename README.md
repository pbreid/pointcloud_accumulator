# Pointcloud Accumulator

## Overview

The Pointcloud Accumulator is a ROS package designed to process .bag files containing pointcloud data. It specifically looks for PointCloud2 or Livox custom message types, accumulates all points into a global pointcloud, and outputs the result as a .pcd file. This tool is particularly useful for combining multiple pointcloud messages from a recorded ROS bag into a single, comprehensive pointcloud file.

## Features

- Processes ROS bag files containing pointcloud data
- Supports both PointCloud2 and Livox custom message types
- Accumulates points from multiple messages into a single global pointcloud
- Outputs results in .pcd format
- Handles various intensity/reflectivity data formats

## Prerequisites

- ROS (tested on ROS Noetic, but should work on other recent versions)
- PCL (Point Cloud Library)
- Catkin workspace set up

## Installation

1. Clone this repository into your catkin workspace's `src` directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/pbreid/pointcloud_accumulator.git
```

2. Build the package:

```bash
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:
source ~/catkin_ws/devel/setup.bash

## Usage

Run the pointcloud accumulator node with the following command:

```bash
rosrun pointcloud_accumulator pointcloud_accumulator_node <bag_file> <output_file> [topic_name]
```

- `<bag_file>`: Path to the input .bag file
- `<output_file>`: Path for the output .pcd file
- `[topic_name]`: (Optional) Name of the topic to process. If not provided, the node will prompt you to choose from available topics.

Example:
```bash
rosrun pointcloud_accumulator pointcloud_accumulator_node /path/to/your/bagfile.bag /path/to/output.pcd
```

If multiple suitable topics are found in the bag file, you will be prompted to choose which one to process.

## Configuration

The node is configured to handle both PointCloud2 messages and Livox custom messages. It looks for 'intensity' or 'reflectivity' fields in the pointcloud data and scales them appropriately.

## Output

The output is a .pcd file containing the accumulated pointcloud data. Each point in the output cloud includes X, Y, Z coordinates and an intensity value.

## Troubleshooting

- If you encounter "Failed to find match for field 'intensity'" errors, check that your input pointcloud data contains either an 'intensity' or 'reflectivity' field.
- Ensure that the topics in your bag file contain either PointCloud2 or Livox custom message types.

## Contributing

Contributions to improve the Pointcloud Accumulator are welcome. Please feel free to submit pull requests or open issues to suggest improvements or report bugs.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- Thanks to the ROS and PCL communities for their excellent libraries and documentation.

## Contact

For any queries or suggestions, please open an issue in the GitHub repository.
