<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Apache-2.0 License][license-shield]][license-url]


<!-- PROJECT TITLE -->
<br />
<div align="center">
<h3 align="center">myBuddy 280 with Robonomics for ROS 2</h3>

<p align="center">
    Python ROS 2 packages for control of the myBuddy 280 manipulator through integration with Robonomics parachain
</p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About Project</a>
      <ul>
        <li><a href="#project-structure">Project Structure</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation-and-building">Installation and Building</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
    </li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About Project

This ROS 2 Python packages provide interface for usage of 
[**Robonomics ROS 2 wrapper**](https://github.com/Fingerling42/robonomics-ros2/) with 
[**myBuddy 280**](https://shop.elephantrobotics.com/products/mybuddy-280), a double 6-DOF dual-arm collaborative robot 
created by [Elephant Robotics](https://www.elephantrobotics.com/en/). The robot consists of two myCobot 280 manipulators 
with ATOM Matrix ESP32-PICO boards, 7" sensor display, two built-in cameras and Raspberry Pi 4B as the main computing 
unit.

The goal of the project is to provide the ability to remotely control a robot and receive its telemetry via secure and 
decentralized cloud of Robonomics Network. [Robonomics](https://robonomics.network/) allows to store digital twins 
of robotics and IoT devices and get access to them through the network.

Current available features include: 

* a custom robot driver for ROS 2 based on [pymycobot](https://github.com/elephantrobotics/pymycobot) library, 
a Python API for Elephant Robotics products: the driver can get states of each joints for both arms and wrist as well 
as send angles to them;
* automatic sending of robot joint states using Robonomics datalogs;
* a remote joint control by sending a launch function to the robot's address.

> **DISCLAIMER**: There is an official [ROS 2 package](https://github.com/elephantrobotics/mycobot_ros2/) for the 
> robot provided by the developers. However, some necessary API functions are missing in this package, 
> so a custom driver is used here. Keep in mind that the custom driver is still in development.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Project Structure

For convenience, the project is divided into several ROS 2 packages:

    .
    ├── mybuddy280_driver           # A package with the custom ROS 2 driver for myBuddy 280
    ├── mybuddy280_interfaces       # A package that describes types of robot's services and messages
    ├── mybuddy280_robonomics       # Main package with Robonomics handler for myBuddy 280
    └── ...

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

Make sure you have the following software installed on myBuddy 280: 

* Linux OS distribution (tested on [Ubuntu 20.04.4](https://releases.ubuntu.com/focal/))
* ROS 2 distribution (tested on [Galactic version](https://docs.ros.org/en/galactic/Installation.html))
* [Python 3](https://www.python.org/downloads/) (tested on 3.8.10)
* A [Robonomics ROS 2 wrapper](https://github.com/Fingerling42/robonomics-ros2) package and all its required software (especially IPFS)

You also need to create an account on Robonomics parachain and write down seed phrase and account type. Make sure, 
that you have a free account balance to make transactions. The guide is available [here](https://wiki.robonomics.network/docs/create-account-in-dapp). 
After creating an account, do not forget to insert the seed phrase to the `account_params.yaml` file of Robonomics 
ROS 2 wrapper package. 

> **WARNING**: The seed phrase is sensitive information that allows anyone to use your account. Make sure you don't 
> upload a config file with it to GitHub or anywhere else.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation and Building

1. Open a console instance on your robot directly or via SSH.

2. Create directory for ROS 2 workspace and the `src` subdirectory:
    ```shell
    mkdir -p your_project_ws/src
    cd your_project_ws/src
    ```

3. Clone this repo to `src` directory:
    ```shell
    git clone https://github.com/Fingerling42/mybuddy280-ros2-robonomics.git
    ```

4. Build the package from `your_project_ws` directory:
    ```shell
    colcon build
    ```
   
5. Source the package to the environment (you will need it to do in every console instance or add it to `.bashrc`):
    ```shell
    source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

1. First, launch IPFS Daemon:
    ```shell
    ipfs daemon
    ```

2. In separate console instance run the ROS 2 launch file from driver directory. It will launch main driver node a node 
that will connect to the robot via a serial port:
    ```shell
     ros2 launch mybuddy280_driver mybuddy280_ros2_launch.py
    ```
   
3. In another console instance run the main launch file with Robonomics handler for the robot (do not forget about 
sourcing the package to the environment). It will launch all necessary nodes: nodes from Robonomics wrapper 
and main integration for myBuddy 280:
    ```shell
     ros2 launch mybuddy280_robonomics mybuddy280_robonomics_launch.py
    ```

4. You will see logs in the console with IPFS ID, path to directory with IPFS files and Robonomics address. The main node 
starts publish datalogs with a states of every joints every 1 min. You can check the datalog transactions, using, 
for example, [Subscan](https://robonomics.subscan.io/) explorer (just enter your Robonomics address). 

5. The node also starts waiting for every launch command, that will be sent to specified address. You can test it using 
the [Robonomics parachain portal](https://polkadot.js.org/apps/?rpc=wss%3A%2F%2Fkusama.rpc.robonomics.network%2F#/extrinsics):
go to **Developers** → **Extrinsics** → **Submission** and find launch function. 

6. The joints are controlled using `/myBuddy280/send_joint_angles` service, so you need to prepare corresponding message
for its request, that will go as a launch parameter. For convenience, this message need to be added as JSON-file to IPFS:
   ```json
      {
       "part_id": "R/L/W",
       "joint_number": [
           1,
           ⋮
           6,
       ],
       "angle": [
           0.0,
           ⋮
           0.0
       ],
       "speed": [
           100,
           ⋮
           100
       ]
   }
   ```

7. Then, you will need to upload the JSON file to IPFS and get its CID in special format (32-byte hash). For sending 
launch on the Robonomics parachain portal just specified the robot address and formatted string as the param. Submit 
transaction and watch for the action.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion or correction, please feel free to participate! You should:

1. Fork the project
2. Add and commit your changes
3. Open a pull request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the Apache-2.0 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Ivan Berman — [@berman_ivan](https://twitter.com/berman_ivan) — berman@robonomics.network

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Best-README-Template](https://github.com/othneildrew/Best-README-Template/)
* myBuddy 280 and its software are created and owned by [Elephant Robotics](https://www.elephantrobotics.com/en/mybuddy-280-pi-en/).
* GitBook about myBuddy and its software is available [here](https://docs.elephantrobotics.com/docs/gitbook-en/17-myBuddy/).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Fingerling42/mybuddy280-ros2-robonomics.svg?style=for-the-badge
[contributors-url]: https://github.com/Fingerling42/mybuddy280-ros2-robonomics/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Fingerling42/mybuddy280-ros2-robonomics.svg?style=for-the-badge
[forks-url]: https://github.com/Fingerling42/mybuddy280-ros2-robonomics/network/members
[stars-shield]: https://img.shields.io/github/stars/Fingerling42/mybuddy280-ros2-robonomics.svg?style=for-the-badge
[stars-url]: https://github.com/Fingerling42/mybuddy280-ros2-robonomics/stargazers
[issues-shield]: https://img.shields.io/github/issues/Fingerling42/mybuddy280-ros2-robonomics.svg?style=for-the-badge
[issues-url]: https://github.com/Fingerling42/mybuddy280-ros2-robonomics/issues
[license-shield]: https://img.shields.io/github/license/Fingerling42/mybuddy280-ros2-robonomics.svg?style=for-the-badge
[license-url]: https://github.com/Fingerling42/mybuddy280-ros2-robonomics/blob/master/LICENSE.txt