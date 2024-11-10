# TossingBotStudien

# starten der Simulation:

1. Schritt

```bash
cd panda_ws
```

2. Schritt

```bash
source devel/setup.bash
```

3. Schritt

```bash
roslaunch panda_gazebo start_simulation.launch
```

Code:

```bash
panda_ws/src/moveit_tutorials/doc/move_group_python_interface/scripts
```

## Überblick

### Gazebo

Gazebo ist ein Open-Source-Simulationswerkzeug, das hauptsächlich zur Modellierung und Simulation von Robotersystemen in realistischen 3D-Umgebungen verwendet wird. Es bietet Entwicklern die Möglichkeit, Robotikprojekte und autonome Systeme in einer sicheren, virtuellen Umgebung zu testen, bevor sie auf physische Roboter oder in die reale Welt übertragen werden.

Integration mit ROS (Robot Operating System): Gazebo lässt sich eng mit dem Robot Operating System (ROS) integrieren, was es Entwicklern erleichtert, reale Robotersteuerungen und -algorithmen direkt in der Simulationsumgebung zu verwenden und zu testen.

- https://gazebosim.org/home

### ROS

Das Robot Operating System (ROS) ist eine Open-Source-Softwareplattform, die speziell für die Entwicklung von Roboteranwendungen konzipiert wurde. Bei einem Roboter müssen viele Komponenten reibungslos miteinander zusammen-arbeiten. Dazu zählen beispielsweise Motoren, Sensoren, Batterien und die verwendete Software. Das Robot Operating System (ROS) als kostenfreie Open Source Lösung stellt eine Vielfalt an Softwarebibliotheken und Werkzeugen zur Verfügung. Mit seiner Verwendung können Roboteranwendungen einfach erstellt werden. Von Treibern bis hin zu hochmodernen Algorithmen und leistungsstarken Entwicklertools bietet ROS alles, was für ein Robotikprojekt benötigt wird.

- https://www.ros.org/
- https://docs.ros.org/en/rolling/

### RViz (ROS Visualization)

RViz (kurz für ROS Visualization) ist ein Visualisierungstool für das Robot Operating System (ROS), das speziell entwickelt wurde, um Roboterentwicklern zu helfen, Sensordaten, Robotermodellbewegungen und Umgebungsinformationen grafisch darzustellen. Es bietet eine visuelle Plattform, auf der verschiedene Datenquellen in ROS in Echtzeit angezeigt und analysiert werden können, was es besonders hilfreich macht, um Roboterentwicklungen zu überwachen, zu debuggen und zu steuern.

- https://wiki.ros.org/rviz

### MoveIt

MoveIt ist eine leistungsfähige Open-Source-Bibliothek und -Software für Bewegungsplanung und -steuerung von Robotern im Robot Operating System (ROS). Es bietet Roboterentwicklern und Forschern Werkzeuge zur Bewegungsplanung, Kollisionserkennung, Manipulation, Kinematik und Interaktion, um Roboterarme, mobile Roboter und hybride Systeme einfach und flexibel zu steuern.

MoveIt ist ideal für die Steuerung von Roboterarmen in der Industrieautomation, um komplexe Aufgaben wie das Aufnehmen und Platzieren von Objekten, das Montieren von Komponenten oder das Sortieren von Waren auszuführen.

- https://moveit.ai/
- https://github.com/moveit/moveit_tutorials

## Getting Started

Im folgenden wird beschrieben, wie man alle notwendige Software installiert und konfiguriert, sodass man mit der Entwicklung beginnen kann:

### Installation benötigter Software

1. Installation von ROS Noetic

- [Hier]:(https://www.ros.org/blog/getting-started/) findet man die außführliche Getting Started Dokumentation.
- [Hier]:(https://wiki.ros.org/noetic/Installation/Debian) die vollständige Installationsanleitung für Debian

2. Packages updaten

```bash
rosdep update
sudo apt update
sudo apt dist-upgrade
```

3. Installation catkin für ROS build System

```bash
sudo apt install ros-noetic-catkin python3-catkin-tools
```

4. Installation wstool

```bash
sudo apt install python3-wstool
```

### Einrichten eines Moveit Workspaces

# Getting Started

https://github.com/moveit/moveit_tutorials/blob/master/doc/getting_started/getting_started.rst

This tutorial will install MoveIt and create a workspace sandbox to run the tutorials and example robot.

Install ROS and Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`\_.
It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly.

Once you have ROS installed, make sure you have the most up to date packages: ::

```bash
rosdep update
sudo apt update
sudo apt dist-upgrade
```

Install `catkin <http://wiki.ros.org/catkin>`\_ the ROS build system: ::

```bash
sudo apt install ros-noetic-catkin python3-catkin-tools
```

Install `wstool <http://wiki.ros.org/wstool>`\_ : ::

```bash
sudo apt install python3-wstool
´´´

Create a Catkin Workspace

^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to have a `catkin <http://wiki.ros.org/catkin>`\_ workspace setup: ::

´´´bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
```

Download MoveIt Source
^^^^^^^^^^^^^^^^^^^^^^
Because the tutorials are actively developed in sync with MoveIt's master, you will most likely need to build all of MoveIt from source.
To this end, follow the instructions below. However, building MoveIt from source takes roughly an hour, so you might want to skip this step for now and try using the binary Debian packages first.
Come back to this step, if building of your workspace fails due to unknown symbols! ::

```bash
wstool init .
wstool merge -t . https://raw.githubusercontent.com/moveit/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials # this is cloned in the next section
wstool update -t .
```

Download Example Code
^^^^^^^^^^^^^^^^^^^^^

To easily follow along with these tutorials, you will need a **ROBOT_moveit_config** package. The default demo robot is the Panda arm from Franka Emika. To get a working **panda_moveit_config** package, we recommend you install from source.

Panda Gazebo: https://rickstaa.dev/panda-gazebo/get_started/install.html

Within your `catkin <http://wiki.ros.org/catkin>`\_ workspace, download the tutorials as well as the `panda_moveit_config` package. You may safely ignore any :code:`git clone` errors saying the destination already exists: ::

```bash
cd ~/ws_moveit/src
git clone https://github.com/moveit/moveit_tutorials.git -b master
git clone https://github.com/moveit/panda_moveit_config.git -b noetic-devel
```

.. note:: For now we will use a pre-generated `panda_moveit_config` package but later we will learn how to make our own in the `MoveIt Setup Assistant tutorial <../setup_assistant/setup_assistant_tutorial.html>`\_.

Build your Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

```bash
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

**Note** In case an upstream package is not (yet) available from the standard ROS repositories or if you experience any build errors in those packages, please try to fetch the latest release candidates from the `ROS testing repositories <http://wiki.ros.org/TestingRepository>`\_ instead: ::

        sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt update

The next command will configure your catkin workspace: ::

```bash
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Source the catkin workspace: ::

```bash
source ~/ws_moveit/devel/setup.bash
```

Optional: add the previous command to your `.bashrc`: ::

```bash
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```

.. note:: Sourcing the `setup.bash` automatically in your `~/.bashrc` is
not required and often skipped by advanced users who use more than one
catkin workspace at a time, but we recommend it for simplicity.

Next Step
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Visualize a robot with the interactive motion planning plugin for RViz <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`\_

### move Group Python Interface

https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/move_group_python_interface_tutorial.rst

# Move Group Python Interface

.. image:: move_group_python_interface.png
:width: 700px

One of the simplest MoveIt user interfaces is through the Python-based Move Group Interface. These wrappers
provide functionality for most operations that the average user will likely need,
specifically setting joint or pose goals, creating motion plans, moving the
robot, adding objects into the environment and attaching/detaching objects from
the robot.

Watch this quick `YouTube video demo <https://youtu.be/3MA5ebXPLsc>`\_ to see the power of the Move Group Python interface!

## Getting Started

If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`\_.

## Start RViz and MoveGroup node

Open two shells. Start RViz and wait for everything to finish loading in the first shell: ::

roslaunch panda_moveit_config demo.launch

Now run the Python code directly in the other shell using `rosrun`: ::

rosrun moveit_tutorials move_group_python_interface_tutorial.py

## Expected Output

In RViz, we should be able to see the following:

Press _<enter>_ in the shell terminal where you ran the `rosrun` command in between each step
#. The robot plans and moves its arm to the joint goal.
#. The robot plans a path to a pose goal.
#. The robot plans a Cartesian path.
#. The robot displays the Cartesian path plan again.
#. The robot executes the Cartesian path plan.
#. A box appears at the location of the Panda end effector.
#. The box changes colors to indicate that it is now attached.
#. The robot plans and executes a Cartesian path with the box attached.
#. The box changes colors again to indicate that it is now detached.
#. The box disappears.

## The Entire Code

Note: the entire code can be seen :codedir:`here in the tutorials GitHub repository<move_group_python_interface/scripts/move_group_python_interface_tutorial.py>`.

.. tutorial-formatter:: ./scripts/move_group_python_interface_tutorial.py

## The Launch File

The entire launch file is :codedir:`here<move_group_python_interface/launch/move_group_python_interface_tutorial.launch>`
on GitHub. All the code in this tutorial can be run from the
`moveit_tutorials` package that you have as part of your MoveIt setup.
