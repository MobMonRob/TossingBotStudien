# TossingBotStudien

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

# Getting Started

## Wichtige Links

- [Panda Gazebo Git Repo](https://github.com/rickstaa/panda-gazebo.git)
- [Panda Gazebo getting started](https://rickstaa.dev/panda-gazebo/get_started/install.html)
- [Moveit tutorials Git Repo](https://github.com/moveit/moveit_tutorials.git)
- [Move group python Interface tutorial](https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/move_group_python_interface_tutorial.rst)
- [Move group python Interface tutorial Video](https://youtu.be/3MA5ebXPLsc)
- [Catkin](http://wiki.ros.org/catkin)
- [wstool](http://wiki.ros.org/wstool)
- [ROS Getting started](https://www.ros.org/blog/getting-started/)

## Einrichten der Umgebung

```bash
sudo apt update
```

```bash
sudo apt install ros-noetic-desktop-full
```

```bash
rosdep update
```

```bash
sudo apt update
```

```bash
sudo apt install ros-noetic-catkin python3-catkin-tools
```

```bash
udo apt install python3-wstool
```

```bash
mkdir -p ~/ws_moveit/src
```

```bash
cd ws_moveit/src
```

```bash
ws_moveit/src$ wstool init .
```

```bash
ws_moveit/src$ wstool merge -t . https://raw.githubusercontent.com/moveit/moveit/master/moveit.rosinstall
```

```bash
/ws_moveit/src$ wstool remove moveit_tutorials
```

```bash
/ws_moveit/src$ wstool update -t .
```

```bash
/ws_moveit/src$ git clone https://github.com/moveit/moveit_tutorials.git -b master
```

```bash
/ws_moveit$ catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
/ws_moveit$ catkin build
```

```bash
/ws_moveit/src$ git clone --recurse-submodules https://github.com/rickstaa/panda-gazebo.git
```

```bash
/ws_moveit$ rosdep install --from-path src --ignore-src -r -y
```

```bash
/ws_moveit$ catkin build -j4 -DCMAKE_BUILD_TYPE=Release
```

```bash
/ws_moveit$ source ~/ws_moveit/devel/setup.bash
```

```bash
/ws_moveit$ roslaunch panda_gazebo start_simulation.launch
```

## Starten der Umgebung

```bash
/ws_moveit$ source ~/ws_moveit/devel/setup.bash
```

```bash
/ws_moveit$ roslaunch panda_gazebo start_simulation.launch
```

Der Code sollte in folgendem Pfad abgelegt werden:

```bash
ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts
```

## Simulationsumgebung konfigurieren

Um die Welt der Simulationsumgebung zu konfigurieren muss die folgende Datei angepasst werden:

- ws_moveit/src/panda-gazebo/panda_gazebo/resources/worlds/empty.world

Mit den folgenden Zeilen definieren wir die Plattform und den Block der geworfen werden soll:

```xml
<!--A platform-->
<include>
<uri>model://platform</uri>
<pose>0.4 0 0.209911 0 0 0</pose>
</include>

<!--A cube-->
<include>
<uri>model://cube</uri>
<pose>0.4 0 0.419088 0 0 0</pose>
</include>
```

Im folgenden ein komplettes Beispiel der world mit Plattform und Block:

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
<world name="empty">
<!--A global light source-->
<include>
<uri>model://sun</uri>
</include>
<!--A ground plane-->
<include>
<uri>model://ground_plane</uri>
</include>
<!--Hier werden die einzufügenden Objekte (Plattform, Würfel) beschrieben-->

<!--A platform-->
<include>
<uri>model://platform</uri>
<pose>0.4 0 0.209911 0 0 0</pose>
</include>
<!--A cube-->
<include>
<uri>model://cube</uri>
<pose>0.4 0 0.419088 0 0 0</pose>
</include>

<!--Camera settings-->
<gui fullscreen='0'>
<camera name='user_camera'>
<pose>1.59801 -1.66211 1.29545 -0 0.419643 2.23219</pose>
<view_controller>orbit</view_controller>
<projection_type>perspective</projection_type>
</camera>
</gui> <!--Load Panda joint fixer Gazebo world plugin-->
<plugin name="panda_joint_locker" filename="libpanda_gazebo.so"/>
</world>
</sdf>
```
