# 🤖 UR5e-Roboter Wurfsimulator

Diese Simulationsumgebung ermöglicht die präzise Simulation von Wurfbewegungen eines UR5e-Roboterarms mit RG2-Greifer in Gazebo. Sie wurde für wissenschaftliche Untersuchungen von Roboterwurfbewegungen entwickelt und bietet eine realitätsnahe Modellierung der Roboterdynamik.

## 🛠️ Installation und Systemeinrichtung

Die Einrichtung der Simulationsumgebung erfordert mehrere Schritte, die systematisch durchgeführt werden müssen:

### 🐢 ROS-Umgebung

Als Grundlage dient ROS Noetic, das auf Ubuntu 20.04 installiert werden muss. Die Installation umfasst zusätzliche Pakete wie MoveIt! und die entsprechenden Kinematik-Plugins:

```bash
# Füge ROS Noetic Repositories hinzu
sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setze den ROS-Repository-Schlüssel
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 0xB01FA116

# Aktualisiere die Paketliste
sudo apt update

# Installiere die Desktop-Full-Version von ROS Noetic
sudo apt install ros-noetic-desktop-full
sudo apt-get install ros-noetic-moveit ros-noetic-moveit-kinematics
sudo apt-get install ros-noetic-trac-ik-kinematics-plugin
```

### 📂 Workspace-Erstellung

Ein Catkin-Workspace muss erstellt werden, der als Container für alle relevanten Pakete dient:

```bash
# Gehe zum Home-Verzeichnis und erstelle einen Workspace
cd ~
mkdir -p ur5e_ws/src
cd ur5e_ws/src
catkin_init_workspace
```

### 📦 Paketinstallation

Das `ur5_rg2_ign`-Paket, das im Verzeichnis `TossingBot2.0/Simulation/ur5_rg2_ign` enthalten ist, muss in den Workspace integriert werden. Dieses Paket enthält sämtliche Beschreibungsdateien, Steuerskripte und Konfigurationen für den UR5e mit RG2 Greifer.

### 🔄 Abhängigkeiten installieren

Nach dem Klonen des Repositories müssen alle Abhängigkeiten installiert werden:

```bash
cd ~/ur5e_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 🔨 Workspace kompilieren

Der Workspace wird mit `catkin_make` kompiliert und anschließend in die Umgebung geladen:

```bash
cd ~/ur5e_ws
catkin_make
source devel/setup.bash
```

## 🗂️ Komponentenübersicht und Dateifunktionen

Das Simulationspaket `ur5_rg2_ign` ist hierarchisch strukturiert und enthält mehrere wichtige Komponenten:

### 📝 URDF/SDF-Modellbeschreibungen

- **`urdf/ur5_rg2.urdf`**: Diese Datei enthält die vollständige kinematische und dynamische Beschreibung des UR5e-Roboters mit RG2-Greifer im URDF-Format (Unified Robot Description Format). Sie definiert die Gelenkverbindungen, visuellen Komponenten, Kollisionsgeometrien sowie Trägheitseigenschaften aller Roboterteile. Diese Datei ist fundamental für die korrekte physikalische Simulation des Roboters.
- **`ur5_rg2/model.sdf`**: SDF-Beschreibung (Simulation Description Format) des Roboters für die Ignition-Gazebo-Umgebung, die detaillierte dynamische Parameter wie Dämpfung und Reibung für realistische Bewegungssimulation enthält.

### 🎨 Mesh-Dateien

- **`ur5_rg2/meshes/visual/*.dae`**: COLLADA-Dateien für die visuelle Darstellung des Roboters mit detaillierten Texturen und Materialien.
- **`ur5_rg2/meshes/collision/*.stl`**: Vereinfachte STL-Geometrien für effiziente Kollisionserkennung, die den Rechenaufwand während der Simulation reduzieren.

### 🚀 Launch-Dateien

- **`launch/ur5_rg2_bringup.launch`**: Startet die grundlegende Robotersimulation ohne zusätzliche Steuerungskomponenten.
- **`launch/ur5_rg2_control.launch`**: Initialisiert die Gelenksteuerung für den Roboter mit entsprechenden Controllern.
- **`launch/ur5_rg2_moveit.launch`**: Startet MoveIt! für die Bewegungsplanung und -kontrolle des Roboters.
- **`launch/ur5_rg2_bringup_moveit.launch`**: Kombinierte Launch-Datei, die sowohl den Roboter als auch MoveIt! in einem Schritt startet.
- **`launch/ur5_rg2_gazebo.launch`**: Startet die vollständige Gazebo-Simulation mit dem UR5e-Roboter in einer anpassbaren Umgebung.
- **`launch/empty_world.world`**: Definiert die Simulationsumgebung mit physikalischen Eigenschaften wie Gravitation und Beleuchtung.

### 💻 Skripte für Robotersteuerung

- **`scripts/estimate_inertial_properties.py`**: Berechnet die Trägheitseigenschaften aller Roboterkomponenten basierend auf einer Gesamtmasse von 18,4 kg für den UR5e und 0,78 kg für den RG2 Greifer, was für eine realistische Simulation der Dynamik unerlässlich ist.
- **`scripts/test_gripper.py`**: Testet die Greiferfunktionalität durch definierte Öffnungs- und Schließbewegungen.
- **`scripts/pick_ball.py`**: Implementiert eine vollständige Pick-and-Throw-Sequenz, die den Roboter einen Ball greifen und mit einer definierten Trajektorie werfen lässt. Dieses Skript stellt den Kern der Wurfexperimente dar und enthält Parameter für verschiedene Wurfbewegungen:

```python
# Bewegung zur Greifposition
grasp_position = [0, -0.15, -2.08, -0.15, 1.5, 1.5]
controller.move_joints(grasp_position)

# Wurfbewegung ausführen
throw_motion_position = [-1.5, -0.6, 0, 0, 0, 0]
controller.move_joints(throw_motion_position, wait_time=0)

# Greifer während der Wurfbewegung öffnen
controller.open_gripper(wait_time=4)
```

### ⚙️ Konfigurationsdateien

- **`config/`**: Enthält YAML-Dateien für die Steuerung der Gelenkcontroller und MoveIt!-Konfigurationen, die Bewegungsparameter wie Beschleunigungsgrenzen und Geschwindigkeitslimits definieren.

## 🎮 Praktische Nutzung der Simulation

Um die Simulationsumgebung zu starten und Wurfexperimente durchzuführen, sind folgende Schritte erforderlich:

### 1️⃣ Simulation starten

In einem Terminal wird die Gazebo-Simulation mit dem UR5e-Roboter gestartet:

```bash
roslaunch ur5_rg2_ign ur5_rg2_gazebo.launch
```

Diese Launch-Datei initialisiert die virtuelle Umgebung und platziert den Roboter in einer definierten Startposition.

### 2️⃣ MoveIt! starten

In einem zweiten Terminal wird die MoveIt!-Planungsumgebung gestartet:

```bash
roslaunch ur5_rg2_ign ur5_rg2_moveit.launch
```

Dadurch wird der Bewegungsplaner aktiviert, der kollisionsfreie Trajektorien für den Roboter berechnet.

### 3️⃣ Wurfskript ausführen

Nachdem die Simulation läuft, kann in einem dritten Terminal das Wurfskript gestartet werden:

```bash
rosrun ur5_rg2_ign pick_ball.py
```

Dieses Skript führt die definierte Wurfsequenz aus, bei der der Roboter einen Ball greift und mit der programmierten Bewegung wirft.

### 4️⃣ Parameter anpassen

Für wissenschaftliche Untersuchungen können verschiedene Parameter im Wurfskript angepasst werden:

- 📐 Gelenkwinkel und Positionen für unterschiedliche Wurfbahnen
- ⏱️ Timing und Geschwindigkeit der Bewegungen
- 👐 Öffnungszeitpunkt des Greifers für optimale Freigabe des Objekts
- 📈 Beschleunigungsprofile für verschiedene Wurftechniken
