# Komplette Anleitung: UR5e-Roboter in Gazebo simulieren und mit Python steuern

Diese Anleitung führt dich Schritt für Schritt durch die Einrichtung eines UR5e-Roboters in Gazebo, der Verwendung von MoveIt! zur Steuerung des Roboters und dem Erstellen eines Python-Skripts zur Steuerung.

## 1. Installiere ROS Noetic und erforderliche Pakete

Wenn ROS Noetic noch nicht installiert ist, folge den folgenden Schritten:

### 1.1 Installiere ROS Noetic

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

### 1.2 Installiere die notwendigen ROS-Pakete

```bash
# Installiere ROS-Abhängigkeiten
sudo apt install python3-rosdep python3-catkin-tools
```

### 1.3 Initialisiere rosdep und aktualisiere:

```bash
sudo rosdep init
rosdep update
```

## 2. Erstelle einen Catkin-Workspace

Falls du noch keinen Workspace eingerichtet hast, erstelle nun einen:

```bash
# Gehe zum Home-Verzeichnis und erstelle einen Workspace
cd ~
mkdir -p ur5e_ws/src
cd ur5e_ws/src
catkin_init_workspace
```

## 3. Klone die notwendigen Repositories für den UR5e

### 3.1 Klone das Universal Robots GitHub-Repository

```bash
# Klone das Universal Robot Repository (für Gazebo und MoveIt! Unterstützung)
git clone https://github.com/ros-industrial/universal_robot.git
```

### 3.2 MoveIt!-Konfiguration für den UR5e-Roboter

Die MoveIt!-Konfiguration für den UR5e-Roboter ist bereits im universal_robot-Repository enthalten. Du musst diese Konfiguration nicht manuell kopieren oder verschieben, sondern sie befindet sich direkt im ur5e_moveit_config Ordner.

Die MoveIt!-Konfigurationsdateien sind unter folgendem Pfad zu finden:

```bash
~/ur5e_ws/src/universal_robot/ur5e_moveit_config
```

### 3.3 Weitere Repositories hinzufügen (optional)

Falls du zusätzliche Repositories benötigst, z. B. für Gazebo oder andere ROS-Pakete, kannst du diese jetzt hinzufügen.

## 4. Installiere alle Abhängigkeiten

Installiere die Abhängigkeiten für den Workspace:

```bash
cd ~/ur5e_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Baue den Catkin-Workspace

Baue deinen Workspace mit catkin_make:

```bash
cd ~/ur5e_ws
catkin_make
```

Lade anschließend die ROS-Umgebungsvariablen:

```bash
source devel/setup.bash
```

## 6. Starte Gazebo mit dem UR5e-Roboter

Starte nun Gazebo, um die Simulation des UR5e-Roboters zu starten:

```bash
roslaunch ur_gazebo ur5e_bringup.launch
```

Dies öffnet Gazebo und lädt das UR5e-Modell.

## 7. Starte MoveIt! für den UR5e-Roboter

Nun starte MoveIt!, um den Roboter zu steuern:

```bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

Dieser Befehl startet MoveIt! in Verbindung mit Gazebo, sodass du den Roboter über die MoveIt!-Planung steuern kannst.

## 8. Erstelle das ur5e_control-Paket und Python-Skript

Falls das Paket ur5e_control noch nicht existiert, kannst du es erstellen. Hier sind die Schritte:

### 8.1 Erstelle das Paket ur5e_control

Wechsle in das src-Verzeichnis deines Workspaces und erstelle das ur5e_control-Paket:

```bash
cd ~/ur5e_ws/src
catkin_create_pkg ur5e_control rospy moveit_commander std_msgs
```

- rospy: Wird benötigt, um ROS mit Python zu verwenden.
- moveit_commander: Die Python-Bibliothek zur Kommunikation mit MoveIt!.
- std_msgs: Standard-Meldungstypen in ROS.

### 8.2 Erstelle das Python-Skript move_robot.py

Erstelle einen Ordner scripts im ur5e_control-Paket, falls dieser nicht existiert:

```bash
mkdir ~/ur5e_ws/src/ur5e_control/scripts
```

Erstelle das Python-Skript move_robot.py im scripts-Ordner:

```bash
touch ~/ur5e_ws/src/ur5e_control/scripts/move_robot.py
```

Bearbeite das Skript move_robot.py:

```python
#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface

def main():
    # Initialisiere MoveIt! und ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot', anonymous=True)

    # Roboter-Planungsgruppe und Umgebung einrichten
    group_name = "manipulator"  # Die Planungsgruppe für den UR5e
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()

    # Hole aktuelle Position des Roboters
    current_joint_values = move_group.get_current_joint_values()
    rospy.loginfo("Aktuelle Gelenkwerte: %s", current_joint_values)

    # Setze Zielposition auf "home" (vordefiniert in der MoveIt!-Konfiguration)
    move_group.set_named_target("home")
    rospy.loginfo("Bewege Roboter zur Home-Position...")
    plan = move_group.go(wait=True)

    # Gib an, dass der Roboter zur Home-Position bewegt wurde
    if plan:
        rospy.loginfo("Roboter erfolgreich in Home-Position bewegt!")
    else:
        rospy.logwarn("Fehler beim Bewegen des Roboters zur Home-Position")

    # Jetzt definieren wir eine benutzerdefinierte Zielposition (Bewegung über Gelenkwerte)
    joint_goal = move_group.get_current_joint_values()

    # Setze neue Zielgelenkwerte (Beispielwerte, du kannst diese ändern)
    joint_goal[0] = -1.57  # Schulter-Pan-Gelenk
    joint_goal[1] = -1.57  # Schulter-Lift-Gelenk
    joint_goal[2] = 1.57   # Ellbogen-Gelenk
    joint_goal[3] = -1.57  # Handgelenk 1
    joint_goal[4] = 1.57   # Handgelenk 2
    joint_goal[5] = 0.0    # Handgelenk 3

    # Bewege den Roboter zu dieser neuen Zielposition
    move_group.set_joint_value_target(joint_goal)
    rospy.loginfo("Bewege Roboter zu benutzerdefinierten Zielwerten...")
    plan = move_group.go(wait=True)

    # Gib an, ob der Roboter erfolgreich zu der Zielposition bewegt wurde
    if plan:
        rospy.loginfo("Roboter erfolgreich zur Zielposition bewegt!")
    else:
        rospy.logwarn("Fehler beim Bewegen des Roboters zur Zielposition")

    # Beende MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
```

### 8.3 Mach das Skript ausführbar

Stelle sicher, dass das Skript ausführbar ist:

```bash
chmod +x ~/ur5e_ws/src/ur5e_control/scripts/move_robot.py
```

## 9. Baue den Workspace erneut

Wechsle in das Workspace-Verzeichnis und baue den Workspace erneut:

```bash
cd ~/ur5e_ws
catkin_make
```

Lade anschließend die ROS-Umgebungsvariablen:

```bash
source devel/setup.bash
```

## 10. Führe das Python-Skript aus

Nun solltest du das Python-Skript ausführen können, um den Roboter zu bewegen:

```bash
rosrun ur5e_control move_robot.py
```

---

## Starten der Simulation und des Skriptes

Hier sind die drei Hauptbefehle zusammengefasst, die du nacheinander in separaten Terminals ausführen musst:

```bash
roslaunch ur_gazebo ur5e_bringup.launch
```

```bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

```bash
rosrun ur5e_control move_robot.py
```
