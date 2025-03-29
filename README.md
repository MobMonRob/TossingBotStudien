# 🤖 TossingBot-Studien

Willkommen beim TossingBot2.0-Projekt! Diese Repository enthält Code und Anleitungen für die Implementierung von Wurfbewegungen sowohl mit einem realen UR5e Roboterarm als auch in der Gazebo-Simulation.

## 📂 Projektstruktur

Das Projekt ist in zwei Hauptteile gegliedert:

### 🦾 Roboter

Im Verzeichnis `TossingBot2.0/Roboter` finden Sie verschiedene Python-Skripte zur Steuerung eines echten UR5e Roboterarms für Wurfbewegungen. Diese sind in verschiedene Varianten unterteilt:

- **Wurf Unterarm Zeitgesteuert**
  - `wurf_unterarm_durchschwingen.py`: Vollständige Unterwurfbewegung
  - `wurf_unterarm_impuls.py`: Kurzimpuls-Unterwurfbewegung

- **Wurf Seite Zeitgesteuert**
  - `wurf.py`: Seitliche Wurfbewegung mit zeitgesteuertem Ansatz

- **Wurf Seite Positionsgesteuert**
  - `wurf.py`: Seitliche Wurfbewegung mit positionsgesteuertem Ansatz

- **Wurf Überkopf Zeitgesteuert**
  - `wurf_ueberkopf.py`: Überkopfwurfbewegung

Alle diese Skripte können mit einem OnRobot RG2 oder RG6 Greifer verwendet werden.

### 🖥️ Simulation

Im Verzeichnis `TossingBot2.0/Simulation` finden Sie eine detaillierte Anleitung zur Einrichtung und Verwendung der UR5e-Robotersimulation in Gazebo mit MoveIt!. Die Anleitung umfasst:

1. Installation von ROS Noetic und erforderlichen Paketen
2. Einrichtung eines Catkin-Workspaces
3. Konfiguration von Gazebo und MoveIt!
4. Erstellung eines eigenen Python-Steuerungsskripts

## 🚀 Erste Schritte

### Für den realen Roboter:

1. Stellen Sie sicher, dass alle Vorbedingungen erfüllt sind:
   - ROS ist installiert und eingerichtet
   - Der UR5e Roboterarm ist verbunden und über ROS ansprechbar
   - Python 3 mit den erforderlichen Abhängigkeiten (rospy, numpy, actionlib)
   - Bei Verwendung eines Greifers: Ein OnRobot RG2/RG6 Greifer ist angeschlossen

2. Führen Sie eines der Wurfskripte aus, zum Beispiel:
   ```bash
   python3 wurf_unterarm_durchschwingen.py --gripper rg2 --ip IP_ADRESSE --port PORT
   ```

### Für die Simulation:

1. Folgen Sie der Installationsanleitung in der Simulation-README.md
2. Starten Sie die Simulation mit:
   ```bash
   roslaunch ur_gazebo ur5e_bringup.launch
   ```
3. Starten Sie MoveIt! mit:
   ```bash
   roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
   ```
4. Führen Sie das Steuerungsskript aus:
   ```bash
   rosrun ur5e_control move_robot.py
   ```

## 📖 Weiterführende Dokumentation

- Detaillierte Informationen zur Robotersteuerung finden Sie in der README.md im `Roboter`-Verzeichnis
- Ausführliche Anweisungen zur Simulation finden Sie in der README.md im `Simulation`-Verzeichnis

## ⚠️ Hinweise

- Bei allen Wurfskripten können Parameter wie Geschwindigkeiten oder Positionen bei Bedarf angepasst werden
- Die Simulation wurde mit ROS Noetic auf Ubuntu 20.04 getestet

## 🤝 Beitragen

Solltet ihr auf diesem Repo aufbauen in eurer Studienarbeit, hoffen wir dass alles gut genug dokumentiert ist. Ggf. fragt nach ob ihr Zugang zu der Studienarbeit von uns bekommen könnt, dort sich Dinge wie der "Versuchsaufbau" am Roboter usw. beschrieben.

P.S.: Die Simulation kostet einen viele Nerven, am Roboter selbst macht es dann aber Spaß ;)

---

Erstellt im Rahmen der TossingBot2.0-Studienarbeit 🎯
