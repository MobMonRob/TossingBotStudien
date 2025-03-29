# 🤖 UR5e Roboterarm Wurfskripte

Diese Repository enthält verschiedene Python-Skripte zur Steuerung eines UR5e Roboterarms für Wurfbewegungen in unterschiedlichen Ausführungen. Hier finden Sie Anleitungen zur Verwendung der verschiedenen Skripte.

## 📋 Vorbedingungen

Für alle Skripte gelten folgende Vorbedingungen:

- 🔄 ROS (Robot Operating System) muss installiert und eingerichtet sein
- 🤖 Ein UR5e Roboterarm muss verbunden und über ROS ansprechbar sein
- 🐍 Python 3 mit den erforderlichen Abhängigkeiten (rospy, numpy, actionlib)
- 🔌 Bei Verwendung eines Greifers: Ein OnRobot RG2/RG6 Greifer muss angeschlossen sein
- Das Python Skript: `onrobot.py` wird verwendet um mit dem Greifer zu kommunizieren. Dieses muss im Order des Skriptes liegen, welches man ausführen will.

## 🚀 Wurfskripte

### 🏋️ Wurf Unterarm Zeitgesteuert

**Dateien:** 
- `UR5e_Scripts/wurf_unterarm_zeitgesteuert/wurf_unterarm_durchschwingen.py`
- `UR5e_Scripts/wurf_unterarm_zeitgesteuert/wurf_unterarm_impuls.py`

Diese Skripte steuern einen UR5e Roboterarm, um eine Unterwurfbewegung auszuführen. Die Bewegung ist zeitgesteuert, was bedeutet, dass die Bewegungsgeschwindigkeiten nach Zeitintervallen definiert sind.

**Verwendung:**
```bash
python3 wurf_unterarm_durchschwingen.py --gripper rg2 --ip IP_ADRESSE --port PORT
```
oder
```bash
python3 wurf_unterarm_impuls.py --gripper rg2 --ip IP_ADRESSE --port PORT
```

Der Parameter `durchschwingen` führt eine komplette Wurfbewegung durch, während `impuls` einen kurzen Wurfimpuls verwendet.

### 🔄 Wurf Seite Zeitgesteuert

**Datei:** `UR5e_Scripts/wurf_seite_zeitgesteuert/wurf.py`

Dieses Skript steuert einen seitlichen Wurf mit dem UR5e Roboterarm, wobei die Bewegung zeitgesteuert erfolgt. Der Arm vollführt eine seitliche Wurfbewegung, ähnlich einem Diskuswurf.

**Verwendung:**
```bash
python3 wurf.py --gripper rg2 --ip IP_ADRESSE --port PORT
```

### 📍 Wurf Seite Positionsgesteuert

**Datei:** `UR5e_Scripts/wurf_seite_positionsgesteuert/wurf.py`

Im Gegensatz zur zeitgesteuerten Version verwendet dieses Skript einen positionsgesteuerten Ansatz für den seitlichen Wurf. Anstatt Zeitintervalle zu definieren, werden präzise Positionen für den Roboterarm vorgegeben.

**Verwendung:**
```bash
python3 wurf.py --gripper rg2 --ip IP_ADRESSE --port PORT
```

### 🏆 Wurf Überkopf Zeitgesteuert

**Datei:** `UR5e_Scripts/wurf_ueberkopf_zeitgesteuert/wurf_ueberkopf.py`

Dieses Skript implementiert einen Überkopfwurf mit dem UR5e Roboterarm. Der Roboter führt eine Wurfbewegung über dem Kopf aus, ähnlich einem Basketball-Wurf.

**Verwendung:**
```bash
python3 wurf_ueberkopf.py --gripper rg2 --ip IP_ADRESSE --port PORT
```

## 🔧 OnRobot Greifer-Steuerung

Alle Skripte können mit dem OnRobot RG2 oder RG6 Greifer verwendet werden. Die Steuerung des Greifers erfolgt über die Datei `onrobot.py`, die in jedem Skriptordner enthalten ist.

Der Greifer wird über die folgenden Parameter gesteuert:
- `--gripper`: Art des Greifers ('rg2' oder 'rg6')
- `--ip`: IP-Adresse des Greifers
- `--port`: Port für die Kommunikation mit dem Greifer (default: 502)

## ⚠️ Hinweise

- Bei Bedarf können Parameter in den Skripten angepasst werden, wie Geschwindigkeiten oder Positionen. 