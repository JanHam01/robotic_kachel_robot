# Directed Robot Avoiding Hindrance
# Projekt für die Robotik Vorlesung an der DHBW Mannheim/ TINF22IT1.
## Von Jan Hampel, Kim Nissen, Katharina Pröll und Simon Weiss

Unser Roboter DIRAC (Directed Robot Avoiding Hindrance) befindet sich in einem 10x10 Grid, in dem er zu einem definierten Ziel fahren muss.  
Dabei muss er Hindernisse auf dem Grid umfahren und dennoch den kürzesten Weg finden.  

# Umsetzung
DIRAC ist ein Kastenroboter, ausgestattet mit einem Lidar-Sensor, um Hindernisse zu erkennen. Hindernisse werden so vorne, links und rechts erkannt.  
Auf seinen vier Rädern bewegt er sich durch die Umgebung. Er kann sich dabei nach vorne bewegen, nach links oder rechts drehen und umdrehen.  
Während DIRAC fährt, lässt sich im Terminal verfolgen, ob er sein Ziel bereits erreicht hat.   

# Eingaben
Nach dem Clonen des Package sind folgende Eingaben nötig (in Ordner "workspace"):  
- source ./devel/setup.bash
- roslaunch dirac_descrpition simulation.launch x_pos:=*hier Koordinaten für x*  y_pos:=*hier Koordinaten für y*  
Die Koordinaten für die x- und y- Position können frei gewählt werden, zB x_pos:=6.0, y_pos:=1.0.  
Der Befehl startet dann die Simulation in gazebo und DIRAC macht sich auf dem schnellsten Weg auf zu seinem Ziel.

# Code-Umsetzung
Um den kürzesten Weg zu finden wurde der A*- Algorithmus implementiert. Die Umsetzung in C++ findet sich in /workspace/src/dirac_descrption/src/astar .  
Die Steuerung des Roboters wurde in C++ implementiert, der Code befindet sich in /workspace/src/dirac_descrption/src.   
Das .srdf file der Welt mitsamt der Hindernisse liegt in /workspace/src/dirac_descprtion/worlds.  
Der Lidar-Sensor wurde in /workspace/src/dirac_descrption/src/lidar_listener.cpp ebenfalls in C++ implementiert.  
Weiter befinden sich in workspace/src/dirac_description die .launch-, .rviz-, .gazebo- und .urdf- files in den entsprechenden Ordnern, um DIRAC in seiner Welt in gazebo starten und fahren zu lassen.  


