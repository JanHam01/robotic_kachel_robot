# DIrected Robot Avoiding hindranCe --> DIRAC
# Projekt für die Robotik Vorlesung an der DHBW Mannheim/ TINF22IT1.
Idee:
Der Roboter DIRAC fährt auf einem 10x10 Grid zu einem festgelegten Ziel.
Dabei muss er Hindernisse umfahren. 
Um diese zu erkennen, besitzt der vierrädrige Kastenrobot DIRAC einen Lidar-Sensor.
-[ ] Zusatz: Kann Route als Liste zurückgeben

In der Umsetzung wurden die Basic Skills des Roboters implementiert (nach vorne fahren, links/rechts drehen, umdrehen). Der Lidar Sensor erkennt ebenfalls vorne, links und rechts. 
Der Code besteht dabei aus einem Package, bestehend aus verschiedenen Methoden. Zudem ist eine Gazebo-Welt mit verschiedenen Hindernissen miteingebaut.
