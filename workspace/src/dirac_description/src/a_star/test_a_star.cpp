//
// Created by robotik on 11.06.24.
//

/* Obstacle 2 0
Obstacle 0 2
Obstacle 0 -1
Current pos: ( 0.499786 , 0.499717 )
End pos: ( 5 , 5 )
0.499786
Path 1 0
Path 0 1
Path 4 0
Path 0 3
make 0
END OF DRIVING
Current pos: ( 1.20768 , 0.499832 )
Desired pos: ( 1.5 , 0.5 )

Obstacle 2 0
Obstacle 0 2
Obstacle 0 -1
Obstacle 1 5
Obstacle 1 -1
Current pos: ( 1.25225 , 0.499632 )
End pos: ( 5 , 5 )
1.25225
Path 1 0
Path 0 1
Path 4 0
Path 0 3
Path 1 0
Path 0 1
make 0 */

#include <iostream>

#include "a_star.h"

AStar aStar;

void testAStar() {
    std::vector<Vector2> obstacles;
    obstacles.emplace_back(2,0);
    obstacles.emplace_back(0,2);
    obstacles.emplace_back(0,-1);


    for (Vector2 p : aStar.getPath(Vector2(0, 0), Vector2(5,5), obstacles, true))
    {
        std::cout << "Path: " << p.getX() << " " << p.getY() << "\n";
    }
}



void testAStar2() {
    std::vector<Vector2> obstacles;
    obstacles.emplace_back(2,0);
    obstacles.emplace_back(0,2);
    obstacles.emplace_back(0,-1);



    for (Vector2 p : aStar.getPath(Vector2(1, 0), Vector2(5,5), obstacles, true))
    {
        std::cout << "Path: " << p.getX() << " " << p.getY() << "\n";
    }
}

int main() {
    testAStar();
    testAStar2();
    return 0;
}