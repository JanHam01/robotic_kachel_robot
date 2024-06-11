//
// Created by robotik on 11.06.24.
//

#include <iostream>

#include "a_star.h"

void testAStar() {
    std::vector<Vector2> obstacles;
    obstacles.emplace_back(1,1);

    AStar aStar;

    for (Vector2 p : aStar.getPath(Vector2(0,0), Vector2(8,5), obstacles))
    {
        std::cout << "Path: " << p.getX() << " " << p.getY() << "\n";
    }
}