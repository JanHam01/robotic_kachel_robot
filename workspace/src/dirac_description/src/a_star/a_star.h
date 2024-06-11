//
// Created by robotik on 10.06.24.
//
#pragma once

#ifndef A_STAR_H
#define A_STAR_H

#include <unordered_set>
#include <vector>
#include "../util.h"
#include "heap/heap.h"

class AStar {
private:
    std::vector<Node*> allNodes;
    Heap openList;
    std::unordered_set<Node*> closedList;

    Vector2 directions[4] = {
        Vector2(1, 0),
        Vector2(0, 1),
        Vector2(-1, 0),
        Vector2(0, -1)
    };

    Node* getNode(const Vector2& pos, Node* parent);

    static int getDistanceToPoint(const Node *a, const Vector2& b);

    static std::vector<Vector2> pathToMotion(std::vector<Vector2>& path);

public:
    ~AStar();

    std::vector<Vector2> getPath(const Vector2& start, const Vector2& end, std::vector<Vector2>& obstacles);

};

#endif //A_STAR_H
