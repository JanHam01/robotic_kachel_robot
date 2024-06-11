#include "a_star.h"

#include <algorithm>


AStar::~AStar() {
    for (Node* node : allNodes) {
        delete node;
    }
}

Node* AStar::getNode(const Vector2 &pos, Node* parent) {
    for (Node* node : allNodes) {
        if (node->getX() == pos.getX() && node->getY() == pos.getY()) {
            return node;
        }
    }
    auto node = new Node(pos, parent);
    allNodes.push_back(node);
    return node;
}

int AStar::getDistanceToPoint(const Node *a, const Vector2& b) {
    int dx = std::abs(a->getX() - b.getX());
    int dy = std::abs(a->getY() - b.getY());
    if (dx < dy) {
        return 14 * dy + 10 * (dx - dy);
    }
    return 14 * dx + 10 * (dy - dx);

}



std::vector<Vector2> AStar::getPath(const Vector2 &start, const Vector2 &end, std::vector<Vector2> &obstacles) {


    openList.add(getNode(start, nullptr));
    Node* current;
    while(!openList.isEmpty()) {
        current = openList.removeFirst();
        closedList.insert(current);

        for (Vector2& direction : directions) {
            auto neighbourPos = Vector2(current->getX() + direction.getX(), current->getY() + direction.getY());
            if (neighbourPos.getX() == end.getX() && neighbourPos.getY() == end.getY()) {
                goto PathFound;
            }

            if (std::find(obstacles.begin(), obstacles.end(), neighbourPos) != obstacles.end()) {
                continue;
            }

            auto neighbour = getNode(neighbourPos, current);


            if (closedList.find(neighbour) != closedList.end()){
                continue;
            }

            int moveCost = current->getG() + 1;

            bool notOpenContains = !openList.contains(neighbour);

            if (moveCost < neighbour->getG() || notOpenContains) {
                neighbour->setG(moveCost);
                neighbour->setH(getDistanceToPoint(neighbour, end));
                neighbour->setParent(current);

                if (notOpenContains) {
                    openList.add(neighbour);
                } else {
                    openList.updateItem(neighbour);
                }
            }
        }
    }

    PathFound:

    std::vector<Vector2> path;
    while (current != nullptr) {
        path.emplace_back(current->getX(), current->getY());
        current = current->getParent();
    }

    std::reverse(path.begin(), path.end());

    return path;
}


