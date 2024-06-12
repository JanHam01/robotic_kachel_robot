#ifndef NODE_H
#define NODE_H

#include "../util.h"

class Node {
private:
  int m_x;
  int m_y;
  int m_g;
  int m_h;
  int m_heapIndex;
  Node *m_parent;

public:
  Node(const Vector2 &pos, Node *parent) : m_x(pos.getX()),
                                           m_y(pos.getY()),
                                           m_g(0),
                                           m_h(0),
                                           m_heapIndex(0),
                                           m_parent(parent) {
  }

  bool operator==(const Node &other) const {
    return m_x == other.m_x && m_y == other.m_y;
  }

  int getX() const;

  int getY() const;

  int getG() const;

  int getF() const;

  int getHeapIndex() const;

  Node *getParent() const;

  void setX(int x);

  void setY(int y);

  void setG(int g);

  void setH(int h);

  void setHeapIndex(int index);

  void setParent(Node *parent);

  bool isSmaller(const Node *other) const;
};
#endif // NODE_H
