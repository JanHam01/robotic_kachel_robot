#include "node.h"

int Node::getX() const {
  return m_x;
}

int Node::getY() const {
  return m_y;
}

int Node::getG() const {
  return m_g;
}


int Node::getF() const {
  return m_g + m_h;
}

int Node::getHeapIndex() const {
  return m_heapIndex;
}

Node *Node::getParent() const {
  return m_parent;
}

void Node::setX(const int x) {
  m_x = x;
}

void Node::setY(const int y) {
  m_y = y;
}

void Node::setG(const int g) {
  m_g = g;
}

void Node::setH(const int h) {
  m_h = h;
}

void Node::setHeapIndex(const int index) {
  m_heapIndex = index;
}

void Node::setParent(Node *parent) {
  m_parent = parent;
}

bool Node::isSmaller(const Node *other) const {
  if (getF() == other->getF()) {
    return m_h < other->m_h;
  }
  return getF() < other->getF();
}
