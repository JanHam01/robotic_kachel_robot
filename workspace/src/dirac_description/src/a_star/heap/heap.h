#ifndef HEAP_H
#define HEAP_H


#include <vector>

#include "../node.h"

class Heap {
private:
  std::vector<Node *> list;
  int count = 0;

  void sortUp(Node *node);

  void sortDown(Node *node);

  void swap(Node *node1, Node *node2);

public:
  void add(Node *node);

  Node *removeFirst();

  void updateItem(Node *node);

  bool contains(Node *other) const;

  bool isEmpty() const;

  void clear();
};

#endif // HEAP_H
