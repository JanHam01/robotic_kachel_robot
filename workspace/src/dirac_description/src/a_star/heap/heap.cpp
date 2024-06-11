
#include "heap.h"

#include <algorithm>

void Heap::add(Node *node) {
    node->setHeapIndex(count);
    list.push_back(node);
    sortUp(node);
    count++;
}

Node* Heap::removeFirst() {
    Node* first = list[0];
    count--;
    list[0] = list[count];
    list.pop_back();
    list[0]->setHeapIndex(0);
    sortDown(list[0]);
    return first;
}

void Heap::updateItem(Node *node) {
    sortUp(node);
}

void Heap::sortUp(Node *node) {
    int parentIndex = (node->getHeapIndex() - 1) / 2;
    while(true) {
        if (node->isSmaller(list[parentIndex])) {
            swap(node, list[parentIndex]);
        } else {
            break;
        }
        parentIndex = (node->getHeapIndex() - 1) / 2;
    }
}

void Heap::sortDown(Node *node) {
    while(true) {
        int leftChildIndex = node->getHeapIndex() * 2 + 1;
        int rightChildIndex = node->getHeapIndex() * 2 + 2;
        if (leftChildIndex < count) {
            int swapIndex = leftChildIndex;
            if (rightChildIndex < count) {
                if (list[rightChildIndex]->isSmaller(list[leftChildIndex])) {
                    swapIndex = rightChildIndex;
                }
            }

           if (list[swapIndex]->isSmaller(node)) {
                swap(list[swapIndex], node);
            } else {
                return;
            }
        }
        else {
            return;
        }
    }
}

void Heap::swap(Node *node1, Node *node2) {
    const int tmp = node1->getHeapIndex();
    node1->setHeapIndex(node2->getHeapIndex());
    node2->setHeapIndex(tmp);
    list[node1->getHeapIndex()] = node1;
    list[node2->getHeapIndex()] = node2;
}

bool Heap::contains(Node* other) const {
    return std::find(list.begin(), list.end(), other) != list.end();
}

bool Heap::isEmpty() const {
    return count == 0;
}

void Heap::clear() {
    list.clear();
    count = 0;
}
