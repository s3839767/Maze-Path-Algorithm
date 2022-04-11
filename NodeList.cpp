#include "NodeList.h"
#include <iostream>

NodeList::NodeList(){
    this->length = 0;
}

NodeList::~NodeList(){
    for (int i = 0; i < length; i++){
        delete nodes[i];   
    }
}

// Creates a deep copy of the NodeList copying the
//      length and nodes in the nodeList through
//      the use of a for loop copying each field (e.g. row, col and dist)
NodeList::NodeList(NodeList& other){
    this->length = other.length;
    
    for (int i = 0; i < length; i++){
        this->nodes[i] = new Node(other.getNode(i)->getRow(), 
                                  other.getNode(i)->getCol(), 
                                  other.getNode(i)->getDistanceTraveled());  
    }
}

int NodeList::getLength(){
    return length;
}

void NodeList::addElement(Node* newPos){
    nodes[length] = newPos;
    ++length; 
}

Node* NodeList::getNode(int i){
    return nodes[i];
}