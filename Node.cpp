#include "Node.h"
#include <iostream>


Node::Node(int row, int col, int dist_traveled)
{
    this->row = row;
    this->col = col;
    this->dist_traveled = dist_traveled;
}

Node::~Node(){
}

int Node::getRow(){
    return row;
}

int Node::getCol(){
    return col;
}

int Node::getDistanceTraveled(){
    return dist_traveled;
}

void Node::setDistanceTraveled(int dist_traveled)
{
    this->dist_traveled = dist_traveled;
}

// calculates the estimated distance to the goalNode for each node
// Estimated distance = distance_travelled of node p 
//                      + Manhattan distance from p to G    
int Node::getEstimatedDist2Goal(Node* goal){
    int manhattDistance = abs(this->col - goal->getCol())
                       + abs(this->row - goal->getRow());

    int estimDistance = manhattDistance + this->dist_traveled;

    return estimDistance;
}

// checks if two nodes are equal
// comparison of row number and column number which
//      returns a boolean value based on if they are equal or not     
 bool Node::equals(Node* node) {
    bool areEqual = false;
    bool rowEqual = this->row == node->getRow();
    bool colEqual = this->col == node->getCol();
    if (rowEqual && colEqual) {
        areEqual = true;
    }
    return areEqual;
}
    
//--------------------------------                             