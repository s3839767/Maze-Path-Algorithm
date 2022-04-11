#include "PathSolver.h"
#include <iostream>

PathSolver::PathSolver(){
    this->nodesExplored = new NodeList();
}

PathSolver::~PathSolver(){
}

// an algorithm that searches a path from the startNode to the goalNode 
void PathSolver::forwardSearch(Env env){

    // creates two nodeLists used for adding nodes (openList and closedList)
    NodeList* openNodeList = new NodeList();
    nodesExplored = new NodeList();

    Node* startNode = 0;
    Node* goalNode = 0;
    
    // nested for loop to find both the start and goal nodes in the env file
    // once found, it creates two node objects and 
    //      adds the startNode to both nodeLists
    for (int row = 0; row < ENV_DIM; row++){
        
        for (int col = 0; col < ENV_DIM; col++){
            
            if (SYMBOL_START == (env[row][col])){ 
                startNode = new Node(row, col, 0);
                openNodeList->addElement(startNode);
                nodesExplored->addElement(startNode);
            }
            else if (SYMBOL_GOAL == (env[row][col])){
                goalNode = new Node(row, col, 0);
        
            }   
                    
        }
    }

    // variable nodeCount implemented to call the highest node in nodeList
    int nodeCount = 0;
    Node* p = 0;
    Node* q = 0;
    
    // a while loop that implements the robot to move from node to node
    //      in the maze until it reaches the goalNode
    while ((openNodeList->getNode(nodeCount)->equals(goalNode)) == false){
        
        p = openNodeList->getNode(nodeCount);

        // a for loop that runs through the openList used to compare 
        //      each node's estimatedDist2Goal when finding the 
        //      shortest path to take.
        for (int i = 0; i < openNodeList->getLength(); i++){
    
            
            // ensures that the node does not exist in the closedList
            //      before running and also gets the next node with the
            //      smallest estimatedDist2Goal
            if(((checkNodeExists(p, nodesExplored) == true))
             || (p->getEstimatedDist2Goal(goalNode) 
             >= openNodeList->getNode(i)->getEstimatedDist2Goal(goalNode) 
             && (checkNodeExists(openNodeList->getNode(i), nodesExplored) 
             == false))) {    
                 
                p = openNodeList->getNode(i);

            }  
            
        }
        
        // an if condition that runs only when theres a movable spot 
        //      or a goalNode SOUTH of the robot
        if((SYMBOL_EMPTY == (env[p->getRow() + 1][p->getCol()])) 
                || (SYMBOL_GOAL == (env[p->getRow() + 1][p->getCol()]))){
            q = new Node(p->getRow() + 1,
                         p->getCol(),
                         p->getDistanceTraveled() + 1);

            // adds the new node to the openList if it does not
            //     exist in the closedList             
            if (checkNodeExists(q, openNodeList) == false){
                openNodeList->addElement(q);
                nodeCount++;
                if (q->equals(goalNode) == true){
                    nodesExplored->addElement(p);
                    nodesExplored->addElement(q);
                }
            }
        }
        // an if condition that runs only when theres a movable spot 
        //      or a goalNode NORTH of the robot
        if((SYMBOL_EMPTY == (env[p->getRow() - 1][p->getCol()])) 
                || (SYMBOL_GOAL == (env[p->getRow() - 1][p->getCol()]))){
            q = new Node(p->getRow() - 1,
                         p->getCol(),
                         p->getDistanceTraveled() + 1);

            // adds the new node to the openList if it does not
            //     exist in the closedList
            if (checkNodeExists(q, openNodeList) == false){
                openNodeList->addElement(q);
                nodeCount++;
                if (q->equals(goalNode) == true){
                    nodesExplored->addElement(p);
                    nodesExplored->addElement(q);
                }
            }
            
        }
        // an if condition that runs only when theres a movable spot 
        //      or a goalNode EAST of the robot
        if((SYMBOL_EMPTY == (env[p->getRow()][p->getCol() + 1])) 
                || (SYMBOL_GOAL == (env[p->getRow()][p->getCol() + 1]))){
            q = new Node(p->getRow(),
                         p->getCol() + 1,
                         p->getDistanceTraveled() + 1);

            // adds the new node to the openList if it does not
            //     exist in the closedList             
            if (checkNodeExists(q, openNodeList) == false){
                openNodeList->addElement(q);
                nodeCount++;
                if (q->equals(goalNode) == true){
                    nodesExplored->addElement(p);
                    nodesExplored->addElement(q);
                }
            }
            
        }
        // an if condition that runs only when theres a movable spot 
        //      or a goalNode WEST of the robot
        if((SYMBOL_EMPTY == (env[p->getRow()][p->getCol() - 1])) 
                || (SYMBOL_GOAL == (env[p->getRow()][p->getCol() - 1]))){
            q = new Node(p->getRow(),
                         p->getCol() - 1,
                         p->getDistanceTraveled() + 1);

            // adds the new node to the openList if it does not
            //     exist in the closedList             
            if (checkNodeExists(q, openNodeList) == false){
                openNodeList->addElement(q);
                nodeCount++;
                if (q->equals(goalNode) == true){
                    nodesExplored->addElement(p);
                    nodesExplored->addElement(q);
                }
            }
        }

        // adds the node to closeList if not already in it 
        if ((checkNodeExists(p, nodesExplored) == false) 
            && (nodesExplored->getNode(nodesExplored->getLength() - 1)
            ->equals(goalNode) == false))
        {  
            nodesExplored->addElement(p);
        }
           
    }
    
}

// a method that checks if a given node exists in a given nodeList
//   comparison of row number and column number between the two nodes
//      returns a boolean value based on if they node exists or not  
bool PathSolver::checkNodeExists(Node* node1, NodeList* nodeList){
    bool nodeExists = false;
    for (int i = 0; i < nodeList->getLength(); i++){
        if (node1->getRow() == nodeList->getNode(i)->getRow()){
            if (node1->getCol() == nodeList->getNode(i)->getCol()){
                nodeExists = true;
            }
        }    
    }
    return nodeExists;
}

NodeList* PathSolver::getNodesExplored(){
    // NodeList* copyNodesExploredList = new NodeList(*nodesExplored);
    // return copyNodesExploredList;
    return nodesExplored;
}
// a similar algorithm that instead returns a deep copy of the path
//      the robot should travel through the use of backtracking 
// (goalNode to startNode)  
NodeList* PathSolver::getPath(Env env){

    NodeList* copyNodesExplored = new NodeList(*nodesExplored);
    // initialises the selectedNode to the last node in 
    //  the deep copy of nodesExplored  
    Node* selectedNode = copyNodesExplored
                    ->getNode(copyNodesExplored->getLength() - 1);

    Node* proximityNode  = 0;
    NodeList* reversedPath = new NodeList();
    reversedPath->addElement(selectedNode);

    // a while loop that implements the robot to move from node to node
    //      in the maze until it reaches the startNode
    while(selectedNode->equals(copyNodesExplored->getNode(0)) == false){
        
        // a for loop used to help implement the adding of nodes
        //  only with a distanceTraveled of - 1 from the current node's
        //  distance travelled  
        for (int i = 0; i < copyNodesExplored->getLength(); i++){

            proximityNode = copyNodesExplored->getNode(i);

            // an if condition that runs only when theres a movable spot 
            // SOUTH of the robot  
            if(proximityNode->getRow() == selectedNode->getRow() + 1 
                && proximityNode->getCol() == selectedNode->getCol()) {
                
                if ((copyNodesExplored->getNode(i)->getDistanceTraveled())
                == (selectedNode->getDistanceTraveled() - 1)) 

                {
        
                    selectedNode = proximityNode;
                    reversedPath->addElement(selectedNode);
                    
                }
            }

            // an if condition that runs only when theres a movable spot 
            // NORTH of the robot
            if(proximityNode->getRow() == selectedNode->getRow() - 1 
                && proximityNode->getCol() == selectedNode->getCol()) {

                if ((copyNodesExplored->getNode(i)->getDistanceTraveled())
                == (selectedNode->getDistanceTraveled() - 1))

                {
                    
                    selectedNode = proximityNode;
                    reversedPath->addElement(proximityNode);
                    
                }
            }

            // an if condition that runs only when theres a movable spot 
            // EAST of the robot
            if(proximityNode->getRow() == selectedNode->getRow() 
            && proximityNode->getCol() == selectedNode->getCol() + 1) {

                if ((copyNodesExplored->getNode(i)->getDistanceTraveled())
                == (selectedNode->getDistanceTraveled() - 1))

                {
                    
                    selectedNode = proximityNode;
                    reversedPath->addElement(proximityNode);
                   
                }
            }

            // an if condition that runs only when theres a movable spot 
            // WEST of the robot
            if(proximityNode->getRow() == selectedNode->getRow() 
            && proximityNode->getCol() == selectedNode->getCol() - 1) {

                if ((copyNodesExplored->getNode(i)->getDistanceTraveled())
                == (selectedNode->getDistanceTraveled() - 1))

                {
                    
                    selectedNode = proximityNode;
                    reversedPath->addElement(proximityNode);
              
                }
            }
        }

    }

    // reverses the path for use in printing solution
    NodeList* path = new NodeList();
    for(int i = 0; i < reversedPath->getLength(); i++){
        path->addElement(reversedPath
            ->getNode(reversedPath->getLength() - 1 - i));
    }
    return path;
}

