#include "Graph.h"

Graph::Graph(Map * m, int p): map(m) {

  // convert proximity into cms
  this->proximity = p;
  // If p is given, but 0
  
  if ( p == 0 )
    this->proximity = 20;

  int l = map->getLength();
  int h = map->getHeight();
  cout << l << " " << h << " " << proximity << endl;
  // create nodes
  for( int x = 0; x < l; x++ ){
    vector<int> column; 
    for( int y = 0; y < h; y++ ){
      column.push_back(-1);
    }
    nodeIndex.push_back(column);
  }
  cout << "Node index columns : " << nodeIndex.size() << endl;
  maxInd = -1;
  generateNavGraph();
}

Graph::Graph(int p, int l, int h){

  // convert proximity into cms
  this->proximity = p;
  // If p is given, but 0
  
  if ( p == 0 )
    this->proximity = 20;
  this->length = l;
  this->height = h;
  cout << l << " " << h << " " << proximity << endl;
  // create nodes
  for( int x = 0; x < l; x++ ){
    vector<int> column; 
    for( int y = 0; y < h; y++ ){
      column.push_back(-1);
    }
    nodeIndex.push_back(column);
  }
  cout << "Node index columns : " << nodeIndex.size() << endl;
  maxInd = -1;
}

void Graph::resetGraph(){
  nodes.clear();
  edges.clear();
  for(int x = 0; x < nodeIndex.size(); x++){
    for(int y = 0; y < nodeIndex[x].size(); y++){
      nodeIndex[x][y] = -1;
    }
  }
  cout << "Graph reset complete" << endl;
}

Graph::~Graph() {
  /*
  for ( int i = 0; i < edges.size(); i++ )
    delete edges.at(i); 
  
  vector<Node*>::iterator n_itr; 
  for ( n_itr = nodes.begin(); n_itr != nodes.end(); n_itr++ )
    delete (*n_itr); 
  */
}

/*
 * return a node with the given index or return one with invalid_index if not found.
 */
Node Graph::getNode(int n){
  return (*(nodes.at(n))); 
}

Node* Graph::getNodePtr(int n){
  return nodes.at(n); 
}

/*
 * return an edge with the given indexes of nodes or return one with invalid indexes if not found
 */
Edge* Graph::getEdge(int n1, int n2) {
  vector<Edge*>::iterator eiter;
  Node nd1 = getNode(n1);
  Node nd2 = getNode(n2);
  if (! (nd1.getID() == Node::invalid_node_index || nd2.getID() == Node::invalid_node_index )){
    for( eiter = edges.begin(); eiter != edges.end(); eiter++ ){
      if ( n1 == (*eiter)->getFrom() && n2 == (*eiter)->getTo() ) 
        return (*eiter); 
      if ( n1 == (*eiter)->getTo() && n2 == (*eiter)->getFrom() ) 
        return (*eiter);
    }
  }
  Edge * e0 = new Edge(Node::invalid_node_index, Node::invalid_node_index);
  return e0;
}

// returns true if the node n is in the Graph::nodes. 
// used by PathPlanner to check if the source/target is a valid node in Graph
bool Graph::isNode(Node n) {
  vector<Node*>::iterator iter; 
  for( iter = nodes.begin(); iter != nodes.end(); iter++ )
    if ( (*(*iter)) == n ) 
      return true; 
  return false;
}

// returns true if the edge exists in the Node::nodeEdges of "From" node
bool Graph::isEdge(Edge e) {
  vector<Edge*> edges = getNode(e.getFrom()).getNodeEdges();
  vector<Edge*>::iterator iter;
  for ( iter = edges.begin() ; iter != edges.end() ; iter++ ) {
    if ( e == (*(*iter)) ){
      // cout << "Edge matches existing" << endl;
      return true;
    }
    Edge en((*iter)->getTo(), (*iter)->getFrom(), (*iter)->getCost(true), (*iter)->getCost(false));
    if ( en == e ){
      // cout << "Edge matches reverse edge" << endl;
      return true;
    }
  }
  return false;
}

void Graph::generateNavGraph() {
  int l = map->getLength();
  int h = map->getHeight();
  cout << l << " " << h << endl;
  // create nodes
  int index = 0;
  for( int x = 0; x < l; x += proximity ){
    for( int y = 0; y < h; y += proximity ){
      bool inBuf = false;
      if ( map->isPointInBuffer(x,y) )
        inBuf = true;
      Node * n = new Node(index, x, y, 0, inBuf, map->getDistanceClosestWall(x,y));
      nodes.push_back(n);
      //cout << x << ":" << y << ":" << index << endl;
      nodeIndex[x][y] = index;
      index++;
    }
  }
  maxInd = index-1;

  cout << "Completed creating nodes" << endl;
  // update node neighbors
  populateNodeNeighbors(true);

  cout << "Completed populating neighbors" << endl;
  // create edges
  
  vector<Node*>::iterator iter;
  for( iter = nodes.begin(); iter != nodes.end(); iter++ ){

    vector<int> nbrs = getNeighbors(*(*iter));
    vector<int>::iterator it;
    for( it = nbrs.begin(); it != nbrs.end(); it++ ){
      Edge * e = new Edge( (*iter)->getID(), getNode(*it).getID() );
      //cout << "for each edge "<< endl;
      if ( !isEdge((*e)) ) {
	int x1,y1,x2,y2;
	x1 = (*iter)->getX();
	y1 = (*iter)->getY();
	x2 = getNode(*it).getX();
	y2 = getNode(*it).getY();
	double distCost = Map::distance(x1,y1,x2,y2);
	int multiplier = 1;
	//std::cout << "Updating edge cost" << std::endl;
	
	if ( map->isPathObstructed(x1,y1,x2,y2)){
	 	multiplier += 20;
	}
	if ( map->isPointInBuffer( x1, y1+(proximity*1.25) ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x1, y1-(proximity*1.25) ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x1+(proximity*1.25), y1 ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x1-(proximity*1.25), y1 ) ){
	 	multiplier += 2;
	}
	/*
	if ( map->isPointInBuffer( x1+(proximity*1.5), y1+(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x1-(proximity*1.5), y1-(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x1+(proximity*1.5), y1-(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x1-(proximity*1.5), y1+(proximity*1.5) ) ){
	 	multiplier += 5;
	}

	if ( map->isPointInBuffer( x2+(proximity*1.5), y2+(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x2-(proximity*1.5), y2-(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x2+(proximity*1.5), y2-(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	if ( map->isPointInBuffer( x2-(proximity*1.5), y2+(proximity*1.5) ) ){
	 	multiplier += 5;
	}
	*/

	if ( map->isPointInBuffer( x2, y2+(proximity*1.25) ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x2, y2-(proximity*1.25) ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x2+(proximity*1.25), y2 ) ){
	 	multiplier += 2;
	}
	if ( map->isPointInBuffer( x2-(proximity*1.25), y2 ) ){
	 	multiplier += 2;
	}
	
	e->setDistCost(multiplier * distCost);

	edges.push_back(e);
	(*iter)->addNodeEdge(e);
	nodes.at(*it)->addNodeEdge(e);
	//cout << "Edge from :" << (*iter)->getX() << " " << (*iter)->getY() << "->" << getNode(*it).getX() << " " << getNode(*it).getY() << " cost : " << e->getCost(0) << endl;	
      }
    }
    //cout << " Neighbor edges size: " << (*iter)->getNodeEdges().size() << endl;
  }
  //cout << "completed generating graph" << endl;
}

bool Graph::addNode(int x, int y, double r, int ind){
  bool node_added = false;
  if(nodeIndex[x][y] == -1){
    nodeIndex[x][y] = ind;
    Node * n = new Node(ind, x, y, r, false, 0);
    nodes.push_back(n);
    // cout << "Added Node " << ind << " x " << x << " y " << y << endl;
    node_added = true;
    if(ind > maxInd){
      maxInd = ind;
    }
  }
  // else{
  //   cout << "Node already exists, try nearby" << endl;
  //   if(nodeIndex[x+1][y] == -1){
  //     nodeIndex[x+1][y] = ind;
  //     Node * n = new Node(ind, x+1, y, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x+1 << " y " << y << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x+1][y+1] == -1){
  //     nodeIndex[x+1][y+1] = ind;
  //     Node * n = new Node(ind, x+1, y+1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x+1 << " y " << y+1 << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x][y+1] == -1){
  //     nodeIndex[x][y+1] = ind;
  //     Node * n = new Node(ind, x, y+1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x << " y " << y+1 << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x-1][y] == -1){
  //     nodeIndex[x-1][y] = ind;
  //     Node * n = new Node(ind, x-1, y, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x-1 << " y " << y << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x-1][y-1] == -1){
  //     nodeIndex[x-1][y-1] = ind;
  //     Node * n = new Node(ind, x-1, y-1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x-1 << " y " << y-1 << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x][y-1] == -1){
  //     nodeIndex[x][y-1] = ind;
  //     Node * n = new Node(ind, x, y-1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x << " y " << y-1 << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x+1][y-1] == -1){
  //     nodeIndex[x+1][y-1] = ind;
  //     Node * n = new Node(ind, x+1, y-1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x+1 << " y " << y-1 << endl;
  //     node_added = true;
  //   }
  //   else if(nodeIndex[x-1][y+1] == -1){
  //     nodeIndex[x-1][y+1] = ind;
  //     Node * n = new Node(ind, x-1, y+1, false);
  //     nodes.push_back(n);
  //     cout << "Added Node " << ind << " x " << x-1 << " y " << y+1 << endl;
  //     node_added = true;
  //   }
  //   else{
  //     cout << "No suitable nearby node" << endl;
  //   }
  // }
  return node_added;
}

void Graph::addEdge(int ind1, int ind2, double distance, vector<CartesianPoint> path){
  Edge * e = new Edge(ind1, ind2);
  //cout << "for each edge "<< endl;
  if(isEdge((*e))){
    // cout << "Existing edge found" << endl;
    vector<Edge*> edges_from = getNode((*e).getFrom()).getNodeEdges();
    vector<Edge*>::iterator iter;
    for ( iter = edges_from.begin() ; iter != edges_from.end() ; iter++ ) {
      Edge en((*iter)->getTo(), (*iter)->getFrom(), (*iter)->getCost(true), (*iter)->getCost(false));
      if((*e) == (*(*iter)) or en == (*e)){
        // cout << "Matching edge found for from node " << distance << " " << (*iter)->getDistCost() << endl;
        if(distance < (*iter)->getDistCost()){
          (*iter)->setDistCost(distance);
          (*iter)->setEdgePath(path);
        }
        break;
      }
    }
    vector<Edge*> edges_to = getNode((*e).getTo()).getNodeEdges();
    for ( iter = edges_to.begin() ; iter != edges_to.end() ; iter++ ) {
      Edge en((*iter)->getTo(), (*iter)->getFrom(), (*iter)->getCost(true), (*iter)->getCost(false));
      if((*e) == (*(*iter)) or en == (*e)){
        // cout << "Matching edge found for to node " << distance << " " << (*iter)->getDistCost() << endl;
        if(distance < (*iter)->getDistCost()){
          (*iter)->setDistCost(distance);
          (*iter)->setEdgePath(path);
        }
        break;
      }
    }
  }
  else{
    nodes[ind1]->addNeighbor(ind2);
    // cout << "Added neighbor to " << ind1 << " from " << ind2 << endl;
    nodes[ind2]->addNeighbor(ind1);
    // cout << "Added neighbor to " << ind2 << " from " << ind1 << endl;
    // int x1,y1,x2,y2;
    // x1 = nodes[ind1]->getX();
    // y1 = nodes[ind1]->getY();
    // x2 = nodes[ind2]->getX();
    // y2 = nodes[ind2]->getY();
    e->setDistCost(distance);
    e->setEdgePath(path);
    edges.push_back(e);
    nodes[ind1]->addNodeEdge(e);
    nodes[ind2]->addNodeEdge(e);
    // cout << "Edge from : " << x1 << " " << y1 << "->" << x2 << " " << y2 << " cost : " << e->getCost(0) << endl;
  }
}

void Graph::populateEdges(){
  vector<Node*>::iterator iter;
  for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
    vector<int> nbrs = getNeighbors(*(*iter));
    vector<int>::iterator it;
    for( it = nbrs.begin(); it != nbrs.end(); it++ ){
      int x1,y1,x2,y2;
      x1 = (*iter)->getX();
      y1 = (*iter)->getY();
      x2 = getNode(*it).getX();
      y2 = getNode(*it).getY();
      double distCost = Map::distance(x1,y1,x2,y2);
      this->addEdge((*iter)->getID(), getNode(*it).getID(), distCost, vector<CartesianPoint>());
    //   Edge * e = new Edge( (*iter)->getID(), getNode(*it).getID() );
    //   //cout << "for each edge "<< endl;
    //   if ( !isEdge((*e)) ) {
    //     int x1,y1,x2,y2;
    //     x1 = (*iter)->getX();
    //     y1 = (*iter)->getY();
    //     x2 = getNode(*it).getX();
    //     y2 = getNode(*it).getY();
    //     double distCost = Map::distance(x1,y1,x2,y2);
    //     e->setDistCost(distCost);
    //     edges.push_back(e);
    //     (*iter)->addNodeEdge(e);
    //     nodes.at(*it)->addNodeEdge(e);
    //     cout << "Edge from :" << (*iter)->getX() << " " << (*iter)->getY() << "->" << getNode(*it).getX() << " " << getNode(*it).getY() << " cost : " << e->getCost(0) << endl; 
    //   }
    }
    //cout << " Neighbor edges size: " << (*iter)->getNodeEdges().size() << endl;
  }
}

bool Graph::isConnected(){
  if(nodes.size() == 0){
    return false;
  }
  set<int> neighbor_nodes;
  neighbor_nodes.insert(nodes[0]->getID());
  // cout << "Neighbor nodes " << neighbor_nodes.size() << " Graph Nodes " << numNodes() << " Added " << nodes[0]->getID() << endl;
  vector<int> nbrs = getNeighbors((*nodes[0]));
  while(nbrs.size()>0){
    int node_id = nbrs[0];
    neighbor_nodes.insert(node_id);
    // cout << "Neighbor nodes " << neighbor_nodes.size() << " Added " << node_id << endl;
    vector<int> new_nbrs = getNeighbors(getNode(node_id));
    for(int i = 0; i < new_nbrs.size(); i++){
      if(find(nbrs.begin(), nbrs.end(), new_nbrs[i]) != nbrs.end() or neighbor_nodes.find(new_nbrs[i]) != neighbor_nodes.end()){
        continue;
      }
      else{
        nbrs.push_back(new_nbrs[i]);
      }
    }
    nbrs.erase(nbrs.begin());
    // cout << "Neighbors to add " << nbrs.size() << endl;
  }
  // cout << "Neighbor nodes " << neighbor_nodes.size() << " Graph Nodes " << numNodes() << endl;
  if(numNodes() == neighbor_nodes.size()){
    return true;
  }
  else{
    return false;
  }
}

/* Used to pick the nodes in a circular area to disable edges */
vector<Node*> Graph::getNodesInRegion( int x, int y, double dist ) {
  vector<Node*> nodesInRegion;
  vector<Node*>::iterator iter;
  for(iter = nodes.begin(); iter != nodes.end(); iter++ ){
    int iterx = (*iter)->getX(); 
    int itery = (*iter)->getY();
    if( iterx - dist <= x )
      if( iterx + dist >= x )
	if( itery - dist <= y )
	  if( itery + dist >= y )
	    if ( Map::distance( x, y, iterx, itery) <= dist ) {
	      nodesInRegion.push_back(*iter);
	    }
  }
  return nodesInRegion;
}


// returns nodes within a square grid
vector<Node*> Graph::getNodesInRegion( int x1, int y1, int x2, int y2 ){
  vector<Node*> nodesInRegion;
  vector<Node*>::iterator iter;
  for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
    int it_x = (*iter)->getX(); 
    int it_y = (*iter)->getY();
    if ( it_x >= x1 && it_x <= x2 && it_y >= y2 && it_y <= y1 )
      nodesInRegion.push_back(*iter);
  }
  return nodesInRegion; 
}

void Graph::clearGraph() {
  vector<Node*>::iterator itr; 
  for(itr = nodes.begin(); itr != nodes.end(); itr++) {
    (*itr)->setAccessible(true);
  }
}

vector<int> Graph::getNeighbors(Node n){
    return n.getNeighbors(); 
}


void Graph::populateNodeNeighbors(bool withmap){
  vector<Node*>::iterator iter;
  if(withmap){
    for ( iter = nodes.begin(); iter != nodes.end(); iter++ ) {
      //cout << "Given node : " << (*iter)->getX() << " " << (*iter)->getY() << endl;  
      for( int x = (*iter)->getX()-proximity; x <= (*iter)->getX()+proximity; x += proximity ){
        for( int y = (*iter)->getY()-proximity; y <= (*iter)->getY()+proximity; y += proximity ){ 
          //cout << "--nearby nodes : " << x << " " << y << endl;
          if( ! ( x == (*iter)->getX() && y == (*iter)->getY() ) ){
            //cout << "--not given node" << endl;
            if ( map->isWithinBorders(x, y) &&  !map->isPathObstructed((*iter)->getX(), (*iter)->getY(), x, y)){
              (*iter)->addNeighbor(getNodeID(x,y));
            }
          }
        }
      }
    }
  }
  else{
    for(iter = nodes.begin(); iter != nodes.end(); iter++){
      vector<Node*> nearby_nodes = getNodesInRegion((*iter)->getX(), (*iter)->getY(), proximity);
      vector<Node*>::iterator iternn;
      for(iternn = nearby_nodes.begin(); iternn != nearby_nodes.end(); iternn++) {
        // cout << "Adding neighbor to " << (*iter)->getX() << " " << (*iter)->getY() << " from " << (*iternn)->getX() << " " << (*iternn)->getY() << endl;
        (*iter)->addNeighbor((*iternn)->getID());
        (*iternn)->addNeighbor((*iter)->getID());
      }
    }
  }
}

int Graph::getNodeID(int x, int y) {
  return nodeIndex[x][y]; 
}

double Graph::calcCost(Node n1, Node n2){
  return n1.getCostTo(n2.getID()); 
} 

void Graph::printGraph() {
  cout << "Printing graph..." << endl ;
  vector<Edge*>::iterator iter; 
  for( iter = edges.begin(); iter != edges.end(); iter++ )
    (*iter)->printEdge();
  cout << "Total number of nodes: " << numNodes() << ", number of edges: " << numEdges() << endl;
}

// output graph in menge format
void Graph::outputGraph() {
  std::ofstream myfile;
  myfile.open("graph.txt");

  myfile << numNodes() << endl;
  vector<Node*>::iterator iter; 
  double degree = 0;
  for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
    cout << (*iter)->getX() << " " << (*iter)->getX()/100.0f << endl;
    myfile << (*iter)->getNeighbors().size() << " " << (*iter)->getX()/100.0f << " " << (*iter)->getY()/100.0f << endl;
    if((*iter)->getNeighbors().size() != (*iter)->getNodeEdges().size()){
    	cout << (*iter)->getNeighbors().size() << " : " << (*iter)->getNodeEdges().size() << endl; 
    }
    degree += (*iter)->getNeighbors().size();
  }
  
  myfile << numEdges() << endl;
  vector<Edge*>::iterator iter1; 
  for( iter1 = edges.begin(); iter1 != edges.end(); iter1++ )
  	myfile << (*iter1)->getFrom() << " " << (*iter1)->getTo() << endl;
  
  cout << "Degree : " << degree << endl;
  myfile.close();
}

