#include "astar.h"

double astar::small_cost = 1;
double astar::diag_cost = sqrt(2 * small_cost * small_cost);

astar::astar(Graph *g)
{
  this->graph = g;
  this->path.clear();
}

astar::astar(Graph g, Node& start, Node& goal, string name)
{
  this->graph = &g;
  this->path.clear();
  search(start.getID(), goal.getID(), name);
}

bool astar::search(int source, int target, string name)
{
  start = new _VNode(graph->getNode(source));
  goal  = new _VNode(graph->getNode(target));
  open.push(start);
  //int count = 0;
  while (!open.empty())
  {
    _VNode* current = open.top(); open.pop(); // Get and remove the top of the open list
    if(current->id == goal->id) // Found the path
    {
      // cout << "Source " << source << " Target " << target << " Start " << start->id << " Goal " << goal->id << " Current " << current->id << endl;
      construct_path(start, current);
      return true;
    }
    closed.push_back(current);

    // Add successor nodes of current to the open list
    vector<int> neighbors = graph->getNode(current->id).getNeighbors();
    // cout << current->id << " " << current->x << " " << current->y << " " << current->g << " " << current->f << " " << current->prev.size() << " " << neighbors.size() << endl;
    for (uint i = 0; i < neighbors.size(); i++)
    {
      _VNode* tmp = new _VNode(graph->getNode(neighbors[i]));
      //double tmpCost = graph->getNode(current->id).getCostTo(tmp->id);

      tmp->g = current->g + graph->getNode(current->id).getCostTo(tmp->id);
      if (name != "skeleton" or name != "hallwayskel")
      {
        tmp->f = tmp->g + euclidian_h(tmp, goal); // Compute f for this node
      }
      else
      {
        tmp->f = tmp->g + 0;
      }
      tmp->prev.push_back(current);
      // cout << tmp->id << " " << tmp->x << " " << tmp->y << " " << tmp->g << " " << tmp->f << " " << tmp->prev.size() << endl;

      bool inClosed = false;
      for(uint j = 0; j < closed.size(); j++)
      {
        if(closed[j]->id == tmp->id)
        {
          inClosed = true;
          if(closed[j]->g > tmp->g)
          {
            //printf("closed[j].g = %f, tmp.g = %f\n", closed[j]->g, tmp->g);
            closed[j] = tmp;
          }
          else if(closed[j]->g == tmp->g)
          {
            //printf("closed[j].g = %f, tmp.g = %f\n", closed[j]->g, tmp->g);
            bool inClosedPrev = false;
            for(int i = 0; i < closed[j]->prev.size(); i++)
            {
              if(closed[j]->prev[i]->id == current->id)
              {
                inClosedPrev = true;
              }
            }
            if(inClosedPrev == false)
            {
              closed[j]->prev.push_back(current);
            }
            //cout << "prev.size = " << closed[j]->prev.size() << endl;
          }
          break;
        }
      }
      if(inClosed || !tmp->accessible)
        continue;

      push_update(open, tmp);
    }
    //count++;
  }
  //cout << "Number of nodes expanded = " << count << endl;
  return false;
}


double astar::euclidian_h(_VNode* a, _VNode* b)
{
  // sqrt(a.x-b.x^2 + a.y-b.y^2)
  return sqrt( (a->x - b->x) * (a->x - b->x) +
               (a->y - b->y) * (a->y - b->y) );
}

double astar::octile_h(_VNode* a, _VNode* b)
{
  double dx, dy;
  dx = abs(a->x - b->x);
  dy = abs(a->y - b->y);
  return min(dx,dy) * diag_cost + (max(dx,dy) - min(dx, dy)) * small_cost;
}

void astar::push_update(priority_queue<_VNode*, vector<_VNode*>, _Compare> &pq, _VNode* n)
{
  list<_VNode*> l; // Hold all node pointers temporarily
  bool found = false;

  while(!pq.empty())
  {
    _VNode* tmp = pq.top();
    pq.pop();
    // cout << tmp->id << " " << n->id << endl;
    if(tmp->id == n->id)
    {
      found = true;
      if(tmp->g < n->g)
      {
        pq.push(tmp);
      }
      else if(tmp->g == n->g)
      {
        //cout << "open->id = " << tmp->id << ", n->id = " << n->id << endl;
        //printf("open->g = %f, n->g = %f\n", tmp->g, n->g);
        for(int i = 0; i < n->prev.size(); i++)
        {
          bool intmpPrev = false;
          for(int j = 0; j < tmp->prev.size(); j++)
          {
            if(tmp->prev[j]->id == n->prev[i]->id)
            {
              intmpPrev = true;
            }
          }
          if(intmpPrev == false)
          {
            tmp->prev.push_back(n->prev[i]);
          }
        }
        //cout << "prev.size = " << tmp->prev.size() << endl;
        pq.push(tmp);
      }
      else
      {
        pq.push(n);
      }
      break;
    }
    l.push_back(tmp);
  }
  // cout << "pq.size() " << pq.size() << " l.size() " << l.size() << endl;

  while(!l.empty())
  {
    pq.push(l.front());
    l.pop_front();
  }
  if(!found)
    pq.push(n);

  return;
}



void astar::construct_path(_VNode* s, _VNode* g)
{
  cout << "Inside construct_path" << endl;
  _VNode* tmp = g;
  path.clear();
  paths.clear();
  /*while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    path.push_front(tmp->prev[0]->id);
    tmp = tmp->prev[0];
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      path.push_front(tmp->prev[1]->id);
      tmp = tmp->prev[1];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();
  while(!tmp->prev.empty())
  {
    //cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      srand(time(NULL));
      int random_number = rand() % (tmp->prev.size());
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
  }
  //cout << "Path size = " << path.size() << endl;
  paths.push_back(path);

  tmp = g;
  path.clear();*/
  path.push_front(tmp->id);
  // cout << "Path size = " << path.size() << endl;
  // cout << "tmp->prev.empty() = " << tmp->prev.empty() << " tmp->id " << tmp->id << " s->id " << s->id << endl;
  int count = 0;
  while(!tmp->prev.empty() and count < 1000)
  {
    // cout << "tmp->prev.size() = " << tmp->prev.size() << endl;
    if(tmp->prev.size()>1)
    {
      // srand(time(NULL));
      // int random_number = rand() % (tmp->prev.size());
      int random_number = 1;
      path.push_front(tmp->prev[random_number]->id);
      tmp = tmp->prev[random_number];
    }
    else
    {
      path.push_front(tmp->prev[0]->id);
      tmp = tmp->prev[0];
    }
    // cout << "Path size = " << path.size() << " tmp->id " << tmp->id << endl;
    if(tmp->id == s->id){
      break;
    }
    count = count + 1;
  }
  // cout << "Path size = " << path.size() << endl;
  paths.push_back(path);
  //cout << "Number of paths = " << paths.size() << endl;
}

/* Returns true if left hand side has a lower f value than right hand side */
bool astar::_Compare::operator() (_VNode* lhs, _VNode* rhs)
{
  // In case of tie breakers best case is a higher g
  /*
  if(lhs->f == rhs->f)
  {
    if(lhs->g > rhs->g)
      return true;
    else
      return false;
  }


  return lhs->f > rhs->f;
  */
  // Different operator ordering
  if(lhs->f > rhs->f)
    return true;
  if(lhs->f < rhs->f)
    return false;
  if(lhs->g > rhs->g)
    return true;
  return false;

}
