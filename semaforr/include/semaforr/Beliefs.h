/*!
 * Beliefs.h
 *
 * \brief Contains spatial representations of the world (e.g. Map, Graph),
 *        the agent's curren state (an AgentState), the agent's knowledge
 *        about other agents and whatever else may be useful.
 *
 * \author Anoop Aroor 
 *
 */

#ifndef BELIEFS_H
#define BELIEFS_H

#include "AgentState.h"
#include "SpatialModel.h"

#include <time.h>
#include <list>
#include <map>
#include <string>
#include <set>
#include <fstream>
#include <sstream>
#include <vector>
# include <cmath>
# include <math.h>
# include <iostream>
#include <rclcpp/rclcpp.hpp>

using std::vector;
using std::set;


class Beliefs
{
public:
    /*! \brief Belief constructor
     *
     * \param m A line segment map
     */

    Beliefs(double width, double height, double granularity, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax){
        agentState = new AgentState(arrMove, arrRotate, moveArrMax, rotateArrMax);
        spatialModel = new SpatialModel(width, height, granularity);
    }
 
    AgentState* getAgentState(){
	//std::cout << "return agent state " << std::endl;
	return agentState;
    }
    SpatialModel* getSpatialModel(){return spatialModel;}

        
private:
    
    /** \brief The agent's current position, its task status, decision state, and immediate sensor values*/
    AgentState *agentState;

    /*! \brief Manages the environment model learned by the agent */ 
    SpatialModel *spatialModel;
};

#endif

