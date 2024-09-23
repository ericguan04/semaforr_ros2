/*!
 * SpatialModel.h
 *
 * \brief Represents the current spatial model of the environment: Regions, exits, conveyors, etc
 *
 * \author Anoop Aroor
 * \date 11/11/2016 Created
 */

#include <FORRRegionList.h>
#include <FORRTrails.h>
#include <FORRConveyors.h>
#include <FORRDoors.h>
#include <FORRHallways.h>
#include <FORRBarriers.h>

class SpatialModel{

public:
	SpatialModel(double width, double height, double granularity){
		abstract_map = new FORRRegionList();
		//trace = new FORRTrace();
		trails = new FORRTrails();
		conveyors = new FORRConveyors(width, height, granularity);
		doors = new FORRDoors();
		hallways = new FORRHallways(width, height);
		barriers = new FORRBarriers();
	};

	FORRRegionList* getRegionList(){return abstract_map;}
	//FORRTrace* getTrace(){return trace;}
	FORRTrails* getTrails(){return trails;}
	FORRConveyors* getConveyors(){return conveyors;}
	FORRDoors* getDoors(){return doors;}
	FORRHallways* getHallways(){return hallways;}
	FORRBarriers* getBarriers(){return barriers;}

private:
	FORRRegionList *abstract_map;
	//FORRTrace *trace;
	FORRTrails *trails;
	FORRConveyors *conveyors;
	FORRDoors *doors;
	FORRHallways *hallways;
	FORRBarriers *barriers;
};
