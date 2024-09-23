#ifndef TIER3ADVISOR_H
#define TIER3ADVISOR_H
          
/*
 * Tier3Advisor.h
 * Created:Jun. 27, 2013
 * Last Modified: April 14, 2014
 *
 * brief Implementation of my idea of tier three advisors.
 * Note: I think this is important fact that I keep forgetting from time to time so I would like to keep it here:
 * advisors should implement their own check whether they are commenting or not. I already have advisor_active member variable
 * that I use when initializing them. This is global flag (for the entire run), while advisor_commenting flag will be set or 
 * reset depending on the environment state, only for active robots.
 *
 * author Slavisa Djukic <sdjukic@hunter.cuny.edu>
 *
 */

# include <string>
# include <vector>
# include <set>
# include <map>

# include "Beliefs.h"
# include "FORRAction.h"

class Tier3Advisor { 

 public:
  // All we need to initialize the advisor is:
  // possible actions it can take, its name, description and flag that indicates
  // whether advisor is active or not
  // NOTE: variable st indicates max step robot can take, not sure if this is 
  // important, check and correct.
  // Interesting thing is that compiler is making me initialize member values
  // in the same order they are declared
  // NOTE: how I am calling Advisor constructor in this call
  Tier3Advisor(Beliefs *beliefs, string name, string description, double weight, double *parameters, bool isActive = true); 

    // Dummy default constructor
    Tier3Advisor() {};

    // Need destructor too, has to be virtual, since individual advisors in the Controller.cpp are casted as generic
    // tier3 advisors so this destructor will be called on them, but they are created with their own constructors
    // which should be called when terminating them
    virtual ~Tier3Advisor();

    /*
     * Function that will produce comment for proposed action
     * It will be overloaded by individual advisor i.e. whether it is greedouy
     * or obstacleAvoider
     * This will work fine because target can be defined differently for different
     * advisors. Greedy will consider where target is and for obstacleAvoider
     * target will be wall
     */
    virtual double actionComment(FORRAction action) = 0;

    // This is factory method that will create concrete instance of Tier3Advisor
    static Tier3Advisor* makeAdvisor(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool isActive);

    // This function will return advices on all proposed actions
    std::map <FORRAction, double> allAdvice();

    // method that returns advisor's name; it is used to print it out in the log
    // to see which advisor gave what advice strenght to which action
    string get_name(){ return name;}
    
    // method that returns the flag of whether advisor is active or not
    bool is_active(){ return active; }
    // accessor to advisor_active variable
    void set_active(bool value) { active = value; }   

    double get_weight() { return weight;}  
    void set_weight(double weight) {weight = weight;} 
    
    // setter and getter methods for isCommenting flag (whether advisor should comment current action)
    bool is_commenting() { return advisor_commenting; }
    // every active advisor for the run will call this function before it is asked for comments
    // if this function sets advisor_commenting variable to false then this advisor will not
    // produce comments this iteration.
    virtual void set_commenting() = 0;
    void normalize(map <FORRAction, double> *);
    void rank(map <FORRAction, double> *);
    void standardize(map <FORRAction, double> *);
    
    // Cannot use private since I am subclassing this class and I need all these member variables
 protected:

    // this is array in which we store magic numbers for our advisors
    double auxiliary_constants[4];
    string name;
    string description;
    double weight;
    Beliefs *beliefs;
    /* some advisers will not be active all the time this flag is showing whether adviser is active or not
     * this flag should be checked in the RobotController so we do not waste function call on it and also confuse the
     * return value from such (unnecessary) call !!!
     * But we have to cycle through all the active advisors for the run and make them check whether they should comment
     * or not.
     */
    bool advisor_commenting;
    // this flag gets set by the configuration file and enables/disables advisor for the whole run
    bool active;
};

class Tier3Greedy : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3Greedy (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3Greedy();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3Greedy() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3GreedyRotation : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3GreedyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3GreedyRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3GreedyRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExitLinear : public Tier3Advisor{
  public:
  // this is constructor for Exit advisor
  Tier3ExitLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitLinear() {};
  // TierExit should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3ExitRotation : public Tier3Advisor{
  public:
  // this is constructor for Exit advisor
  Tier3ExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExitFieldLinear : public Tier3Advisor{
  public:
  // this is constructor for Exitfield advisor
  Tier3ExitFieldLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFieldLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFieldLinear() {};
  // TierExitField should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3ExitFieldRotation : public Tier3Advisor{
  public:
  // this is constructor for Exitfield advisor
  Tier3ExitFieldRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitFieldRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitFieldRotation() {};
  // Tier3ExitField should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExitClosest : public Tier3Advisor{
  public:
  // this is constructor for Exitfield advisor
  Tier3ExitClosest (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitClosest();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitClosest() {};
  // Tier3ExitClosest should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3ExitClosestRotation : public Tier3Advisor{
  public:
  // this is constructor for Exitfield advisor
  Tier3ExitClosestRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3ExitClosestRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3ExitClosestRotation() {};
  // Tier3ExitClosestRotation should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3RegionLeaverLinear : public Tier3Advisor{
  public:
  // this is constructor for RegionLeaver advisor
  Tier3RegionLeaverLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionLeaverLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionLeaverLinear() {};
  // TierRegionLeaver should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3RegionLeaverRotation : public Tier3Advisor{
  public:
  // this is constructor for RegionLeaver advisor
  Tier3RegionLeaverRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3RegionLeaverRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3RegionLeaverRotation() {};
  // Tier3RegionLeaver should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3EnterLinear : public Tier3Advisor{
  public:
  // this is constructor for Enter advisor
  Tier3EnterLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3EnterLinear();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3EnterLinear() {};
  // TierEnter should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3EnterRotation : public Tier3Advisor{
  public:
  // this is constructor for rotationfinder advisor
  Tier3EnterRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3EnterRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3EnterRotation() {};
  // Tier3EnterRotation should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3EnterExit : public Tier3Advisor{
  public:
  // this is constructor for EnterExit advisor
  Tier3EnterExit (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3EnterExit();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3EnterExit() {};
  // TierEnterExit should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3EnterExitRotation : public Tier3Advisor{
  public:
  // this is constructor for enterexitrotation advisor
  Tier3EnterExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3EnterExitRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3EnterExitRotation() {};
  // Tier3EnterExitRotation should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3AvoidRobot : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3AvoidRobot (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3AvoidRobot();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3AvoidRobot() {};
  // Tier3AvoidRobot should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3AvoidRobotRotation : public Tier3Advisor{
  public:
  // this is constructor for greedy advisor
  Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Here is dummy default constructor
  Tier3AvoidRobotRotation();
  // Destructor have to be declared for non-virtual classes, virtual; reasons stated in the parent class
  virtual ~Tier3AvoidRobotRotation() {};
  // Tier3Greedy should inherit constructor of the Tier3Advisor class, nothing more is needed
  // only difference is implementation of action comment function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ElbowRoom : public Tier3Advisor{
 public:
  Tier3ElbowRoom(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Default constructor
  Tier3ElbowRoom();
  // Same as for Tier3Greedy, need to define destructor
  virtual ~Tier3ElbowRoom () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ElbowRoomRotation : public Tier3Advisor{
 public:
  Tier3ElbowRoomRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true); 
  // Default constructor
  Tier3ElbowRoomRotation();
  // Same as for Tier3Greedy, need to define destructor
  virtual ~Tier3ElbowRoomRotation () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};
/*
class Tier3CloseIn : public Tier3Advisor{
 public:
  Tier3CloseIn(Beliefs *beliefs, string name, string description, double weight, double * magic_init, bool is_active = true); 
  // Default constructor
  Tier3CloseIn();

  virtual ~Tier3CloseIn () {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3CloseInRotation : public Tier3Advisor{
 public:
  Tier3CloseInRotation(Beliefs *beliefs, string name, string description, double weight, double * magic_init, bool is_active = true); 
  // Default constructor
  Tier3CloseInRotation();

  virtual ~Tier3CloseInRotation() {};
  // This type of advisor will avoid obstacles by implementing Gompertz function
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};
*/
// this is advisor that favors big steps
class Tier3BigStep : public Tier3Advisor{
 public:
  Tier3BigStep(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BigStep();
  
  virtual ~Tier3BigStep() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3BigStepRotation : public Tier3Advisor{
 public:
  Tier3BigStepRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BigStepRotation();
  
  virtual ~Tier3BigStepRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};


class Tier3GoAroundRotation : public Tier3Advisor{
 public:
  Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3GoAroundRotation();
  
  virtual ~Tier3GoAroundRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};



// Idea for this advisor
// since it is fuzzy it should allow robot to go away from the target if it
// is close to it (assume going around the wall)
// But the further away from the target the more paranoid it should become

/* advisor added Jan. 13, 2014
 * To strengthen the moves in the direction of the target I added this
 * advisor that gives positive comment to all the moves that are 45 degrees
 * from the line that connects robot and target-> then we consider that we
 * are generally getting closer to the target with those steps.
 * Moves that are between 45 and 90 degrees from that line, in both 
 * directions are considered possible for this advisor so it gives them 0.
 * And this advisor does not like moves that are away from the target so
 * it gives -5 to those actions.
 */ 
class Tier3Unlikely : public Tier3Advisor{
 public:
  Tier3Unlikely(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3Unlikely();
  
  virtual ~Tier3Unlikely() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3UnlikelyRotation : public Tier3Advisor{
 public:
  Tier3UnlikelyRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3UnlikelyRotation();
  
  virtual ~Tier3UnlikelyRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3UnlikelyField : public Tier3Advisor{
 public:
  Tier3UnlikelyField(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3UnlikelyField();
  
  virtual ~Tier3UnlikelyField() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3UnlikelyFieldRotation : public Tier3Advisor{
 public:
  Tier3UnlikelyFieldRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3UnlikelyFieldRotation();
  
  virtual ~Tier3UnlikelyFieldRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3Explorer : public Tier3Advisor{
 public:
  Tier3Explorer(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3Explorer();
  
  virtual ~Tier3Explorer() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExplorerRotation : public Tier3Advisor{
 public:
  Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3ExplorerRotation();
  
  virtual ~Tier3ExplorerRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExplorerEndPoints : public Tier3Advisor{
 public:
  Tier3ExplorerEndPoints(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3ExplorerEndPoints();
  
  virtual ~Tier3ExplorerEndPoints() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3ExplorerEndPointsRotation : public Tier3Advisor{
 public:
  Tier3ExplorerEndPointsRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3ExplorerEndPointsRotation();
  
  virtual ~Tier3ExplorerEndPointsRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3BaseLine : public Tier3Advisor{
 public:
  Tier3BaseLine(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BaseLine();
  
  virtual ~Tier3BaseLine() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};

class Tier3BaseLineRotation : public Tier3Advisor{
 public:
  Tier3BaseLineRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  
  Tier3BaseLineRotation();
  
  virtual ~Tier3BaseLineRotation() {};
  
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();
};



class Tier3ConveyLinear : public Tier3Advisor{
 public: 
  Tier3ConveyLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ConveyLinear();
  
  virtual ~Tier3ConveyLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3ConveyRotation : public Tier3Advisor{
 public: 
  Tier3ConveyRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ConveyRotation();
  
  virtual ~Tier3ConveyRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3TrailerLinear : public Tier3Advisor{
 public: 
  Tier3TrailerLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3TrailerLinear();
  
  virtual ~Tier3TrailerLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};



class Tier3TrailerRotation : public Tier3Advisor{
 public: 
  Tier3TrailerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3TrailerRotation();
  
  virtual ~Tier3TrailerRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3EnterDoorLinear : public Tier3Advisor{
 public: 
  Tier3EnterDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3EnterDoorLinear();
  
  virtual ~Tier3EnterDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3EnterDoorRotation : public Tier3Advisor{
 public: 
  Tier3EnterDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3EnterDoorRotation();
  
  virtual ~Tier3EnterDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3ExitDoorLinear : public Tier3Advisor{
 public: 
  Tier3ExitDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ExitDoorLinear();
  
  virtual ~Tier3ExitDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3ExitDoorRotation : public Tier3Advisor{
 public: 
  Tier3ExitDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ExitDoorRotation();
  
  virtual ~Tier3ExitDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3AccessLinear : public Tier3Advisor{
 public: 
  Tier3AccessLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3AccessLinear();
  
  virtual ~Tier3AccessLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3AccessRotation : public Tier3Advisor{
 public: 
  Tier3AccessRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3AccessRotation();
  
  virtual ~Tier3AccessRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

/*
class Tier3NeighborDoorLinear : public Tier3Advisor{
 public: 
  Tier3NeighborDoorLinear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3NeighborDoorLinear();
  
  virtual ~Tier3NeighborDoorLinear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3NeighborDoorRotation : public Tier3Advisor{
 public: 
  Tier3NeighborDoorRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3NeighborDoorRotation();
  
  virtual ~Tier3NeighborDoorRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};*/

class Tier3LearnSpatialModel : public Tier3Advisor{
 public: 
  Tier3LearnSpatialModel(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3LearnSpatialModel();
  
  virtual ~Tier3LearnSpatialModel(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3LearnSpatialModelRotation : public Tier3Advisor{
 public: 
  Tier3LearnSpatialModelRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3LearnSpatialModelRotation();
  
  virtual ~Tier3LearnSpatialModelRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3Curiosity : public Tier3Advisor{
 public: 
  Tier3Curiosity(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Curiosity();
  
  virtual ~Tier3Curiosity(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3CuriosityRotation : public Tier3Advisor{
 public: 
  Tier3CuriosityRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3CuriosityRotation();
  
  virtual ~Tier3CuriosityRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3Enfilade : public Tier3Advisor{
 public: 
  Tier3Enfilade(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Enfilade();
  
  virtual ~Tier3Enfilade(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3EnfiladeRotation : public Tier3Advisor{
 public: 
  Tier3EnfiladeRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3EnfiladeRotation();
  
  virtual ~Tier3EnfiladeRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Thigmotaxis : public Tier3Advisor{
 public: 
  Tier3Thigmotaxis(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Thigmotaxis();
  
  virtual ~Tier3Thigmotaxis(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3ThigmotaxisRotation : public Tier3Advisor{
 public: 
  Tier3ThigmotaxisRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3ThigmotaxisRotation();
  
  virtual ~Tier3ThigmotaxisRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3VisualScanRotation : public Tier3Advisor{
 public: 
  Tier3VisualScanRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3VisualScanRotation();
  
  virtual ~Tier3VisualScanRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3LeastAngle : public Tier3Advisor{
 public: 
  Tier3LeastAngle(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3LeastAngle();
  
  virtual ~Tier3LeastAngle(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3LeastAngleRotation : public Tier3Advisor{
 public: 
  Tier3LeastAngleRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3LeastAngleRotation();
  
  virtual ~Tier3LeastAngleRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Interpersonal : public Tier3Advisor{
 public: 
  Tier3Interpersonal(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Interpersonal();
  
  virtual ~Tier3Interpersonal(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3InterpersonalRotation : public Tier3Advisor{
 public: 
  Tier3InterpersonalRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3InterpersonalRotation();
  
  virtual ~Tier3InterpersonalRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

/*class Tier3Formation : public Tier3Advisor{
 public: 
  Tier3Formation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Formation();
  
  virtual ~Tier3Formation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FormationRotation : public Tier3Advisor{
 public: 
  Tier3FormationRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FormationRotation();
  
  virtual ~Tier3FormationRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};*/

class Tier3Front : public Tier3Advisor{
 public: 
  Tier3Front(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Front();
  
  virtual ~Tier3Front(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FrontRotation : public Tier3Advisor{
 public: 
  Tier3FrontRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FrontRotation();
  
  virtual ~Tier3FrontRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Rear : public Tier3Advisor{
 public: 
  Tier3Rear(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Rear();
  
  virtual ~Tier3Rear(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3RearRotation : public Tier3Advisor{
 public: 
  Tier3RearRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3RearRotation();
  
  virtual ~Tier3RearRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Side : public Tier3Advisor{
 public: 
  Tier3Side(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Side();
  
  virtual ~Tier3Side(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3SideRotation : public Tier3Advisor{
 public: 
  Tier3SideRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3SideRotation();
  
  virtual ~Tier3SideRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Visible : public Tier3Advisor{
 public: 
  Tier3Visible(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Visible();
  
  virtual ~Tier3Visible(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3VisibleRotation : public Tier3Advisor{
 public: 
  Tier3VisibleRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3VisibleRotation();
  
  virtual ~Tier3VisibleRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

/*class Tier3Wait : public Tier3Advisor{
 public: 
  Tier3Wait(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Wait();
  
  virtual ~Tier3Wait(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3WaitRotation : public Tier3Advisor{
 public: 
  Tier3WaitRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3WaitRotation();
  
  virtual ~Tier3WaitRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};*/

class Tier3CrowdAvoid : public Tier3Advisor{
 public: 
  Tier3CrowdAvoid(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3CrowdAvoid();
  
  virtual ~Tier3CrowdAvoid(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3CrowdAvoidRotation : public Tier3Advisor{
 public: 
  Tier3CrowdAvoidRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3CrowdAvoidRotation();
  
  virtual ~Tier3CrowdAvoidRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3FindTheCrowd : public Tier3Advisor{
 public: 
  Tier3FindTheCrowd(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheCrowd();
  
  virtual ~Tier3FindTheCrowd(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FindTheCrowdRotation : public Tier3Advisor{
 public: 
  Tier3FindTheCrowdRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheCrowdRotation();
  
  virtual ~Tier3FindTheCrowdRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3RiskAvoid : public Tier3Advisor{
 public: 
  Tier3RiskAvoid(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3RiskAvoid();
  
  virtual ~Tier3RiskAvoid(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3RiskAvoidRotation : public Tier3Advisor{
 public: 
  Tier3RiskAvoidRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3RiskAvoidRotation();
  
  virtual ~Tier3RiskAvoidRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3FindTheRisk : public Tier3Advisor{
 public: 
  Tier3FindTheRisk(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheRisk();
  
  virtual ~Tier3FindTheRisk(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FindTheRiskRotation : public Tier3Advisor{
 public: 
  Tier3FindTheRiskRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheRiskRotation();
  
  virtual ~Tier3FindTheRiskRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3FlowAvoid : public Tier3Advisor{
 public: 
  Tier3FlowAvoid(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FlowAvoid();
  
  virtual ~Tier3FlowAvoid(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FlowAvoidRotation : public Tier3Advisor{
 public: 
  Tier3FlowAvoidRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FlowAvoidRotation();
  
  virtual ~Tier3FlowAvoidRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3FindTheFlow : public Tier3Advisor{
 public: 
  Tier3FindTheFlow(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheFlow();
  
  virtual ~Tier3FindTheFlow(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FindTheFlowRotation : public Tier3Advisor{
 public: 
  Tier3FindTheFlowRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FindTheFlowRotation();
  
  virtual ~Tier3FindTheFlowRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Follow : public Tier3Advisor{
 public: 
  Tier3Follow(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Follow();
  
  virtual ~Tier3Follow(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3FollowRotation : public Tier3Advisor{
 public: 
  Tier3FollowRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3FollowRotation();
  
  virtual ~Tier3FollowRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Crossroads : public Tier3Advisor{
 public: 
  Tier3Crossroads(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Crossroads();
  
  virtual ~Tier3Crossroads(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3CrossroadsRotation : public Tier3Advisor{
 public: 
  Tier3CrossroadsRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3CrossroadsRotation();
  
  virtual ~Tier3CrossroadsRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};

class Tier3Stay : public Tier3Advisor{
 public: 
  Tier3Stay(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3Stay();
  
  virtual ~Tier3Stay(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};


class Tier3StayRotation : public Tier3Advisor{
 public: 
  Tier3StayRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active = true);
  Tier3StayRotation();
  
  virtual ~Tier3StayRotation(){};
  virtual double actionComment(FORRAction action);
  virtual void set_commenting();

};
#endif



