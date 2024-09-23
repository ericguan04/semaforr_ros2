/*!
 * FORRAction.h
 *
 * \brief 
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 */
#ifndef FORRACTION_H
#define FORRACTION_H

#include <string>
#include <iostream>
#include <set>


enum FORRActionType {
  FORWARD,
  RIGHT_TURN,
  LEFT_TURN,
  PAUSE
};

class FORRAction {

  public:
    FORRActionType type;
    //    void * parameter;  
    // this is number 1-5 for motion command in Localization 
    // it determines the "intensity" of movement
    int parameter;
    // This is to make my life easier with Advisors
    // Each FORRAction type will have string description
    //std::string actionDescriptions[] = {"Noop", "Forward", "Backward", "Right turn", "Left turn", "Wide right turn", "Wide left turn", "Pause", "Halt"};
    
    // This part was a lot of headache < operator has to 
    // be defined if you want to use the class as a key
    // for a map and it has to be const function that
    // accepts const argument otherwise compiler will kill you
    bool operator < (const FORRAction &action) const;

    bool operator == (const FORRAction &action) const;

    // I need constructor now that this is a class
    FORRAction(FORRActionType actionType, int par):type(actionType), parameter(par) {};
    // Also default constructor once you defined others
    FORRAction(): type(PAUSE), parameter(0) {};

    // Since I am returning map whose key is this class
    // I think I need copy constructor
    FORRAction(const FORRAction& action);


};


#endif
