/*!
 * FORRActionStats.h
 *
 * Class that collects data about a decision
 *
 * \author Raj Korpan <rkorpan@gradcenter.cuny.edu>
 */
#ifndef FORRACTIONSTATS_H
#define FORRACTIONSTATS_H

#include <string>
#include <iostream>
#include <set>

class FORRActionStats {

  public:
    double decisionTier;
    std::string vetoedActions;
    std::string advisors;
    std::string advisorComments;
    std::string advisorInfluence;
    double planningComputationTime;
    double learningComputationTime;
    double graphingComputationTime;
    std::string chosenPlanner;
    std::string plannerComments;

    FORRActionStats(double decTier, std::string vActions, std::string adv, std::string advComments, std::string advInfluence, double planTime, double learnTime, double graphTime, std::string chsPlan, std::string plnComments) : decisionTier(decTier), vetoedActions(vActions), advisors(adv), advisorComments(advComments), advisorInfluence(advInfluence), planningComputationTime(planTime), learningComputationTime(learnTime), graphingComputationTime(graphTime), chosenPlanner(chsPlan), plannerComments(plnComments) {};
    FORRActionStats(): decisionTier(0), vetoedActions(" "), advisors(" "), advisorComments(" "), advisorInfluence(" "), planningComputationTime(0), learningComputationTime(0), graphingComputationTime(0), chosenPlanner(" "), plannerComments(" ") {};

};


#endif
