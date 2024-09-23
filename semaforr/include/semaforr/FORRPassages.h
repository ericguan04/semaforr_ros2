/************************************************
FORRPassages.h 
This file contains the class which contains information about the passages that FORR learns and uses

Written by Raj Korpan, 2020
**********************************************/

#ifndef FORRPASSAGES_H
#define FORRPASSAGES_H

#include <vector>
#include <string>
#include <limits>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <utility>      //for exit
#include <algorithm>
#include "FORRGeometry.h"

/* FORRPassages class
 *
 * FORRPassages represent the passages that are learned by SemaFORR.
 *
 */
class FORRPassages{
public:
    FORRPassages(vector< vector<int> > hg, AgentState* as){
        highway_grid = hg;
        // cout << "Highway grid" << endl;
        // for(int i = 0; i < highway_grid.size(); i++){
        //   for(int j = 0; j < highway_grid[0].size(); j++){
        //     cout << highway_grid[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        agentState = as;
    };
    vector< vector<int> > getPassages(){return passages_grid;}
    void setPassages(vector< vector<int> > pg) { passages_grid = pg; }
    ~FORRPassages(){};

    void clearAllPassages(){
        passages_grid.clear();
    }
    map<int, vector< vector<int> > > getGraphNodes() {return graph_nodes;}
    map<int, vector< vector<int> > > getGraphEdges() {return graph_edges_map;}
    vector< vector<int> > getGraph() {return reduced_graph;}
    vector< vector<int> > getAveragePassage() {return average_passage;}
    vector< vector<CartesianPoint> > getGraphTrails() {return graph_trails;}
    vector< vector<int> > getGraphThroughIntersections() {return graph_through_intersections;}
    vector< vector<CartesianPoint> > getGraphIntersectionTrails() {return graph_intersection_trails;}


    void learnPassages(vector<CartesianPoint> stepped_history, vector < vector<CartesianPoint> > stepped_laser_history) {
        int min_passage_length = 7;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j ++){
            col.push_back(0);
          }
          decisions_grid.push_back(col);
        }
        // for(int i = 0; i < highway_grid.size(); i++){
        //   vector<int> col;
        //   for(int j = 0; j < highway_grid[i].size(); j ++){
        //     col.push_back(0);
        //   }
        //   decisions_only_grid.push_back(col);
        // }
        // vector< vector< vector<int> > > laser_grid_connections;
        // for(int i = 0; i < highway_grid.size(); i++){
        //   vector< vector<int> > col_connect;
        //   for(int j = 0; j < highway_grid[i].size(); j ++){
        //     vector<int> connections;
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     connections.push_back(0);
        //     col_connect.push_back(connections); // N Incoming, S Incoming, E Incoming, W Incoming, N Outgoing, S Outgoing, E Outgoing, W Outgoing
        //   }
        //   laser_grid_connections.push_back(col_connect);
        // }
        double step_length = 1;
        for(int k = 0; k < stepped_history.size()-1 ; k++){
          double start_x = stepped_history[k].get_x();
          if(int(start_x) < 0)
            start_x = 0;
          if(int(start_x) >= decisions_grid.size())
            start_x = decisions_grid.size()-1;
          double start_y = stepped_history[k].get_y();
          if(int(start_y) < 0)
            start_y = 0;
          if(int(start_y) >= decisions_grid[0].size())
            start_y = decisions_grid[0].size()-1;
          double end_x = stepped_history[k+1].get_x();
          if(int(end_x) < 0)
            end_x = 0;
          if(int(end_x) >= decisions_grid.size())
            end_x = decisions_grid.size()-1;
          double end_y = stepped_history[k+1].get_y();
          if(int(end_y) < 0)
            end_y = 0;
          if(int(end_y) >= decisions_grid[0].size())
            end_y = decisions_grid[0].size()-1;
          decisions_grid[int(start_x)][int(start_y)] = 1;
          decisions_grid[int(end_x)][int(end_y)] = 1;
          // decisions_only_grid[int(start_x)][int(start_y)] = 1;
          // decisions_only_grid[int(end_x)][int(end_y)] = 1;
          // decisions_grid[floor(start_y)][floor(start_x)] = 1;
          // decisions_grid[floor(end_y)][floor(end_x)] = 1;
          // decisions_grid[ceil(start_y)][ceil(start_x)] = 1;
          // decisions_grid[ceil(end_y)][ceil(end_x)] = 1;
          // decisions_grid[floor(start_y)][ceil(start_x)] = 1;
          // decisions_grid[floor(end_y)][ceil(end_x)] = 1;
          // decisions_grid[ceil(start_y)][floor(start_x)] = 1;
          // decisions_grid[ceil(end_y)][floor(end_x)] = 1;
          // vector< vector< vector<int> > > decision_laser_grid_connections;
          // for(int i = 0; i < highway_grid.size(); i++){
          //   vector< vector<int> > col_connect;
          //   for(int j = 0; j < highway_grid[i].size(); j ++){
          //     vector<int> connections;
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     connections.push_back(0);
          //     col_connect.push_back(connections); // N Incoming, S Incoming, E Incoming, W Incoming, N Outgoing, S Outgoing, E Outgoing, W Outgoing
          //   }
          //   decision_laser_grid_connections.push_back(col_connect);
          // }
          double length = sqrt((start_x - end_x) * (start_x - end_x) + (start_y - end_y) * (start_y - end_y));
          // cout << k << " " << start_x << " " << start_y << " " << end_x << " " << end_y << " length " << length << endl;
          if(length >= step_length or int(start_x) != int(end_x) or int(start_y) != int(end_y)){
            double step_size = step_length / length;
            if(step_size > 0.1){
              step_size = 0.1;
            }
            double tx, ty;
            // cout << "step_size " << step_size << endl;
            for(double j = 0; j <= 1; j += step_size){
              tx = (end_x * j) + (start_x * (1 - j));
              ty = (end_y * j) + (start_y * (1 - j));
              if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
                // cout << tx << " " << ty << endl;
                decisions_grid[int(tx)][int(ty)] = 1;
                // decisions_only_grid[int(tx)][int(ty)] = 1;
                // if(int(start_x) > int(end_x) and int(start_y) == int(end_y)){
                //   if(int(tx) != int(start_x)){
                //     // cout << "update 2" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][2] = 1;
                //   }
                //   if(int(tx) != int(end_x)){
                //     // cout << "update 7" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][7] = 1;
                //   }
                // }
                // else if(int(start_x) == int(end_x) and int(start_y) > int(end_y)){
                //   if(int(ty) != int(start_y)){
                //     // cout << "update 0" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][0] = 1;
                //   }
                //   if(int(ty) != int(end_y)){
                //     // cout << "update 5" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][5] = 1;
                //   }
                // }
                // else if(int(start_x) < int(end_x) and int(start_y) == int(end_y)){
                //   if(int(tx) != int(start_x)){
                //     // cout << "update 3" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][3] = 1;
                //   }
                //   if(int(tx) != int(end_x)){
                //     // cout << "update 6" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][6] = 1;
                //   }
                // }
                // else if(int(start_x) == int(end_x) and int(start_y) < int(end_y)){
                //   if(int(ty) != int(start_y)){
                //     // cout << "update 1" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][1] = 1;
                //   }
                //   if(int(ty) != int(end_y)){
                //     // cout << "update 4" << endl;
                //     decision_laser_grid_connections[int(tx)][int(ty)][4] = 1;
                //   }
                // }
              }
              // decisions_grid[floor(ty)][floor(tx)] = 1
              // decisions_grid[ceil(ty)][ceil(tx)] = 1
              // decisions_grid[floor(ty)][ceil(tx)] = 1
              // decisions_grid[ceil(ty)][floor(tx)] = 1
            }
          }
          // for(int i = 0; i < decision_laser_grid_connections.size(); i++){
          //   for(int j = 0; j < decision_laser_grid_connections[0].size(); j ++){
          //     for(int l = 0; l < decision_laser_grid_connections[0][0].size(); l++){
          //       laser_grid_connections[i][j][l] += decision_laser_grid_connections[i][j][l];
          //     }
          //   }
          // }
        }
        // cout << "After decisions_grid" << endl;
        // for(int i = 0; i < decisions_grid.size(); i++){
        //   for(int j = 0; j < decisions_grid[0].size(); j++){
        //     cout << decisions_grid[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // vector< vector<int> > lasers_grid;
        // for(int i = 0; i < highway_grid.size(); i++){
        //   vector<int> col;
        //   for(int j = 0; j < highway_grid[i].size(); j ++){
        //     col.push_back(0);
        //   }
        //   lasers_grid.push_back(col);
        // }
        // for(int k = 0; k < stepped_laser_history.size(); k++){
        //   double start_x = stepped_history[k].get_x();
        //   if(int(start_x) < 0)
        //     start_x = 0;
        //   if(int(start_x) >= decisions_grid.size())
        //     start_x = decisions_grid.size()-1;
        //   double start_y = stepped_history[k].get_y();
        //   if(int(start_y) < 0)
        //     start_y = 0;
        //   if(int(start_y) >= decisions_grid[0].size())
        //     start_y = decisions_grid[0].size()-1;
        //   vector< vector<int> > decision_lasers_grid;
        //   // vector< vector< vector<int> > > decision_laser_grid_connections;
        //   for(int i = 0; i < highway_grid.size(); i++){
        //     vector<int> col;
        //     // vector< vector<int> > col_connect;
        //     for(int j = 0; j < highway_grid[i].size(); j ++){
        //       col.push_back(0);
        //       // vector<int> connections;
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // connections.push_back(0);
        //       // col_connect.push_back(connections); // N Incoming, S Incoming, E Incoming, W Incoming, N Outgoing, S Outgoing, E Outgoing, W Outgoing
        //     }
        //     decision_lasers_grid.push_back(col);
        //     // decision_laser_grid_connections.push_back(col_connect);
        //   }
        //   for(int j = 0; j < stepped_laser_history[k].size(); j++){
        //     double end_x = stepped_laser_history[k][j].get_x();
        //     if(int(end_x) < 0)
        //       end_x = 0;
        //     if(int(end_x) >= decisions_grid.size())
        //       end_x = decisions_grid.size()-1;
        //     double end_y = stepped_laser_history[k][j].get_y();
        //     if(int(end_y) < 0)
        //       end_y = 0;
        //     if(int(end_y) >= decisions_grid[0].size())
        //       end_y = decisions_grid[0].size()-1;
        //     if((int(start_y) == int(end_y)) or (int(start_x) == int(end_x))){
        //       double length = sqrt((start_x - end_x) * (start_x - end_x) + (start_y - end_y) * (start_y - end_y));
        //       // cout << k << " " << start_x << " " << start_y << " " << end_x << " " << end_y << " length " << length << endl;
        //       if(decisions_grid[int(start_x)][int(start_y)] == 1 and decisions_grid[int(end_x)][int(end_y)] == 1){
        //         decision_lasers_grid[int(start_x)][int(start_y)] = 1;
        //         decision_lasers_grid[int(end_x)][int(end_y)] = 1;
        //       }
        //       if(length >= step_length or int(start_x) != int(end_x) or int(start_y) != int(end_y)){
        //         double step_size = step_length / length;
        //         if(step_size > 0.1){
        //           step_size = 0.1;
        //         }
        //         double tx, ty;
        //         // cout << "step_size " << step_size << endl;
        //         // for(double i = 0; i <= 1; i += step_size){
        //         //   tx = (end_x * i) + (start_x * (1 - i));
        //         //   ty = (end_y * i) + (start_y * (1 - i));
        //         //   if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
        //         //     // cout << tx << " " << ty << endl;
        //         //     decision_lasers_grid[int(tx)][int(ty)] = 1;
        //         //   }
        //         //   // decisions_grid[floor(ty)][floor(tx)] = 1
        //         //   // decisions_grid[ceil(ty)][ceil(tx)] = 1
        //         //   // decisions_grid[floor(ty)][ceil(tx)] = 1
        //         //   // decisions_grid[ceil(ty)][floor(tx)] = 1
        //         // }
        //         if(int(start_x) > int(end_x) and int(start_y) == int(end_y)){
        //           for(double i = 0; i <= 1; i += step_size){
        //             tx = (end_x * i) + (start_x * (1 - i));
        //             ty = (end_y * i) + (start_y * (1 - i));
        //             if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
        //               // cout << tx << " " << ty << " " << int(tx) << " " << int(ty) << " " << int(start_x) << " " << int(start_y) << " " << int(end_x) << " " << int(end_y) << endl;
        //               // cout << (int(tx) != int(start_x)) << " " << (int(ty) != int(start_y)) << " " << (int(tx) != int(end_x)) << " " << (int(ty) != int(end_y)) << endl;
        //               if(decisions_grid[int(start_x)][int(start_y)] == 1 and decisions_grid[int(end_x)][int(end_y)] == 1){
        //                 decision_lasers_grid[int(tx)][int(ty)] = 1;
        //               }
        //               // if(int(tx) != int(start_x)){
        //               //   // cout << "update 2" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][2] = 1;
        //               // }
        //               // if(int(tx) != int(end_x)){
        //               //   // cout << "update 7" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][7] = 1;
        //               // }
        //               // for(int l = 0; l < decision_laser_grid_connections[int(tx)][int(ty)].size(); l++){
        //               //   cout << decision_laser_grid_connections[int(tx)][int(ty)][l] << ",";
        //               // }
        //               // cout << endl;
        //             }
        //           }
        //         }
        //         else if(int(start_x) == int(end_x) and int(start_y) > int(end_y)){
        //           for(double i = 0; i <= 1; i += step_size){
        //             tx = (end_x * i) + (start_x * (1 - i));
        //             ty = (end_y * i) + (start_y * (1 - i));
        //             if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
        //               // cout << tx << " " << ty << " " << int(tx) << " " << int(ty) << " " << int(start_x) << " " << int(start_y) << " " << int(end_x) << " " << int(end_y) << endl;
        //               // cout << (int(tx) != int(start_x)) << " " << (int(ty) != int(start_y)) << " " << (int(tx) != int(end_x)) << " " << (int(ty) != int(end_y)) << endl;
        //               if(decisions_grid[int(start_x)][int(start_y)] == 1 and decisions_grid[int(end_x)][int(end_y)] == 1){
        //                 decision_lasers_grid[int(tx)][int(ty)] = 1;
        //               }
        //               // if(int(ty) != int(start_y)){
        //               //   // cout << "update 0" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][0] = 1;
        //               // }
        //               // if(int(ty) != int(end_y)){
        //               //   // cout << "update 5" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][5] = 1;
        //               // }
        //               // for(int l = 0; l < decision_laser_grid_connections[int(tx)][int(ty)].size(); l++){
        //               //   cout << decision_laser_grid_connections[int(tx)][int(ty)][l] << ",";
        //               // }
        //               // cout << endl;
        //             }
        //           }
        //         }
        //         else if(int(start_x) < int(end_x) and int(start_y) == int(end_y)){
        //           for(double i = 0; i <= 1; i += step_size){
        //             tx = (end_x * i) + (start_x * (1 - i));
        //             ty = (end_y * i) + (start_y * (1 - i));
        //             if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
        //               // cout << tx << " " << ty << " " << int(tx) << " " << int(ty) << " " << int(start_x) << " " << int(start_y) << " " << int(end_x) << " " << int(end_y) << endl;
        //               // cout << (int(tx) != int(start_x)) << " " << (int(ty) != int(start_y)) << " " << (int(tx) != int(end_x)) << " " << (int(ty) != int(end_y)) << endl;
        //               if(decisions_grid[int(start_x)][int(start_y)] == 1 and decisions_grid[int(end_x)][int(end_y)] == 1){
        //                 decision_lasers_grid[int(tx)][int(ty)] = 1;
        //               }
        //               // if(int(tx) != int(start_x)){
        //               //   // cout << "update 3" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][3] = 1;
        //               // }
        //               // if(int(tx) != int(end_x)){
        //               //   // cout << "update 6" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][6] = 1;
        //               // }
        //               // for(int l = 0; l < decision_laser_grid_connections[int(tx)][int(ty)].size(); l++){
        //               //   cout << decision_laser_grid_connections[int(tx)][int(ty)][l] << ",";
        //               // }
        //               // cout << endl;
        //             }
        //           }
        //         }
        //         else if(int(start_x) == int(end_x) and int(start_y) < int(end_y)){
        //           for(double i = 0; i <= 1; i += step_size){
        //             tx = (end_x * i) + (start_x * (1 - i));
        //             ty = (end_y * i) + (start_y * (1 - i));
        //             if(int(tx) >= 0 and int(ty) >= 0 and int(tx) < decisions_grid.size() and int(ty) < decisions_grid[0].size()){
        //               // cout << tx << " " << ty << " " << int(tx) << " " << int(ty) << " " << int(start_x) << " " << int(start_y) << " " << int(end_x) << " " << int(end_y) << endl;
        //               // cout << (int(tx) != int(start_x)) << " " << (int(ty) != int(start_y)) << " " << (int(tx) != int(end_x)) << " " << (int(ty) != int(end_y)) << endl;
        //               if(decisions_grid[int(start_x)][int(start_y)] == 1 and decisions_grid[int(end_x)][int(end_y)] == 1){
        //                 decision_lasers_grid[int(tx)][int(ty)] = 1;
        //               }
        //               // if(int(ty) != int(start_y)){
        //               //   // cout << "update 1" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][1] = 1;
        //               // }
        //               // if(int(ty) != int(end_y)){
        //               //   // cout << "update 4" << endl;
        //               //   decision_laser_grid_connections[int(tx)][int(ty)][4] = 1;
        //               // }
        //               // for(int l = 0; l < decision_laser_grid_connections[int(tx)][int(ty)].size(); l++){
        //               //   cout << decision_laser_grid_connections[int(tx)][int(ty)][l] << ",";
        //               // }
        //               // cout << endl;
        //             }
        //           }
        //         }
        //       }
        //     }
        //   }
        //   for(int i = 0; i < decision_lasers_grid.size(); i++){
        //     for(int j = 0; j < decision_lasers_grid[0].size(); j ++){
        //       lasers_grid[i][j] += decision_lasers_grid[i][j];
        //       // for(int l = 0; l < decision_laser_grid_connections[0][0].size(); l++){
        //       //   laser_grid_connections[i][j][l] += decision_laser_grid_connections[i][j][l];
        //       // }
        //     }
        //   }
        // }
        // // cout << "After lasers_grid" << endl;
        // // for(int i = 0; i < lasers_grid.size(); i++){
        // //   for(int j = 0; j < lasers_grid[0].size(); j++){
        // //     cout << lasers_grid[i][j] << " ";
        // //   }
        // //   cout << endl;
        // // }
        // // cout << "After laser_grid_connections" << endl;
        // // for(int i = 0; i < laser_grid_connections.size(); i++){
        // //   for(int j = 0; j < laser_grid_connections[0].size(); j++){
        // //     for(int k = 0; k < laser_grid_connections[0][0].size(); k++){
        // //       cout << laser_grid_connections[i][j][k] << ",";
        // //     }
        // //     cout << " ";
        // //   }
        // //   cout << endl;
        // // }
        // // cout << "After decisions_grid + lasers_grid" << endl;
        // for(int i = 0; i < decisions_grid.size(); i++){
        //   for(int j = 0; j < decisions_grid[0].size(); j++){
        //     // if(lasers_grid[i][j] > 1 and ((laser_grid_connections[i][j][0] > 1 and laser_grid_connections[i][j][5] > 1) or (laser_grid_connections[i][j][1] > 1 and laser_grid_connections[i][j][4] > 1) or (laser_grid_connections[i][j][2] > 1 and laser_grid_connections[i][j][7] > 1) or (laser_grid_connections[i][j][3] > 1 and laser_grid_connections[i][j][6] > 1)) and decisions_grid[i][j] == 0){
        //     if(lasers_grid[i][j] > 4 and decisions_grid[i][j] == 0){
        //       decisions_grid[i][j] = 1;
        //     }
        //     // cout << decisions_grid[i][j] << " ";
        //   }
        //   // cout << endl;
        // }
        vector< vector<int> > passage_grid;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j ++){
            // if(highway_grid[i][j] < 0){
            //   vector<int> values;
            //   if(i > 0){
            //     if(highway_grid[i-1][j] >= 0)
            //       values.push_back(highway_grid[i-1][j]);
            //   }
            //   if(j > 0){
            //     if(highway_grid[i][j-1] >= 0)
            //       values.push_back(highway_grid[i][j-1]);
            //   }
            //   if(i < highway_grid.size()-1){
            //     if(highway_grid[i+1][j] >= 0)
            //       values.push_back(highway_grid[i+1][j]);
            //   }
            //   if(j < highway_grid[0].size()-1){
            //     if(highway_grid[i][j+1] >= 0)
            //       values.push_back(highway_grid[i][j+1]);
            //   }
            //   if(values.size() >= 3){
            //     col.push_back(1);
            //   }
            //   else if(decisions_grid[i][j] > 0){
            //     col.push_back(1);
            //   }
            //   else{
            //     col.push_back(-1);
            //   }
            // }
            // else{
            //   col.push_back(highway_grid[i][j]);
            // }
            if(decisions_grid[i][j] > 0){
              col.push_back(1);
            }
            // else if(highway_grid[i][j] >= 0){
            //   vector<int> values;
            //   if(i > 0){
            //     if(decisions_grid[i-1][j] > 0)
            //       values.push_back(1);
            //   }
            //   if(j > 0){
            //     if(decisions_grid[i][j-1] > 0)
            //       values.push_back(1);
            //   }
            //   if(i < highway_grid.size()-1){
            //     if(decisions_grid[i+1][j] > 0)
            //       values.push_back(1);
            //   }
            //   if(j < highway_grid[0].size()-1){
            //     if(decisions_grid[i][j+1] > 0)
            //       values.push_back(1);
            //   }
            //   if(values.size() >= 2){
            //     col.push_back(1);
            //   }
            //   else{
            //     col.push_back(-1);
            //   }
            // }
            else if(decisions_grid[i][j] == 0){
              vector<int> values;
              if(i > 0){
                if(decisions_grid[i-1][j] > 0)
                  values.push_back(1);
              }
              if(j > 0){
                if(decisions_grid[i][j-1] > 0)
                  values.push_back(1);
              }
              if(i < highway_grid.size()-1){
                if(decisions_grid[i+1][j] > 0)
                  values.push_back(1);
              }
              if(j < highway_grid[0].size()-1){
                if(decisions_grid[i][j+1] > 0)
                  values.push_back(1);
              }
              // if(values.size() >= 3 and ((laser_grid_connections[i][j][0] > 1 and laser_grid_connections[i][j][5] > 1 and laser_grid_connections[i][j][1] > 1 and laser_grid_connections[i][j][4] > 1) or (laser_grid_connections[i][j][2] > 1 and laser_grid_connections[i][j][7] > 1 and laser_grid_connections[i][j][3] > 1 and laser_grid_connections[i][j][6] > 1))){
              if(values.size() >= 3){
                col.push_back(1);
              }
              else{
                col.push_back(-1);
              }
            }
            else{
              col.push_back(-1);
            }
          }
          passage_grid.push_back(col);
        }
        // cout << "After passage_grid" << endl;
        // for(int i = 0; i < passage_grid.size(); i++){
        //   for(int j = 0; j < passage_grid[0].size(); j++){
        //     cout << passage_grid[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // cout << "Before disconnected_cells" << endl;
        // vector < vector<int> > disconnected_cells;
        // for(int i = 0; i < highway_grid.size(); i++){
        //   vector<int> col;
        //   for(int j = 0; j < highway_grid[i].size(); j ++){
        //     col.push_back(0);
        //   }
        //   disconnected_cells.push_back(col);
        // }
        // for(int i = 0; i < passage_grid.size(); i++){
        //   for(int j = 0; j < passage_grid[0].size(); j++){
        //     if(passage_grid[i][j] > 0){
        //       if(i > 0){
        //         if(passage_grid[i-1][j] > 0){
        //           if(laser_grid_connections[i-1][j][2] <= 1 or laser_grid_connections[i][j][7] <= 1){
        //             disconnected_cells[i][j]++;
        //             disconnected_cells[i-1][j]++;
        //             // vector<int> points;
        //             // points.push_back(i);
        //             // points.push_back(j);
        //             // points.push_back(i-1);
        //             // points.push_back(j);
        //             // disconnected_cells.push_back(points);
        //             // cout << i << " " << j << " " << i-1 << " " << j << " " << laser_grid_connections[i][j][7] << " " << laser_grid_connections[i-1][j][2] << endl;
        //           }
        //         }
        //       }
        //       if(j > 0){
        //         if(passage_grid[i][j-1] > 0){
        //           if(laser_grid_connections[i][j-1][0] <= 1 or laser_grid_connections[i][j][5] <= 1){
        //             disconnected_cells[i][j]++;
        //             disconnected_cells[i][j-1]++;
        //             // vector<int> points;
        //             // points.push_back(i);
        //             // points.push_back(j);
        //             // points.push_back(i);
        //             // points.push_back(j-1);
        //             // disconnected_cells.push_back(points);
        //             // cout << i << " " << j << " " << i << " " << j-1 << " " << laser_grid_connections[i][j][5] << " " << laser_grid_connections[i][j-1][0] << endl;
        //           }
        //         }
        //       }
        //       if(i < passage_grid.size()-1){
        //         if(passage_grid[i+1][j] > 0){
        //           if(laser_grid_connections[i+1][j][3] <= 1 or laser_grid_connections[i][j][6] <= 1){
        //             disconnected_cells[i][j]++;
        //             disconnected_cells[i+1][j]++;
        //             // vector<int> points;
        //             // points.push_back(i);
        //             // points.push_back(j);
        //             // points.push_back(i+1);
        //             // points.push_back(j);
        //             // disconnected_cells.push_back(points);
        //             // cout << i << " " << j << " " << i+1 << " " << j << " " << laser_grid_connections[i][j][6] << " " << laser_grid_connections[i+1][j][3] << endl;
        //           }
        //         }
        //       }
        //       if(j < passage_grid[0].size()-1){
        //         if(passage_grid[i][j+1] > 0){
        //           if(laser_grid_connections[i][j+1][1] <= 1 or laser_grid_connections[i][j][4] <= 1){
        //             disconnected_cells[i][j]++;
        //             disconnected_cells[i][j+1]++;
        //             // vector<int> points;
        //             // points.push_back(i);
        //             // points.push_back(j);
        //             // points.push_back(i);
        //             // points.push_back(j+1);
        //             // disconnected_cells.push_back(points);
        //             // cout << i << " " << j << " " << i << " " << j+1 << " " << laser_grid_connections[i][j][4] << " " << laser_grid_connections[i][j+1][1] << endl;
        //           }
        //         }
        //       }
        //     }
        //   }
        // }
        // cout << "After disconnected_cells" << endl;
        // for(int i = 0; i < disconnected_cells.size(); i++){
        //   for(int j = 0; j < disconnected_cells[0].size(); j++){
        //     cout << disconnected_cells[i][j] << " ";
        //     if(disconnected_cells[i][j] >= 3){
        //       passage_grid[i][j] = -1;
        //     }
        //   }
        //   cout << endl;
        // }
        vector < vector < pair<int, int> > > horizontals;
        vector < vector <int> > horizontal_lengths;
        for(int i = 0; i < passage_grid.size(); i++){
          vector < pair<int, int> > row;
          vector <int> row_lengths;
          int start = -1;
          int stop = -1;
          for(int j = 0; j < passage_grid[i].size(); j++){
            if(passage_grid[i][j] >= 0 and start == -1){
              start = j;
              stop = j;
            }
            else if(passage_grid[i][j] >= 0 and start > -1){
              stop = j;
            }
            else if(passage_grid[i][j] < 0 and start > -1 and stop > -1){
              // cout << "row " << i << " start " << start << " stop " << stop << " length " << stop-start+1 << endl;
              row.push_back(make_pair(start, stop));
              row_lengths.push_back(stop-start+1);
              start = -1;
              stop = -1;
            }
          }
          horizontals.push_back(row);
          horizontal_lengths.push_back(row_lengths);
        }
        vector< vector<int> > horizontal_passages;
        vector< vector<int> > horizontal_passages_filled;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(-1);
          }
          horizontal_passages.push_back(col);
          horizontal_passages_filled.push_back(col);
        }
        for(int i = 0; i < horizontals.size(); i++){
          for(int j = 0; j < horizontals[i].size(); j++){
            if(horizontal_lengths[i][j] >= min_passage_length){
              for(int k = horizontals[i][j].first; k <= horizontals[i][j].second; k++){
                horizontal_passages[i][k] = 1;
                horizontal_passages_filled[i][k] = 1;
              }
            }
          }
        }
        // cout << "After horizontal_passages" << endl;
        // for(int i = 0; i < horizontal_passages.size(); i++){
        //   for(int j = 0; j < horizontal_passages[0].size(); j++){
        //     cout << horizontal_passages[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int i = 0; i < passage_grid.size(); i++){
        //   for(int j = 0; j < passage_grid[i].size(); j++){
        //     if(passage_grid[i][j] >= 0 and horizontal_passages[i][j] == -1){
        //       if(i > 0 and i < passage_grid.size()-1){
        //         if(horizontal_passages[i-1][j] >= 0 or horizontal_passages[i+1][j] >= 0){
        //           horizontal_passages_filled[i][j] = 1;
        //         }
        //       }
        //       else if(i == 0){
        //         if(horizontal_passages[i+1][j] >= 0){
        //           horizontal_passages_filled[i][j] = 1;
        //         }
        //       }
        //       else if(i == passage_grid.size()-1){
        //         if(horizontal_passages[i-1][j] >= 0){
        //           horizontal_passages_filled[i][j] = 1;
        //         }
        //       }
        //     }
        //   }
        // }
        // cout << "After horizontal_passages_filled" << endl;
        // for(int i = 0; i < horizontal_passages_filled.size(); i++){
        //   for(int j = 0; j < horizontal_passages_filled[0].size(); j++){
        //     cout << horizontal_passages_filled[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        vector<int> dx;
        dx.push_back(1);
        dx.push_back(0);
        dx.push_back(-1);
        dx.push_back(0);
        // dx.push_back(1);
        // dx.push_back(1);
        // dx.push_back(-1);
        // dx.push_back(-1);
        vector<int> dy;
        dy.push_back(0);
        dy.push_back(1);
        dy.push_back(0);
        dy.push_back(-1);
        // dy.push_back(1);
        // dy.push_back(-1);
        // dy.push_back(1);
        // dy.push_back(-1);

        // const int dx[] = {+1, 0, -1, 0, +1, +1, -1, -1};
        // const int dy[] = {0, +1, 0, -1, +1, -1, +1, -1};
        int row_count = horizontal_passages_filled.size();
        int col_count = horizontal_passages_filled[0].size();
        vector< vector<int> > final_horizontal;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(0);
          }
          final_horizontal.push_back(col);
        }
        int horizontal_component = 0;
        for(int i = 0; i < row_count; ++i){
          for(int j = 0; j < col_count; ++j){
            if(final_horizontal[i][j] == 0 && horizontal_passages_filled[i][j] >= 0){
              agentState->dfs(i, j, ++horizontal_component, dx, dy, row_count, col_count, &final_horizontal, &horizontal_passages_filled);
            }
          }
        }
        // cout << "final horizontal_component " << horizontal_component << endl;
        // cout << "After final_horizontal" << endl;
        // for(int i = 0; i < final_horizontal.size(); i++){
        //   for(int j = 0; j < final_horizontal[0].size(); j++){
        //     cout << final_horizontal[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int i = 0; i < final_horizontal.size(); i++){
        //   for(int j = 0; j < final_horizontal[0].size(); j++){
        //     if(final_horizontal[i][j] > 0){
        //       cout << i << " " << j << endl;
        //     }
        //   }
        // }
        vector < vector<int> > horizontal_ends;
        for(int i = 1; i <= horizontal_component; i++){
          // cout << "horizontal_component " << i << endl;
          int right = -1;
          int left = final_horizontal[0].size();
          // cout << right << " " << left << endl;
          for(int j = 0; j < final_horizontal.size(); j++){
            for(int k = 0; k < final_horizontal[j].size(); k++){
              if(final_horizontal[j][k] == i){
                if(k > right){
                  right = k;
                }
                if(k < left){
                  left = k;
                }
                // cout << right << " " << left << endl;
              }
            }
          }
          for(int j = 0; j < final_horizontal.size(); j++){
            if(final_horizontal[j][right] == i){
              vector<int> point;
              point.push_back(j);
              point.push_back(right);
              // cout << j << " " << right << endl;
              horizontal_ends.push_back(point);
            }
            if(final_horizontal[j][left] == i){
              vector<int> point;
              point.push_back(j);
              point.push_back(left);
              // cout << j << " " << left << endl;
              horizontal_ends.push_back(point);
            }
          }
        }
        // cout << "After horizontal_ends " << horizontal_ends.size() << endl;
        // cout << "Before horizontal_unconnected" << endl;
        // vector < vector<int> > horizontal_unconnected;
        // for(int k = 0; k < final_horizontal[0].size(); k++){
        //   for(int j = 0; j < final_horizontal.size()-1; j++){
        //     if(final_horizontal[j][k] > 0 and (laser_grid_connections[j][k][4] <= 1 or laser_grid_connections[j+1][k][1] <= 1) and final_horizontal[j+1][k] > 0){
        //       vector<int> point;
        //       point.push_back(j);
        //       point.push_back(k);
        //       cout << j << " " << k << " ";
        //       for(int l = 0; l < laser_grid_connections[j][k].size(); l++){
        //         cout << laser_grid_connections[j][k][l] << ",";
        //       }
        //       cout << endl;
        //       horizontal_unconnected.push_back(point);
        //     }
        //   }
        //   for(int j = 1; j < final_horizontal.size(); j++){
        //     if(final_horizontal[j][k] > 0 and (laser_grid_connections[j][k][5] <= 1 or laser_grid_connections[j-1][k][0] <= 1) and final_horizontal[j-1][k] > 0){
        //       vector<int> point;
        //       point.push_back(j);
        //       point.push_back(k);
        //       cout << j << " " << k << " ";
        //       for(int l = 0; l < laser_grid_connections[j][k].size(); l++){
        //         cout << laser_grid_connections[j][k][l] << ",";
        //       }
        //       cout << endl;
        //       horizontal_unconnected.push_back(point);
        //     }
        //   }
        // }
        // cout << "After horizontal_unconnected " << horizontal_unconnected.size() << endl;
        vector < vector < pair<int, int> > > verticals;
        vector < vector <int> > vertical_lengths;
        for(int j = 0; j < passage_grid[0].size(); j++){
          vector < pair<int, int> > col;
          vector <int> col_lengths;
          int start = -1;
          int stop = -1;
          for(int i = 0; i < passage_grid.size(); i++){
            if(passage_grid[i][j] >= 0 and start == -1){
              start = i;
              stop = i;
            }
            else if(passage_grid[i][j] >= 0 and start > -1){
              stop = i;
            }
            else if(passage_grid[i][j] < 0 and start > -1 and stop > -1){
              // cout << "col " << i << " start " << start << " stop " << stop << " length " << stop-start+1 << endl;
              col.push_back(make_pair(start, stop));
              col_lengths.push_back(stop-start+1);
              start = -1;
              stop = -1;
            }
          }
          verticals.push_back(col);
          vertical_lengths.push_back(col_lengths);
        }
        vector< vector<int> > vertical_passages;
        vector< vector<int> > vertical_passages_filled;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(-1);
          }
          vertical_passages.push_back(col);
          vertical_passages_filled.push_back(col);
        }

        for(int i = 0; i < verticals.size(); i++){
          for(int j = 0; j < verticals[i].size(); j++){
            if(vertical_lengths[i][j] >= min_passage_length){
              for(int k = verticals[i][j].first; k <= verticals[i][j].second; k++){
                vertical_passages[k][i] = 1;
                vertical_passages_filled[k][i] = 1;
              }
            }
          }
        }
        // cout << "After vertical_passages" << endl;
        // for(int i = 0; i < vertical_passages.size(); i++){
        //   for(int j = 0; j < vertical_passages[0].size(); j++){
        //     cout << vertical_passages[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int i = 0; i < passage_grid.size(); i++){
        //   for(int j = 0; j < passage_grid[i].size(); j++){
        //     if(passage_grid[i][j] >= 0 and vertical_passages[i][j] == -1){
        //       if(j > 0 and j < passage_grid[i].size()-1){
        //         if(vertical_passages[i][j-1] >= 0 or vertical_passages[i][j+1] >= 0){
        //           vertical_passages_filled[i][j] = 1;
        //         }
        //       }
        //       else if(j == 0){
        //         if(vertical_passages[i][j+1] >= 0){
        //           vertical_passages_filled[i][j] = 1;
        //         }
        //       }
        //       else if(j == passage_grid[i].size()-1){
        //         if(vertical_passages[i][j-1] >= 0){
        //           vertical_passages_filled[i][j] = 1;
        //         }
        //       }
        //     }
        //   }
        // }
        // cout << "After vertical_passages_filled" << endl;
        // for(int i = 0; i < vertical_passages_filled.size(); i++){
        //   for(int j = 0; j < vertical_passages_filled[0].size(); j++){
        //     cout << vertical_passages_filled[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        row_count = vertical_passages_filled.size();
        col_count = vertical_passages_filled[0].size();
        vector< vector<int> > final_vertical;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(0);
          }
          final_vertical.push_back(col);
        }
        int vertical_component = 0;
        for(int i = 0; i < row_count; ++i){
          for(int j = 0; j < col_count; ++j){
            if(final_vertical[i][j] == 0 && vertical_passages_filled[i][j] >= 0){
              agentState->dfs(i, j, ++vertical_component, dx, dy, row_count, col_count, &final_vertical, &vertical_passages_filled);
            }
          }
        }
        // cout << "final vertical_component " << vertical_component << endl;
        // cout << "After final_vertical" << endl;
        // for(int i = 0; i < final_vertical.size(); i++){
        //   for(int j = 0; j < final_vertical[0].size(); j++){
        //     cout << final_vertical[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int i = 0; i < final_vertical.size(); i++){
        //   for(int j = 0; j < final_vertical[0].size(); j++){
        //     if(final_vertical[i][j] > 0){
        //       cout << i << " " << j << endl;
        //     }
        //   }
        // }
        vector < vector<int> > vertical_ends;
        for(int i = 1; i <= vertical_component; i++){
          // cout << "vertical_component " << i << endl;
          int bottom = -1;
          int top = final_vertical[0].size();
          // cout << bottom << " " << top << endl;
          for(int j = 0; j < final_vertical.size(); j++){
            for(int k = 0; k < final_vertical[j].size(); k++){
              if(final_vertical[j][k] == i){
                if(j > bottom){
                  bottom = j;
                }
                if(j < top){
                  top = j;
                }
                // cout << bottom << " " << top << endl;
              }
            }
          }
          for(int k = 0; k < final_vertical[0].size(); k++){
            if(final_vertical[bottom][k] == i){
              vector<int> point;
              point.push_back(bottom);
              point.push_back(k);
              // cout << bottom << " " << k << endl;
              vertical_ends.push_back(point);
            }
            if(final_vertical[top][k] == i){
              vector<int> point;
              point.push_back(top);
              point.push_back(k);
              // cout << top << " " << k << endl;
              vertical_ends.push_back(point);
            }
          }
        }
        // cout << "After vertical_ends " << vertical_ends.size() << endl;
        // cout << "Before vertical_unconnected" << endl;
        // vector < vector<int> > vertical_unconnected;
        // for(int j = 0; j < final_vertical.size(); j++){
        //   for(int k = 0; k < final_vertical[j].size()-1; k++){
        //     if(final_vertical[j][k] > 0 and (laser_grid_connections[j][k][7] <= 1 or laser_grid_connections[j][k+1][2] <= 1) and final_vertical[j][k+1] > 0){
        //       vector<int> point;
        //       point.push_back(j);
        //       point.push_back(k);
        //       cout << j << " " << k << " ";
        //       for(int l = 0; l < laser_grid_connections[j][k].size(); l++){
        //         cout << laser_grid_connections[j][k][l] << ",";
        //       }
        //       cout << endl;
        //       vertical_unconnected.push_back(point);
        //     }
        //   }
        //   for(int k = 1; k < final_vertical[j].size(); k++){
        //     if(final_vertical[j][k] > 0 and (laser_grid_connections[j][k][6] <= 1 or laser_grid_connections[j][k-1][3] <= 1) and final_vertical[j][k-1] > 0){
        //       vector<int> point;
        //       point.push_back(j);
        //       point.push_back(k);
        //       cout << j << " " << k << " ";
        //       for(int l = 0; l < laser_grid_connections[j][k].size(); l++){
        //         cout << laser_grid_connections[j][k][l] << ",";
        //       }
        //       cout << endl;
        //       vertical_unconnected.push_back(point);
        //     }
        //   }
        // }
        // cout << "After vertical_unconnected " << vertical_unconnected.size() << endl;
        vector < vector<int> > final_combined;
        for(int i = 0; i < final_horizontal.size(); i++){
          vector<int> row;
          for(int j = 0; j < final_horizontal[i].size(); j++){
            if(final_horizontal[i][j] > 0 and final_vertical[i][j] > 0){
              row.push_back(1);
            }
            else{
              row.push_back(-1);
            }
          }
          final_combined.push_back(row);
        }
        // cout << "After final_combined" << endl;
        // for(int i = 0; i < final_combined.size(); i++){
        //   for(int j = 0; j < final_combined[0].size(); j++){
        //     cout << final_combined[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        for(int i = 0; i < horizontal_ends.size(); i++){
          final_combined[horizontal_ends[i][0]][horizontal_ends[i][1]] = 1;
        }
        for(int i = 0; i < vertical_ends.size(); i++){
          final_combined[vertical_ends[i][0]][vertical_ends[i][1]] = 1;
        }
        // cout << "After ends" << endl;
        // for(int i = 0; i < final_combined.size(); i++){
        //   for(int j = 0; j < final_combined[0].size(); j++){
        //     cout << final_combined[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int i = 0; i < horizontal_unconnected.size(); i++){
        //   final_combined[horizontal_unconnected[i][0]][horizontal_unconnected[i][1]] = 1;
        // }
        // for(int i = 0; i < vertical_unconnected.size(); i++){
        //   final_combined[vertical_unconnected[i][0]][vertical_unconnected[i][1]] = 1;
        // }
        // cout << "After unconnected" << endl;
        // for(int i = 0; i < final_combined.size(); i++){
        //   for(int j = 0; j < final_combined[0].size(); j++){
        //     cout << final_combined[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // dx.clear();
        // dx.push_back(1);
        // dx.push_back(0);
        // dx.push_back(-1);
        // dx.push_back(0);
        // dy.clear();
        // dy.push_back(0);
        // dy.push_back(1);
        // dy.push_back(0);
        // dy.push_back(-1);
        row_count = final_combined.size();
        col_count = final_combined[0].size();
        vector< vector<int> > intersections;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(0);
          }
          intersections.push_back(col);
        }
        int intersection_component = 0;
        for(int i = 0; i < row_count; ++i){
          for(int j = 0; j < col_count; ++j){
            if(intersections[i][j] == 0 && final_combined[i][j] >= 0){
              agentState->dfs(i, j, ++intersection_component, dx, dy, row_count, col_count, &intersections, &final_combined);
            }
          }
        }
        // cout << "final intersection_component " << intersection_component << endl;
        // cout << "After intersections" << endl;
        // for(int i = 0; i < intersections.size(); i++){
        //   for(int j = 0; j < intersections[0].size(); j++){
        //     cout << intersections[i][j] << " ";
        //   }
        //   cout << endl;
        // }

        vector < vector<int> > passages_without_intersections;
        for(int i = 0; i < final_horizontal.size(); i++){
          vector<int> row;
          for(int j = 0; j < final_horizontal[i].size(); j++){
            if(intersections[i][j] == 0 and (final_horizontal[i][j] > 0 or final_vertical[i][j] > 0)){
              row.push_back(1);
            }
            else{
              row.push_back(-1);
            }
          }
          passages_without_intersections.push_back(row);
        }
        // cout << "After passages_without_intersections" << endl;
        // for(int i = 0; i < passages_without_intersections.size(); i++){
        //   for(int j = 0; j < passages_without_intersections[0].size(); j++){
        //     cout << passages_without_intersections[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        row_count = passages_without_intersections.size();
        col_count = passages_without_intersections[0].size();
        vector< vector<int> > pass_wo_int;
        for(int i = 0; i < highway_grid.size(); i++){
          vector<int> col;
          for(int j = 0; j < highway_grid[i].size(); j++){
            col.push_back(0);
          }
          pass_wo_int.push_back(col);
        }
        int passage_component = 0;
        for(int i = 0; i < row_count; ++i){
          for(int j = 0; j < col_count; ++j){
            if(pass_wo_int[i][j] == 0 && passages_without_intersections[i][j] >= 0){
              agentState->dfs(i, j, ++passage_component, dx, dy, row_count, col_count, &pass_wo_int, &passages_without_intersections);
            }
          }
        }
        // cout << "final passage_component " << passage_component << endl;
        // cout << "After pass_wo_int" << endl;
        // for(int i = 0; i < pass_wo_int.size(); i++){
        //   for(int j = 0; j < pass_wo_int[0].size(); j++){
        //     cout << pass_wo_int[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // for(int k = 1; k <= passage_component; k++){
        //   int minx = 100000;
        //   int maxx = -1;
        //   int miny = 100000;
        //   int maxy = -1;
        //   for(int i = 0; i < pass_wo_int.size(); i++){
        //     for(int j = 0; j < pass_wo_int[0].size(); j++){
        //       if(pass_wo_int[i][j] == k){
        //         if(i < minx)
        //           minx = i;
        //         if(i > maxx)
        //           maxx = i;
        //         if(j < miny)
        //           miny = j;
        //         if(j > maxy)
        //           maxy = j;
        //       }
        //     }
        //   }
        //   cout << "passage " << k << " width " << (maxx - minx +1) << " height " << (maxy - miny +1) << endl;
        // }

        int new_ind = intersection_component;
        for(int i = 0; i < pass_wo_int.size(); i++){
          for(int j = 0; j < pass_wo_int[i].size(); j++){
            if(pass_wo_int[i][j] > 0 and intersections[i][j] == 0){
              intersections[i][j] = pass_wo_int[i][j] + new_ind;
            }
          }
        }
        // cout << "After intersections + pass_wo_int" << endl;
        // for(int i = 0; i < intersections.size(); i++){
        //   for(int j = 0; j < intersections[0].size(); j++){
        //     cout << intersections[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        int max_component = intersection_component + passage_component;
        // cout << "max_component " << max_component << endl;

        vector< vector<int> > edges;
        for(int i = 0; i < intersections.size(); i++){
          int start = -1;
          int stop = -1;
          for(int j = 0; j < intersections[0].size(); j++){
            if(intersections[i][j] > 0 and start == -1){
              start = intersections[i][j];
            }
            else if((intersections[i][j] > 0 or intersections[i][j] == -1) and start > -1 and (stop == -1 or stop == start)){
              stop = intersections[i][j];
            }
            else if(intersections[i][j] == 0){
              start = -1;
              stop = -1;
            }
            if(start > -1 and stop > -1){
              vector<int> current_pair;
              if(start < stop){
                current_pair.push_back(start);
                current_pair.push_back(stop);
              }
              else{
                current_pair.push_back(stop);
                current_pair.push_back(start);
              }
              if(start != stop and find(edges.begin(), edges.end(), current_pair) == edges.end()){
                edges.push_back(current_pair);
              }
              start = stop;
              stop = -1;
            }
          }
        }
        for(int j = 0; j < intersections[0].size(); j++){
          int start = -1;
          int stop = -1;
          for(int i = 0; i < intersections.size(); i++){
            if(intersections[i][j] > 0 and start == -1){
              start = intersections[i][j];
            }
            else if((intersections[i][j] > 0 or intersections[i][j] == -1) and start > -1 and (stop == -1 or stop == start)){
              stop = intersections[i][j];
            }
            else if(intersections[i][j] == 0){
              start = -1;
              stop = -1;
            }
            if(start > -1 and stop > -1){
              vector<int> current_pair;
              if(start < stop){
                current_pair.push_back(start);
                current_pair.push_back(stop);
              }
              else{
                current_pair.push_back(stop);
                current_pair.push_back(start);
              }
              if(start != stop and find(edges.begin(), edges.end(), current_pair) == edges.end()){
                edges.push_back(current_pair);
              }
              start = stop;
              stop = -1;
            }
          }
        }
        // cout << "After edges" << endl;
        // for(int i = 0; i < edges.size(); i++){
        //   cout << "edge " << i << " connecting " << edges[i][0] << " " << edges[i][1] << endl;
        // }
        vector< vector<int> > graph;
        vector<int> graph_edges;
        vector<int> one_sided_edges;
        // cout << "graph_edges" << endl;
        for(int i = new_ind+1; i < max_component+1; i++){
          graph_edges.push_back(i);
          // cout << i << endl;
        }
        for(int i = 0; i < graph_edges.size(); i++){
          vector< vector<int> > matches;
          for(int j = 0; j < edges.size(); j++){
            if(edges[j][1] == graph_edges[i]){
              matches.push_back(edges[j]);
            }
          }
          if(matches.size() < 2){
            one_sided_edges.push_back(graph_edges[i]);
          }
          else{
            for(int j = 0; j < matches.size()-1; j++){
              for(int k = j+1; k < matches.size(); k++){
                vector<int> seq;
                seq.push_back(matches[j][0]);
                seq.push_back(matches[j][1]);
                seq.push_back(matches[k][0]);
                graph.push_back(seq);
              }
            }
          }
        }
        // cout << "after graph" << endl;
        // for(int i = 0; i < graph.size(); i++){
        //   cout << graph[i][0] << " " << graph[i][1] << " " << graph[i][2] << endl;
        // }
        // cout << "one_sided_edges " << one_sided_edges.size() << endl;
        for(int i = 0; i < one_sided_edges.size(); i++){
          // cout << one_sided_edges[i] << endl;
          for(int j = 0; j < intersections.size(); j++){
            for(int k = 0; k < intersections[0].size(); k++){
              if(intersections[j][k] == one_sided_edges[i]){
                intersections[j][k] = 0;
              }
            }
          }
        }
        // cout << "After removed one sided edges" << endl;
        // for(int i = 0; i < intersections.size(); i++){
        //   for(int j = 0; j < intersections[0].size(); j++){
        //     cout << intersections[i][j] << " ";
        //   }
        //   cout << endl;
        // }
        // cout << "checking if graph is connected" << endl;
        vector< set<int> > components;
        for(int j = 0; j < graph.size(); j++){
          // cout << j << " " << graph[j][0] << " " << graph[j][1] << " " << graph[j][2] << endl;
          set<int> neighbor_nodes;
          // neighbor_nodes.insert(graph[j][0]);
          // cout << "Neighbor nodes " << neighbor_nodes.size() << " Graph Nodes " << numNodes() << " Added " << nodes[0]->getID() << endl;
          vector<int> nbrs;
          nbrs.push_back(graph[j][0]);
          while(nbrs.size()>0){
            int node_id = nbrs[0];
            neighbor_nodes.insert(node_id);
            // cout << "Neighbor nodes " << neighbor_nodes.size() << " Added " << node_id << endl;
            vector<int> new_nbrs;
            for(int i = 0; i < graph.size(); i++){
              if(graph[i][0] == node_id){
                new_nbrs.push_back(graph[i][2]);
                // cout << "neighbor " << graph[i][2] << endl;
              }
              if(graph[i][2] == node_id){
                new_nbrs.push_back(graph[i][0]);
                // cout << "neighbor " << graph[i][0] << endl;
              }
            }
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
          // cout << "neighbor_nodes " << neighbor_nodes.size() << endl;
          if(new_ind == neighbor_nodes.size()){
            // cout << "entire graph one component" << endl;
            components.push_back(neighbor_nodes);
            break;
          }
          else if(neighbor_nodes.size() < new_ind){
            // cout << "insert component if not already on list" << endl;
            bool found = false;
            for(int i = 0; i < components.size(); i++){
              for(set<int>::iterator it = components[i].begin(); it != components[i].end(); it++){
                if((*it) == graph[j][0]){
                  found = true;
                  break;
                }
              }
              if(found == true){
                break;
              }
            }
            if(found == false){
              // cout << "not found, adding to components" << endl;
              components.push_back(neighbor_nodes);
            }
          }
        }
        // cout << "components " << components.size() << endl;
        set<int> smallest_component;
        set<int> smallest_component_edges;
        int smallest_component_size;
        if(components.size() > 1){
          int biggest_component;
          int biggest_component_size = 0;
          for(int i = 0; i < components.size(); i++){
            // cout << "components " << i << " " << components[i].size() << endl;
            if(components[i].size() > biggest_component_size){
              biggest_component_size = components[i].size();
              biggest_component = i;
            }
          }
          // cout << "biggest_component " << biggest_component << " biggest_component_size " << biggest_component_size << endl;
          for(int i = 0; i < components.size(); i++){
            // cout << "components " << i << " " << components[i].size() << endl;
            if(i != biggest_component){
              for(set<int>::iterator it = components[i].begin(); it != components[i].end(); it++){
                // cout << "added to smallest_component " << (*it) << endl;
                smallest_component.insert((*it));
              }
            }
          }
          smallest_component_size = smallest_component.size();
          // cout << "smallest_component_size " << smallest_component_size << endl;
          // cout << "smallest_component " << smallest_component.size() << endl;
          for(int i = 0; i < graph.size(); i++){
            // cout << graph[i][0] << " " << graph[i][1] << " " << graph[i][2] << endl;
            if(smallest_component.find(graph[i][0]) != smallest_component.end()){
              smallest_component_edges.insert(graph[i][1]);
            }
            if(smallest_component.find(graph[i][2]) != smallest_component.end()){
              smallest_component_edges.insert(graph[i][2]);
            }
          }
          // cout << "smallest_component_edges " << smallest_component_edges.size() << endl;
        }

        for(int i = 0; i < intersections.size(); i++){
          for(int j = 0; j < intersections[0].size(); j++){
            if(intersections[i][j] < new_ind+1 and intersections[i][j] > 0){
              if(smallest_component.find(intersections[i][j]) != smallest_component.end()){
                intersections[i][j] = 0;
              }
            }
            else if(intersections[i][j] >= new_ind+1){
              if(smallest_component_edges.find(intersections[i][j]) != smallest_component_edges.end()){
                intersections[i][j] = 0;
              }
            }
          }
        }
        // cout << "After removed smallest components" << endl;
        // for(int i = 0; i < intersections.size(); i++){
        //   for(int j = 0; j < intersections[0].size(); j++){
        //     cout << intersections[i][j] << " ";
        //   }
        //   cout << endl;
        // }

        // cout << "before missing_labels" << endl;
        vector<int> missing_labels;
        for(int i = 1; i <= max_component; i++){
          bool found = false;
          for(int j = 0; j < intersections.size(); j++){
            for(int k = 0; k < intersections[0].size(); k++){
              if(intersections[j][k] == i){
                found = true;
                break;
              }
            }
            if(found == true){
              break;
            }
          }
          if(found == false){
            // cout << "missing " << i << endl;
            missing_labels.push_back(i);
          }
        }
        // cout << "missing_labels " << missing_labels.size() << " new_ind " << new_ind << endl;
        if(missing_labels.size() > 0){
          for(int i = missing_labels.size()-1; i >= 0; i--){
            int current_ind = missing_labels[i];
            // cout << "current_ind " << current_ind << endl;
            for(int j = 0; j < intersections.size(); j++){
              for(int k = 0; k < intersections[0].size(); k++){
                if(intersections[j][k] > current_ind){
                  intersections[j][k] = intersections[j][k] - 1;
                }
              }
            }
            if(current_ind <= new_ind){
              new_ind = new_ind - 1;
            }
            max_component = max_component - 1;
          }
          // cout << "new_ind " << new_ind << endl;
          // cout << "After fixed missing_labels" << endl;
          // for(int i = 0; i < intersections.size(); i++){
          //   for(int j = 0; j < intersections[0].size(); j++){
          //     cout << intersections[i][j] << " ";
          //   }
          //   cout << endl;
          // }
          vector< vector<int> > fixed_edges;
          for(int i = 0; i < intersections.size(); i++){
            int start = -1;
            int stop = -1;
            for(int j = 0; j < intersections[0].size(); j++){
              if(intersections[i][j] > 0 and start == -1){
                start = intersections[i][j];
              }
              else if((intersections[i][j] > 0 or intersections[i][j] == -1) and start > -1 and (stop == -1 or stop == start)){
                stop = intersections[i][j];
              }
              else if(intersections[i][j] == 0){
                start = -1;
                stop = -1;
              }
              if(start > -1 and stop > -1){
                vector<int> current_pair;
                if(start < stop){
                  current_pair.push_back(start);
                  current_pair.push_back(stop);
                }
                else{
                  current_pair.push_back(stop);
                  current_pair.push_back(start);
                }
                if(start != stop and find(fixed_edges.begin(), fixed_edges.end(), current_pair) == fixed_edges.end()){
                  fixed_edges.push_back(current_pair);
                }
                start = stop;
                stop = -1;
              }
            }
          }
          for(int j = 0; j < intersections[0].size(); j++){
            int start = -1;
            int stop = -1;
            for(int i = 0; i < intersections.size(); i++){
              if(intersections[i][j] > 0 and start == -1){
                start = intersections[i][j];
              }
              else if((intersections[i][j] > 0 or intersections[i][j] == -1) and start > -1 and (stop == -1 or stop == start)){
                stop = intersections[i][j];
              }
              else if(intersections[i][j] == 0){
                start = -1;
                stop = -1;
              }
              if(start > -1 and stop > -1){
                vector<int> current_pair;
                if(start < stop){
                  current_pair.push_back(start);
                  current_pair.push_back(stop);
                }
                else{
                  current_pair.push_back(stop);
                  current_pair.push_back(start);
                }
                if(start != stop and find(fixed_edges.begin(), fixed_edges.end(), current_pair) == fixed_edges.end()){
                  fixed_edges.push_back(current_pair);
                }
                start = stop;
                stop = -1;
              }
            }
          }
          vector< vector<int> > fixed_graph;
          vector<int> fixed_graph_edges;
          for(int i = new_ind+1; i < max_component+1; i++){
            fixed_graph_edges.push_back(i);
            // cout << i << endl;
          }
          for(int i = 0; i < fixed_graph_edges.size(); i++){
            vector< vector<int> > matches;
            for(int j = 0; j < fixed_edges.size(); j++){
              if(fixed_edges[j][1] == fixed_graph_edges[i]){
                matches.push_back(fixed_edges[j]);
              }
            }
            for(int j = 0; j < matches.size()-1; j++){
              for(int k = j+1; k < matches.size(); k++){
                vector<int> seq;
                seq.push_back(matches[j][0]);
                seq.push_back(matches[j][1]);
                seq.push_back(matches[k][0]);
                fixed_graph.push_back(seq);
              }
            }
          }
          graph = fixed_graph;
          // cout << "after fixed graph" << endl;
          // for(int i = 0; i < graph.size(); i++){
          //   cout << graph[i][0] << " " << graph[i][1] << " " << graph[i][2] << endl;
          // }
        }

        // cout << "graph_nodes" << endl;
        for(int i = 1; i < new_ind+1; i++){
          graph_nodes.insert(make_pair(i, vector< vector<int> >()));
          node_steps.insert(make_pair(i, vector<int>()));
          // cout << i << endl;
        }
        // cout << "graph_edges_map" << endl;
        for(int i = new_ind+1; i < max_component+1; i++){
          graph_edges_map.insert(make_pair(i, vector< vector<int> >()));
          // cout << i << endl;
        }

        for(int i = 0; i < intersections.size(); i++){
          for(int j = 0; j < intersections[0].size(); j++){
            if(intersections[i][j] < new_ind+1 and intersections[i][j] > 0){
              vector<int> current_grid;
              current_grid.push_back(i);
              current_grid.push_back(j);
              graph_nodes[intersections[i][j]].push_back(current_grid);
              // cout << "node " << intersections[i][j] << " point " << i << " " << j << endl;
            }
            else if(intersections[i][j] >= new_ind+1){
              vector<int> current_grid;
              current_grid.push_back(i);
              current_grid.push_back(j);
              graph_edges_map[intersections[i][j]].push_back(current_grid);
              // cout << "edge " << intersections[i][j] << " point " << i << " " << j << endl;
            }
          }
        }
        // cout << "graph_nodes " << graph_nodes.size() << " graph_edges_map " << graph_edges_map.size() << endl;
        passages_grid = intersections;
        cout << "final passages_grid" << endl;
        for(int i = 0; i < passages_grid.size(); i++){
          for(int j = 0; j < passages_grid[0].size(); j++){
            cout << passages_grid[i][j] << " ";
          }
          cout << endl;
        }
        
        int index_val = 0;
        for(int i = 1; i < new_ind+1; i++){
          vector< vector<int> > points = graph_nodes[i];
          double x = 0, y = 0;
          for(int j = 0; j < points.size(); j++){
            x += points[j][0]*2;
            y += points[j][1]*2;
            x += (points[j][0]+1)*2;
            y += (points[j][1]+1)*2;
          }
          x = x / (points.size()*4);
          y = y / (points.size()*4);
          // cout << "node " << i << " index_val " << index_val << " x " << x << " y " << y << " node_x " << (int)(x*100) << " node_y " << (int)(y*100) << endl;
          vector<int> avg_psg;
          avg_psg.push_back((int)(x*100));
          avg_psg.push_back((int)(y*100));
          average_passage.push_back(avg_psg);
        }
        for(int i = new_ind+1; i < max_component+1; i++){
          vector< vector<int> > points = graph_edges_map[i];
          double x = 0, y = 0;
          for(int j = 0; j < points.size(); j++){
            x += points[j][0]*2;
            y += points[j][1]*2;
            x += (points[j][0]+1)*2;
            y += (points[j][1]+1)*2;
          }
          x = x / (points.size()*4);
          y = y / (points.size()*4);
          // cout << "edge " << i << " index_val " << index_val << " x " << x << " y " << y << " edge_x " << (int)(x*100) << " edge_y " << (int)(y*100) << endl;
          vector<int> avg_psg;
          avg_psg.push_back((int)(x*100));
          avg_psg.push_back((int)(y*100));
          average_passage.push_back(avg_psg);
          index_val++;
        }
        // cout << "graph_nodes " << graph_nodes.size() << " graph_edges_map " << graph_edges_map.size() << endl;
        // for(int k = 0; k < graph.size(); k++){
        //   cout << graph[k][0] << " " << graph[k][1] << " " << graph[k][2] << endl;
        //   for(int i = 0; i < intersections.size(); i++){
        //     for(int j = 0; j < intersections[0].size(); j++){
        //       if(intersections[i][j] == graph[k][0] or intersections[i][j] == graph[k][2]){
        //         cout << intersections[i][j] << " " << i << " " << j << endl;
        //         vector<int> current_grid;
        //         current_grid.push_back(i);
        //         current_grid.push_back(j);
        //         graph_edges_map[graph[k][1]].push_back(current_grid);
        //       }
        //     }
        //   }
        //   // vector< vector<int> > intersection1_points = graph_nodes[graph[i][0]];
        //   // for(int j = 0; j < intersection1_points.size(); j++){
        //   //   graph_edges_map[graph[i][1]].push_back(intersection1_points[j]);
        //   // }
        //   // vector< vector<int> > intersection2_points = graph_nodes[graph[i][1]];
        //   // for(int j = 0; j < intersection2_points.size(); j++){
        //   //   graph_edges_map[graph[i][1]].push_back(intersection2_points[j]);
        //   // }
        // }
        // cout << "graph_nodes " << graph_nodes.size() << " graph_edges_map " << graph_edges_map.size() << endl;
        reduced_graph = graph;
    }

    void learnPassageTrails(vector<CartesianPoint> stepped_history, vector < vector<CartesianPoint> > stepped_laser_history) {
        // cout << "creating trails between intersections" << endl;
        vector<double> dist_between_steps;
        for(int k = 0; k < stepped_history.size(); k++){
          double start_x = stepped_history[k].get_x();
          if(int(start_x) < 0)
            start_x = 0;
          if(int(start_x) >= decisions_grid.size())
            start_x = decisions_grid.size()-1;
          double start_y = stepped_history[k].get_y();
          if(int(start_y) < 0)
            start_y = 0;
          if(int(start_y) >= decisions_grid[0].size())
            start_y = decisions_grid[0].size()-1;
          map<int, vector< vector<int> > >::iterator it;
          for(it = graph_nodes.begin(); it != graph_nodes.end(); it++){
            vector< vector<int> > points = it->second;
            for(int i = 0; i < points.size(); i++){
              if(((int)(start_x) == points[i][0] and (int)(start_y) == points[i][1])){
                node_steps[it->first].push_back(k);
              }
            }
          }
          if(k != stepped_history.size()-1){
            dist_between_steps.push_back(stepped_history[k].get_distance(stepped_history[k+1]));
          }
        }
        dist_between_steps.push_back(0.0);
        for(map<int, vector<int> >::iterator it = node_steps.begin(); it != node_steps.end(); it++){
          // cout << "intersection " << it->first << " " << (it->second).size() << endl;
          double dist_limit = 0.25;
          while((it->second).size() == 0){
            dist_limit = dist_limit + 0.25;
            for(int k = 0; k < stepped_history.size(); k++){
              vector< vector<int> > points = graph_nodes[it->first];
              for(int i = 0; i < points.size(); i++){
                if(stepped_history[k].get_distance(CartesianPoint((double)(points[i][0])+0.5, (double)(points[i][1])+0.5)) < dist_limit){
                  (it->second).push_back(k);
                }
              }
            }
          }
        }
        // cout << "found points associated with each intersection" << endl;
        
        // vector<bool> reverse_order;
        for(int i = 0; i < reduced_graph.size(); i++){
          // cout << i << " " << reduced_graph[i][0] << " " << reduced_graph[i][1] << " " << reduced_graph[i][2] << endl;
          vector<int> intersection1_points = node_steps[reduced_graph[i][0]];
          vector<int> intersection2_points = node_steps[reduced_graph[i][2]];
          vector< vector<int> > passage12_points = graph_edges_map[reduced_graph[i][1]];
          // cout << "intersection1_points " << intersection1_points.size() << " intersection2_points " << intersection2_points.size() << endl;
          vector<int> start_ind;
          vector<int> end_ind;
          vector<int> length_path;
          vector<bool> need_to_reverse;
          for(int j = 0; j < intersection1_points.size(); j++){
            for(int k = 0; k < intersection2_points.size(); k++){
              if(intersection1_points[j] < intersection2_points[k]){
                length_path.push_back(intersection2_points[k] - intersection1_points[j]);
                start_ind.push_back(intersection1_points[j]);
                end_ind.push_back(intersection2_points[k]);
                need_to_reverse.push_back(false);
              }
              else{
                length_path.push_back(intersection1_points[j] - intersection2_points[k]);
                start_ind.push_back(intersection2_points[k]);
                end_ind.push_back(intersection1_points[j]);
                need_to_reverse.push_back(true);
              }
            }
          }
          // cout << "length_path " << length_path.size() << " start_ind " << start_ind.size() << " end_ind " << end_ind.size() << " need_to_reverse " << need_to_reverse.size() << endl;
          vector<CartesianPoint> finalTrail;
          int pathID = -1;
          double min_length = 1000000;
          double max_overlap = 0;
          for(int j = 0; j < start_ind.size(); j++){
          //   vector<CartesianPoint> trailPositions;
          //   trailPositions.push_back(stepped_history[start_ind[j]]);
          //   for(int k = start_ind[j]; k < end_ind[j]; k++){
          //     for(int n = end_ind[j]; n > k; n--){
          //       if(canAccessPoint(stepped_laser_history[k], stepped_history[k], stepped_history[n], 3.5)) {
          //         trailPositions.push_back(stepped_history[n]);
          //         k = n-1;
          //       }
          //     }
          //   }
          //   trailPositions.push_back(stepped_history[end_ind[j]]);
          //   cout << "trailPositions " << trailPositions.size() << endl;
          //   if(need_to_reverse[j]){
          //     // cout << "reversing order of trailPositions" << endl;
          //     reverse(trailPositions.begin(),trailPositions.end());
          //   }
          //   double trailLength = 0;
          //   for(int k = 0; k < trailPositions.size()-1; k++){
          //     trailLength += trailPositions[k].get_distance(trailPositions[k+1]);
          //   }
          //   double count_overlap = 0;
          //   for(int k = 0; k < trailPositions.size(); k++){
          //     for(int n = 0; n < passage12_points.size(); n++){
          //       if(passage12_points[n][0] == (int)(trailPositions[k].get_x()) and passage12_points[n][1] == (int)(trailPositions[k].get_y())){
          //         count_overlap = count_overlap + 1;
          //       }
          //     }
          //   }
          //   cout << "trailLength " << trailLength << " count_overlap / trailPositions.size() " << count_overlap / trailPositions.size() << endl;
          //   if(trailLength < min_length and count_overlap / trailPositions.size() >= 0.6){
          //     min_length = trailLength;
          //     finalTrail = trailPositions;
          //   }
            double trailLength = 0;
            double count_overlap = 0;
            for(int n = 0; n < passage12_points.size(); n++){
              for(int k = start_ind[j]; k < end_ind[j]; k++){
                if(passage12_points[n][0] == (int)(stepped_history[k].get_x()) and passage12_points[n][1] == (int)(stepped_history[k].get_y())){
                  count_overlap = count_overlap + 1;
                  break;
                }
              }
            }
            for(int k = start_ind[j]; k < end_ind[j]; k++){
              trailLength = trailLength + dist_between_steps[k];
            }
            // cout << "trailLength " << trailLength << " count_overlap " << count_overlap << " passage12_points.size() " << passage12_points.size() << endl;
            if(trailLength < min_length and count_overlap / passage12_points.size() >= max_overlap and count_overlap / passage12_points.size() >= 0.6){
              min_length = trailLength;
              max_overlap = count_overlap / passage12_points.size();
              pathID = j;
            }
          }
          // cout << "pathID " << pathID << endl;
          if(pathID > -1){
            finalTrail.push_back(stepped_history[start_ind[pathID]]);
            for(int k = start_ind[pathID]; k < end_ind[pathID]; k++){
              for(int n = end_ind[pathID]; n > k; n--){
                if(canAccessPoint(stepped_laser_history[k], stepped_history[k], stepped_history[n], 3.5)) {
                  finalTrail.push_back(stepped_history[n]);
                  k = n-1;
                }
              }
            }
            finalTrail.push_back(stepped_history[end_ind[pathID]]);
            if(need_to_reverse[pathID]){
              // cout << "reversing order of finalTrail" << endl;
              reverse(finalTrail.begin(),finalTrail.end());
            }
            // cout << "finalTrail " << finalTrail.size() << endl;
          }
          bool addedTrail = false;
          if(finalTrail.size() > 0){
            graph_trails.push_back(finalTrail);
            addedTrail = true;
          }
          else{
            vector<int> match_x;
            vector<int> match_x_dist;
            vector<int> match_y;
            vector<int> match_y_dist;
            for(int j = 0; j < intersection1_points.size(); j++){
              for(int k = 0; k < intersection2_points.size(); k++){
                if((int)(stepped_history[intersection1_points[j]].get_x()) == (int)(stepped_history[intersection2_points[k]].get_x())){
                  // cout << "match_x " << (int)(stepped_history[intersection1_points[j]].get_x()) << " match_x_dist " << abs((int)(stepped_history[intersection1_points[j]].get_y()) - (int)(stepped_history[intersection2_points[k]].get_y())) << endl;
                  match_x.push_back((int)(stepped_history[intersection1_points[j]].get_x()));
                  match_x_dist.push_back(abs((int)(stepped_history[intersection1_points[j]].get_y()) - (int)(stepped_history[intersection2_points[k]].get_y())));
                }
                if((int)(stepped_history[intersection1_points[j]].get_y()) == (int)(stepped_history[intersection2_points[k]].get_y())){
                  // cout << "match_y " << (int)(stepped_history[intersection1_points[j]].get_y()) << " match_y_dist " << abs((int)(stepped_history[intersection1_points[j]].get_x()) - (int)(stepped_history[intersection2_points[k]].get_x())) << endl;
                  match_y.push_back((int)(stepped_history[intersection1_points[j]].get_y()));
                  match_y_dist.push_back(abs((int)(stepped_history[intersection1_points[j]].get_x()) - (int)(stepped_history[intersection2_points[k]].get_x())));
                }
              }
            }
            // cout << "match_x " << match_x.size() << " match_y " << match_y.size() << endl;
            if(match_x.size() > 0 and match_y.size() == 0){
              for(int j = 0; j < match_x.size(); j++){
                vector<CartesianPoint> passagePath;
                for(int k = 0; k < passage12_points.size(); k++){
                  if(passage12_points[k][0] == match_x[j]){
                    passagePath.push_back(CartesianPoint(passage12_points[k][0], passage12_points[k][1]));
                  }
                }
                // cout << "passagePath " << passagePath.size() << " match_x_dist " << match_x_dist[j] << " difference " << abs((int)(passagePath.size()) - match_x_dist[j]) << endl;
                if(abs((int)(passagePath.size()) - match_x_dist[j]) <= 2 and passagePath.size() > 0){
                  sort(passagePath.begin(), passagePath.end());
                  if(stepped_history[intersection1_points[0]].get_distance(passagePath[0]) > stepped_history[intersection1_points[0]].get_distance(passagePath[passagePath.size()-1])){
                    reverse(passagePath.begin(),passagePath.end());
                  }
                  graph_trails.push_back(passagePath);
                  addedTrail = true;
                  break;
                }
              }
            }
            else if(match_x.size() == 0 and match_y.size() > 0){
              for(int j = 0; j < match_y.size(); j++){
                vector<CartesianPoint> passagePath;
                for(int k = 0; k < passage12_points.size(); k++){
                  if(passage12_points[k][1] == match_y[j]){
                    passagePath.push_back(CartesianPoint(passage12_points[k][0], passage12_points[k][1]));
                  }
                }
                // cout << "passagePath " << passagePath.size() << " match_y_dist " << match_y_dist[j] << " difference " << abs((int)(passagePath.size()) - match_y_dist[j]) << endl;
                if(abs((int)(passagePath.size()) - match_y_dist[j]) <= 2 and passagePath.size() > 0){
                  sort(passagePath.begin(), passagePath.end());
                  if(stepped_history[intersection1_points[0]].get_distance(passagePath[0]) > stepped_history[intersection1_points[0]].get_distance(passagePath[passagePath.size()-1])){
                    reverse(passagePath.begin(),passagePath.end());
                  }
                  graph_trails.push_back(passagePath);
                  addedTrail = true;
                  break;
                }
              }
            }
            else if(match_x.size() > 0 and match_y.size() > 0){
              for(int j = 0; j < match_x.size(); j++){
                vector<CartesianPoint> passagePath;
                for(int k = 0; k < passage12_points.size(); k++){
                  if(passage12_points[k][0] == match_x[j]){
                    passagePath.push_back(CartesianPoint(passage12_points[k][0], passage12_points[k][1]));
                  }
                }
                // cout << "passagePath " << passagePath.size() << " match_x_dist " << match_x_dist[j] << " difference " << abs((int)(passagePath.size()) - match_x_dist[j]) << endl;
                if(abs((int)(passagePath.size()) - match_x_dist[j]) <= 2 and passagePath.size() > 0){
                  sort(passagePath.begin(), passagePath.end());
                  if(stepped_history[intersection1_points[0]].get_distance(passagePath[0]) > stepped_history[intersection1_points[0]].get_distance(passagePath[passagePath.size()-1])){
                    reverse(passagePath.begin(),passagePath.end());
                  }
                  graph_trails.push_back(passagePath);
                  addedTrail = true;
                  break;
                }
              }
              if(addedTrail == false){
                for(int j = 0; j < match_y.size(); j++){
                  vector<CartesianPoint> passagePath;
                  for(int k = 0; k < passage12_points.size(); k++){
                    if(passage12_points[k][1] == match_y[j]){
                      passagePath.push_back(CartesianPoint(passage12_points[k][0], passage12_points[k][1]));
                    }
                  }
                  // cout << "passagePath " << passagePath.size() << " match_y_dist " << match_y_dist[j] << " difference " << abs((int)(passagePath.size()) - match_y_dist[j]) << endl;
                  if(abs((int)(passagePath.size()) - match_y_dist[j]) <= 2 and passagePath.size() > 0){
                    sort(passagePath.begin(), passagePath.end());
                    if(stepped_history[intersection1_points[0]].get_distance(passagePath[0]) > stepped_history[intersection1_points[0]].get_distance(passagePath[passagePath.size()-1])){
                      reverse(passagePath.begin(),passagePath.end());
                    }
                    graph_trails.push_back(passagePath);
                    addedTrail = true;
                    break;
                  }
                }
              }
            }
          }
          // cout << "addedTrail " << addedTrail << endl;
          if(addedTrail == false){
            vector<CartesianPoint> closestPoints;
            double minDistBetween = 1000000;
            for(int j = 0; j < intersection1_points.size(); j++){
              for(int k = 0; k < intersection2_points.size(); k++){
                double distBetween = stepped_history[intersection1_points[j]].get_distance(stepped_history[intersection2_points[k]]);
                if(distBetween < minDistBetween){
                  minDistBetween = distBetween;
                  closestPoints.clear();
                  closestPoints.push_back(stepped_history[intersection1_points[j]]);
                  closestPoints.push_back(stepped_history[intersection2_points[k]]);
                }
              }
            }
            // cout << "closestPoints " << closestPoints.size() << endl;
            graph_trails.push_back(closestPoints);
          }
        }
        // cout << "found trails between intersections, check if trail follows passage" << endl;
        // for(int i = 0; i < reduced_graph.size(); i++){
        //   // cout << i << " " << reduced_graph[i][0] << " " << reduced_graph[i][1] << " " << reduced_graph[i][2] << endl;
        //   vector<CartesianPoint> trailPositions = graph_trails[i];
        //   vector< vector<int> > points = graph_edges_map[reduced_graph[i][1]];
        //   // cout << "trailPositions " << trailPositions.size() << " passage points " << points.size() << endl;
        //   double count_overlap = 0;
        //   for(int j = 0; j < trailPositions.size(); j++){
        //     for(int k = 0; k < points.size(); k++){
        //       if(points[k][0] == (int)(trailPositions[j].get_x()) and points[k][1] == (int)(trailPositions[j].get_y())){
        //         count_overlap = count_overlap + 1;
        //       }
        //     }
        //   }
        //   // cout << "overlap " << overlap << endl;
        //   if(count_overlap / trailPositions.size() < 0.75){
        //     vector<int> intersection1_points = node_steps[reduced_graph[i][0]];
        //     vector<int> intersection2_points = node_steps[reduced_graph[i][2]];
        //     // cout << "intersection1_points " << intersection1_points.size() << " intersection2_points " << intersection2_points.size() << endl;
        //     bool new_path_found = false;
        //     double min_start_x, min_start_y, min_end_x, min_end_y;
        //     double min_start_end_dist = 1000000;
        //     for(int j = 0; j < intersection1_points.size(); j++){
        //       for(int k = 0; k < intersection2_points.size(); k++){
        //         double start_x = stepped_history[intersection1_points[j]].get_x();
        //         if(int(start_x) < 0)
        //           start_x = 0;
        //         if(int(start_x) >= decisions_grid.size())
        //           start_x = decisions_grid.size()-1;
        //         double start_y = stepped_history[intersection1_points[j]].get_y();
        //         if(int(start_y) < 0)
        //           start_y = 0;
        //         if(int(start_y) >= decisions_grid[0].size())
        //           start_y = decisions_grid[0].size()-1;
        //         double end_x = stepped_history[intersection2_points[k]].get_x();
        //         if(int(end_x) < 0)
        //           end_x = 0;
        //         if(int(end_x) >= decisions_grid.size())
        //           end_x = decisions_grid.size()-1;
        //         double end_y = stepped_history[intersection2_points[k]].get_y();
        //         if(int(end_y) < 0)
        //           end_y = 0;
        //         if(int(end_y) >= decisions_grid[0].size())
        //           end_y = decisions_grid[0].size()-1;
        //         // cout << start_x << " " << start_y << " " << end_x << " " << end_y << endl;
        //         double start_end_dist = stepped_history[intersection1_points[j]].get_distance(stepped_history[intersection2_points[k]]);
        //         if((int)(start_x) == (int)(end_x) or (int)(start_y) == (int)(end_y)){
        //           if(canAccessPoint(stepped_laser_history[intersection1_points[j]], stepped_history[intersection1_points[j]], stepped_history[intersection2_points[k]], 25) or canAccessPoint(stepped_laser_history[intersection2_points[k]], stepped_history[intersection2_points[k]], stepped_history[intersection1_points[j]], 25)){
        //             // cout << "found laser that connects" << endl;
        //             vector<CartesianPoint> new_trailPositions;
        //             if(((int)(start_x)) == ((int)(end_x))){
        //               // cout << "x values same, iterate over y values " << ((int)(start_y)) << " " << ((int)(end_y)) << endl;
        //               int start_val, end_val;
        //               bool reverse = false;
        //               if(((int)(start_y)) < ((int)(end_y))){
        //                 start_val = ((int)(start_y));
        //                 end_val = ((int)(end_y));
        //               }
        //               else{
        //                 end_val = ((int)(start_y));
        //                 start_val = ((int)(end_y));
        //                 reverse = true;
        //               }
        //               if(reverse == false){
        //                 new_trailPositions.push_back(CartesianPoint(start_x, start_y));
        //                 for(int l = start_val; l <= end_val; l++){
        //                   if(canAccessPoint(stepped_laser_history[intersection1_points[j]], stepped_history[intersection1_points[j]], CartesianPoint(start_x, l), 25) or canAccessPoint(stepped_laser_history[intersection2_points[k]], stepped_history[intersection2_points[k]], CartesianPoint(start_x, l), 25)){
        //                     new_trailPositions.push_back(CartesianPoint(start_x, l));
        //                   }
        //                 }
        //                 new_trailPositions.push_back(CartesianPoint(end_x, end_y));
        //               }
        //               else{
        //                 new_trailPositions.push_back(CartesianPoint(end_x, end_y));
        //                 for(int l = start_val; l <= end_val; l++){
        //                   if(canAccessPoint(stepped_laser_history[intersection1_points[j]], stepped_history[intersection1_points[j]], CartesianPoint(start_x, l), 25) or canAccessPoint(stepped_laser_history[intersection2_points[k]], stepped_history[intersection2_points[k]], CartesianPoint(start_x, l), 25)){
        //                     new_trailPositions.push_back(CartesianPoint(start_x, l));
        //                   }
        //                 }
        //                 new_trailPositions.push_back(CartesianPoint(start_x, start_y));
        //                 std::reverse(new_trailPositions.begin(), new_trailPositions.end());
        //               }
        //             }
        //             else if(((int)(start_y)) == ((int)(end_y))){
        //               // cout << "y values same, iterate over x values " << ((int)(start_x)) << " " << ((int)(end_x)) << endl;
        //               int start_val, end_val;
        //               bool reverse = false;
        //               if(((int)(start_x)) < ((int)(end_x))){
        //                 start_val = ((int)(start_x));
        //                 end_val = ((int)(end_x));
        //               }
        //               else{
        //                 end_val = ((int)(start_x));
        //                 start_val = ((int)(end_x));
        //                 reverse = true;
        //               }
        //               if(reverse == false){
        //                 new_trailPositions.push_back(CartesianPoint(start_x, start_y));
        //                 for(int l = start_val; l <= end_val; l++){
        //                   if(canAccessPoint(stepped_laser_history[intersection1_points[j]], stepped_history[intersection1_points[j]], CartesianPoint(l, start_y), 25) or canAccessPoint(stepped_laser_history[intersection2_points[k]], stepped_history[intersection2_points[k]], CartesianPoint(l, start_y), 25)){
        //                     new_trailPositions.push_back(CartesianPoint(l, start_y));
        //                   }
        //                 }
        //                 new_trailPositions.push_back(CartesianPoint(end_x, end_y));
        //               }
        //               else{
        //                 new_trailPositions.push_back(CartesianPoint(end_x, end_y));
        //                 for(int l = start_val; l <= end_val; l++){
        //                   if(canAccessPoint(stepped_laser_history[intersection1_points[j]], stepped_history[intersection1_points[j]], CartesianPoint(l, start_y), 25) or canAccessPoint(stepped_laser_history[intersection2_points[k]], stepped_history[intersection2_points[k]], CartesianPoint(l, start_y), 25)){
        //                     new_trailPositions.push_back(CartesianPoint(l, start_y));
        //                   }
        //                 }
        //                 new_trailPositions.push_back(CartesianPoint(start_x, start_y));
        //                 std::reverse(new_trailPositions.begin(), new_trailPositions.end());
        //               }
        //             }
        //             graph_trails[i] = new_trailPositions;
        //             new_path_found = true;
        //             break;
        //           }
        //           else if(start_end_dist > 25 and start_end_dist < min_start_end_dist){
        //             min_start_end_dist = start_end_dist;
        //             min_start_x = start_x;
        //             min_start_y = start_y;
        //             min_end_x = end_x;
        //             min_end_y = end_y;
        //           }
        //         }
        //       }
        //       if(new_path_found == true){
        //         break;
        //       }
        //     }
        //     if(new_path_found == false and min_start_end_dist < 1000000){
        //       // cout << "no direct path found, follow closest path between" << endl;
        //       vector<CartesianPoint> new_trailPositions;
        //       new_trailPositions.push_back(CartesianPoint(min_start_x, min_start_y));
        //       if(((int)(min_start_x)) == ((int)(min_end_x))){
        //         for(int l = ((int)(min_start_y)); l <= ((int)(min_end_y)); l++){
        //           new_trailPositions.push_back(CartesianPoint(min_start_x, l));
        //         }
        //         new_trailPositions.push_back(CartesianPoint(min_end_x, min_end_y));
        //       }
        //       else{
        //         for(int l = ((int)(min_start_x)); l <= ((int)(min_end_x)); l++){
        //           new_trailPositions.push_back(CartesianPoint(l, min_start_y));
        //         }
        //         new_trailPositions.push_back(CartesianPoint(min_end_x, min_end_y));
        //       }
        //       graph_trails[i] = new_trailPositions;
        //     }
        //     // cout << "new_path_found " << new_path_found << " trail size " << graph_trails[i].size() << endl;
        //   }
        // }
        // cout << "graph_trails " << graph_trails.size() << endl;
        // for(int i = 0; i < graph_trails.size(); i++){
        //   cout << "  " << i << " " << reduced_graph[i][0] << " " << reduced_graph[i][1] << " " << reduced_graph[i][2] << endl;
        //   for(int j = 0; j < graph_trails[i].size(); j++){
        //     cout << graph_trails[i][j].get_x() << " " << graph_trails[i][j].get_y() << endl;
        //   } 
        // }
        // cout << "find trail between intersection points" << endl;
        vector< vector<int> > graph_by_edges;
        for(int i = 0; i < reduced_graph.size(); i++){
          vector<int> pair;
          pair.push_back(reduced_graph[i][1]);
          pair.push_back(reduced_graph[i][0]);
          graph_by_edges.push_back(pair);
          pair.clear();
          pair.push_back(reduced_graph[i][1]);
          pair.push_back(reduced_graph[i][2]);
          graph_by_edges.push_back(pair);
        }
        
        for(int i = 0; i < graph_by_edges.size(); i++){
          for(int j = 0; j < reduced_graph.size(); j++){
            if(reduced_graph[j][1] != graph_by_edges[i][0] and reduced_graph[j][0] == graph_by_edges[i][1] and graph_by_edges[i][0] < reduced_graph[j][1]){
              vector<int> connection = graph_by_edges[i];
              connection.push_back(reduced_graph[j][1]);
              graph_through_intersections.push_back(connection);
            }
            if(reduced_graph[j][1] != graph_by_edges[i][0] and reduced_graph[j][2] == graph_by_edges[i][1] and graph_by_edges[i][0] < reduced_graph[j][1]){
              vector<int> connection = graph_by_edges[i];
              connection.push_back(reduced_graph[j][1]);
              graph_through_intersections.push_back(connection);
            }
          }
        }
        
        // for(int i = 0; i < graph_through_intersections.size(); i++){
        //   // cout << i << " " << graph_through_intersections[i][0] << " " << graph_through_intersections[i][1] << " " << graph_through_intersections[i][2] << endl;
        //   CartesianPoint start;
        //   CartesianPoint end;
        //   bool found_start = false;
        //   bool found_end = false;
        //   for(int k = 0; k < graph_trails.size(); k++){
        //     // cout << k << " " << reduced_graph[k][0] << " " << reduced_graph[k][1] << " " << reduced_graph[k][2] << endl;
        //     if(graph_through_intersections[i][0] == reduced_graph[k][1] and graph_through_intersections[i][1] == reduced_graph[k][0]){
        //       start = graph_trails[k][0];
        //       // cout << "found start " << start.get_x() << " " << start.get_y() << endl;
        //       found_start = true;
        //     }
        //     else if(graph_through_intersections[i][0] == reduced_graph[k][1] and graph_through_intersections[i][1] == reduced_graph[k][2]){
        //       start = graph_trails[k][graph_trails[k].size()-1];
        //       // cout << "found start " << start.get_x() << " " << start.get_y() << endl;
        //       found_start = true;
        //     }
        //     if(graph_through_intersections[i][2] == reduced_graph[k][1] and graph_through_intersections[i][1] == reduced_graph[k][0]){
        //       end = graph_trails[k][0];
        //       // cout << "found end " << end.get_x() << " " << end.get_y() << endl;
        //       found_end = true;
        //     }
        //     else if(graph_through_intersections[i][2] == reduced_graph[k][1] and graph_through_intersections[i][1] == reduced_graph[k][2]){
        //       end = graph_trails[k][graph_trails[k].size()-1];
        //       // cout << "found end " << end.get_x() << " " << end.get_y() << endl;
        //       found_end = true;
        //     }
        //     if(found_start and found_end){
        //       break;
        //     }
        //   }
        //   vector<CartesianPoint> trailPositions;
        //   if(start == end){
        //     // cout << "start and end the same, add one" << endl;
        //     trailPositions.push_back(start);
        //     graph_intersection_trails.push_back(trailPositions);
        //   }
        //   else{
        //     // cout << "start and end different" << endl;
        //     vector<int> intersection_points = node_steps[graph_through_intersections[i][1]];
        //     // cout << "intersection_points " << intersection_points.size() << endl;
        //     int start_ind = -1;
        //     int end_ind = -1;
        //     for(int j = 0; j < intersection_points.size(); j++){
        //       if(stepped_history[intersection_points[j]] == start){
        //         start_ind = intersection_points[j];
        //       }
        //       if(stepped_history[intersection_points[j]] == end){
        //         end_ind = intersection_points[j];
        //       }
        //       if(start_ind != -1 and end_ind != -1){
        //         break;
        //       }
        //     }
        //     // cout << "start_ind " << start_ind << " end_ind " << end_ind << endl;
        //     if(canAccessPoint(stepped_laser_history[start_ind], stepped_history[start_ind], stepped_history[end_ind], 25) or canAccessPoint(stepped_laser_history[end_ind], stepped_history[end_ind], stepped_history[start_ind], 25)){
        //       // cout << "points can see each other, add both" << endl;
        //       trailPositions.push_back(start);
        //       trailPositions.push_back(end);
        //       graph_intersection_trails.push_back(trailPositions);
        //     }
        //     else{
        //       // cout << "points cannot see each other, find in sequence between" << endl;
        //       vector<int> in_between_inds;
        //       bool found_each_other = false;
        //       vector<int> all_sees_start;
        //       for(int j = 0; j < intersection_points.size(); j++){
        //         if(canAccessPoint(stepped_laser_history[intersection_points[j]], stepped_history[intersection_points[j]], stepped_history[start_ind], 25) or canAccessPoint(stepped_laser_history[start_ind], stepped_history[start_ind], stepped_history[intersection_points[j]], 25)){
        //           all_sees_start.push_back(intersection_points[j]);
        //         }
        //       }
        //       // cout << "all_sees_start " << all_sees_start.size() << endl;
        //       while(found_each_other == false){
        //         double min_dist_to_end = 1000000;
        //         int between_ind = -1;
        //         for(int j = 0; j < all_sees_start.size(); j++){
        //           if(canAccessPoint(stepped_laser_history[all_sees_start[j]], stepped_history[all_sees_start[j]], stepped_history[end_ind], 25) or canAccessPoint(stepped_laser_history[end_ind], stepped_history[end_ind], stepped_history[all_sees_start[j]], 25)){
        //             // cout << "found something that sees end " << all_sees_start[j] << endl;
        //             in_between_inds.push_back(all_sees_start[j]);
        //             found_each_other = true;
        //             break;
        //           }
        //           double dist_to_end = stepped_history[end_ind].get_distance(stepped_history[all_sees_start[j]]);
        //           if(dist_to_end < min_dist_to_end and find(in_between_inds.begin(), in_between_inds.end(), all_sees_start[j]) == in_between_inds.end()){
        //             min_dist_to_end = dist_to_end;
        //             between_ind = all_sees_start[j];
        //           }
        //         }
        //         if(!found_each_other){
        //           // cout << "added closest to end " << between_ind << " min_dist_to_end " << min_dist_to_end << endl;
        //           in_between_inds.push_back(between_ind);
        //           all_sees_start.clear();
        //           for(int j = 0; j < intersection_points.size(); j++){
        //             if(canAccessPoint(stepped_laser_history[intersection_points[j]], stepped_history[intersection_points[j]], stepped_history[between_ind], 25) or canAccessPoint(stepped_laser_history[between_ind], stepped_history[between_ind], stepped_history[intersection_points[j]], 25)){
        //               all_sees_start.push_back(intersection_points[j]);
        //             }
        //           }
        //         }
        //       }
        //       // cout << "finished finding in between points" << endl;
        //       trailPositions.push_back(start);
        //       for(int j = 0; j < in_between_inds.size(); j++){
        //         trailPositions.push_back(stepped_history[in_between_inds[j]]);
        //       }
        //       trailPositions.push_back(end);
        //       graph_intersection_trails.push_back(trailPositions);
        //     }
        //   }
        // }
        // cout << "graph_intersection_trails " << graph_intersection_trails.size() << endl;
        // for(int i = 0; i < graph_intersection_trails.size(); i++){
        //   cout << "  " << i << " " << graph_through_intersections[i][0] << " " << graph_through_intersections[i][1] << " " << graph_through_intersections[i][2] << endl;
        //   for(int j = 0; j < graph_intersection_trails[i].size(); j++){
        //     cout << graph_intersection_trails[i][j].get_x() << " " << graph_intersection_trails[i][j].get_y() << endl;
        //   }
        // }
    }

private:
    vector< vector<int> > highway_grid;
    AgentState* agentState;
    vector< vector<int> > passages_grid;
    vector< vector<int> > decisions_grid;
    // vector< vector<int> > decisions_only_grid;
    map<int, vector< vector<int> > > graph_nodes;
    map<int, vector< vector<int> > > graph_edges_map;
    map<int, vector<int> > node_steps;
    vector< vector<int> > reduced_graph;
    vector< vector<int> > average_passage;
    vector< vector<CartesianPoint> > graph_trails;
    vector< vector<int> > graph_through_intersections;
    vector< vector<CartesianPoint> > graph_intersection_trails;
};

#endif