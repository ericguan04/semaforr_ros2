/************************************************
FORRHallways.h 
This file contains the class which contains information about the hallways that FORR learns and uses

Written by Raj Korpan, adapted from Sarah Mathew and Gil Dekel, 2018
**********************************************/

#ifndef FORRHALLWAYS_H
#define FORRHALLWAYS_H

#include <AgentState.h>
#include <FORRGeometry.h>
#include <Aggregate.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>

using namespace std;

//=========================================================//=========================================================//

struct Segment {
public:
  Segment(CartesianPoint left, CartesianPoint right, double step){
    left_point_ = left;
    right_point_ = right;
    angle_ = FindAngle();
    section_ = FindSection(step);
  }

  Segment(CartesianPoint left, CartesianPoint right, vector <CartesianPoint> leftLaser, vector <CartesianPoint> rightLaser, double step){
    left_point_ = left;
    right_point_ = right;
    angle_ = FindAngle();
    section_ = FindSection(step);
    left_laser_ = leftLaser;
    right_laser_ = rightLaser;
    segment_data_.push_back(left.get_x());
    segment_data_.push_back(left.get_y());
    segment_data_.push_back(right.get_x());
    segment_data_.push_back(right.get_y());
    segment_data_.push_back(angle_);
  }


  void PrintSegment() {
    cout << left_point_.get_x() << " " << left_point_.get_y() << " " << right_point_.get_x() << " " << right_point_.get_y() << " " << angle_ << endl;
  }

  CartesianPoint GetLeftPoint() const {return left_point_;}
  CartesianPoint GetRightPoint() const {return right_point_;}
  double GetAngle() const {return angle_;}
  int GetSection() const {return section_;}
  vector <CartesianPoint> GetLeftLaser() const {return left_laser_;}
  vector <CartesianPoint> GetRightLaser() const {return right_laser_;}
  vector<double> getSegmentData() const {return segment_data_;};

private:
  CartesianPoint left_point_;
  CartesianPoint right_point_;
  double angle_;
  int section_;
  vector <CartesianPoint> left_laser_;
  vector <CartesianPoint> right_laser_;
  vector<double> segment_data_;

  double FindAngle() {
    //double angle = atan2((left_point_.get_y() - right_point_.get_y()),(left_point_.get_x() - right_point_.get_x()))+M_PI;
    //angle = (angle < (2*M_PI) ? angle : (0.0));
    CartesianPoint coordinate = TranslateSegment();
    double angle = atan2(coordinate.get_y(), coordinate.get_x());
    angle = MakeAnglePositive(angle);
    return angle;
  }
  CartesianPoint TranslateSegment(const CartesianPoint reference_point = CartesianPoint(0,0)){
    double new_x = right_point_.get_x() - (left_point_.get_x()- reference_point.get_x());
    double new_y = right_point_.get_y() - (left_point_.get_y()- reference_point.get_y());
    CartesianPoint new_coordinate = CartesianPoint(new_x, new_y);
    return new_coordinate;
  }
  double MakeAnglePositive(double angle) {
    if(angle < 0)
      angle += M_PI;
    return angle;
  }
  int FindSection(double step) {
    double angle_fix = angle_ * 180/M_PI;
    int section = -1;
    if(angle_fix <= step and angle_fix >= 0){
      section = 0;
    }
    else if(angle_fix <= 45+step and angle_fix >= 45-step){
      section = 1;
    }
    else if(angle_fix <= 90+step and angle_fix >= 90-step){
      section = 2;
    }
    else if(angle_fix <= 135+step and angle_fix >= 135-step){
      section = 3;
    }
    else if(angle_fix <= 180 and angle_fix >= 180-step){
      section = 0;
    }
    return section;
  }

};


//=========================================================//=========================================================//

/* FORRHallways class
 *
 * FORRHallways represent the hallways that are learned by SemaFORR.
 *
 */
class FORRHallways{
public:
    FORRHallways(double wid, double hgt){
        hallways = vector<Aggregate>();
        initial_hallways = vector<Aggregate>();
        map_width_ = wid;
        map_height_ = hgt;
        threshold = 0.7;
        step = 22.5;
        hallway_names.push_back("horizontal");
        hallway_names.push_back("minor_diagonal");
        hallway_names.push_back("vertical");
        hallway_names.push_back("major_diagonal");
        hallway_sections.push_back(vector<Segment>());
        hallway_sections.push_back(vector<Segment>());
        hallway_sections.push_back(vector<Segment>());
        hallway_sections.push_back(vector<Segment>());
    };
    vector<Aggregate> getHallways(){return hallways;}
    void setHallways(vector< vector<CartesianPoint> > hlws, vector<int> idvals){
      for(int i = 0; i < hlws.size(); i++){
        initial_hallways.push_back(Aggregate(hlws[i], idvals[i]));
      }
      hallways = initial_hallways;
    }
    int getWidth(){return map_width_;}
    int getHeight(){return map_height_;}
    ~FORRHallways(){};

    void clearAllHallways(){
        hallways.clear();
    }

    void learnHallways(AgentState *agentState, vector<CartesianPoint> trails_trace, vector< vector<CartesianPoint> > *laser_hist) {
        vector<CartesianPoint> new_trails_coordinates;
        vector<vector<CartesianPoint> > new_laser_history;

        for(int i = 0; i < trails_trace.size(); i++) {
          trails_coordinates.push_back(trails_trace[i]);
          new_trails_coordinates.push_back(trails_trace[i]);
        }
        for(int i = 0 ; i < laser_hist->size() ; i++){
          laser_history.push_back((*laser_hist)[i]);
          new_laser_history.push_back((*laser_hist)[i]);
        }
        agent_state = agentState;
        
        vector<Segment> trails_segments;
        CreateSegments(trails_segments, new_trails_coordinates, new_laser_history);
        cout << "num of segments " << trails_segments.size() << endl;

        // vector<vector<Segment> > hallway_sections(4);
        for(int i = 0; i < trails_segments.size(); i++) {
          hallway_sections[trails_segments[i].GetSection()].push_back(trails_segments[i]);
          // double angle = trails_segments[i].GetAngle() * 180/M_PI;
          // if(angle <= step and angle >= 0){
          //   //cout << "Section 0 = " << trails_segments[i].GetAngle() << endl;
          //   hallway_sections[0].push_back(trails_segments[i]);
          //   //trails_segments[i].PrintSegment();
          // }
          // else if(angle <= 45+step and angle >= 45-step){
          //   //cout << "Section 1 = " << trails_segments[i].GetAngle() << endl;
          //   hallway_sections[1].push_back(trails_segments[i]);
          // }
          // else if(angle <= 90+step and angle >= 90-step){
          //   //cout << "Section 2 = " << trails_segments[i].GetAngle() << endl;
          //   hallway_sections[2].push_back(trails_segments[i]);
          // }
          // else if(angle <= 135+step and angle >= 135-step){
          //   //cout << "Section 3 = " << trails_segments[i].GetAngle() << endl;
          //   hallway_sections[3].push_back(trails_segments[i]);
          // }
          // else if(angle <= 180 and angle >= 180-step){
          //   //cout << "Section 0 = " << trails_segments[i].GetAngle() << endl;
          //   hallway_sections[0].push_back(trails_segments[i]);
          //   //trails_segments[i].PrintSegment();
          // }
        }
        
        vector<Aggregate> all_aggregates;
        for(int i = 0; i < hallway_sections.size(); i++) {
          cout << "num of segments in hallway section " << hallway_sections[i].size() << endl;
          if(hallway_sections[i].size() > 0){

            //vector<vector<double> > segments_normalized(hallway_sections[i].size(), vector<double>(5,0));
            //NormalizeVector(segments_normalized, hallway_sections[i]);
            //cout << "num of segments normalized " << segments_normalized.size() << endl;

            vector<vector<double> > segments_data;
            ConvertSegmentsToDouble(segments_data, hallway_sections[i]);

            vector<vector<double> > segments_similarities;
            ListSimilarities(segments_similarities, segments_data);
            //ListSimilarities(segments_similarities, segments_normalized);
            cout << "num of segments similarities " << segments_similarities.size() << endl;

            vector<vector<double> > most_similar_segments;
            FindMostSimilarSegments(most_similar_segments, segments_similarities);
            cout << "num of most similar segments " << most_similar_segments.size() << endl;

            vector<Segment> mean_segments;
            CreateMeanSegments(mean_segments, most_similar_segments, hallway_sections[i], step);
            cout << "num of mean_segments " << mean_segments.size() << endl;

            vector<vector<CartesianPoint> > initial_hallway_groups = ProcessHallwayData(mean_segments, map_width_, map_height_, threshold);
            if(initial_hallways.size() > 0){
              for(int j = 0; j < initial_hallways.size(); j++){
                if(initial_hallways[j].getHallwayType() == i){
                  initial_hallway_groups.push_back(initial_hallways[j].getPoints());
                }
              }
            }
            if(initial_hallway_groups.size()>0){
              /*cout << "Initial Aggregates" << endl;
              for(int j = 0; j < initial_hallway_groups.size(); j++){
                for(int k = 0; k < initial_hallway_groups[j].size(); k++){
                  cout << initial_hallway_groups[j][k].get_x() << " " << initial_hallway_groups[j][k].get_y() << " ";
                }
                cout << ";";
              }
              cout << endl;*/
              vector<vector<CartesianPoint> > merged_hallway_groups = MergeNearbyHallways(initial_hallway_groups, trails_coordinates, laser_history, i, step, map_width_, map_height_, threshold);
              // vector<vector<CartesianPoint> > hallway_groups = FillHallways(merged_hallway_groups, trails_coordinates, laser_history, i, step, map_width_, map_height_, threshold);
              /*cout << "Final Aggregates" << endl;
              for(int j = 0; j < hallway_groups.size(); j++){
                for(int k = 0; k < hallway_groups[j].size(); k++){
                  cout << hallway_groups[j][k].get_x() << " " << hallway_groups[j][k].get_y() << " ";
                }
                cout << ";";
              }
              cout << endl;*/
              //cout << "process agg" << endl;
              for(int j = 0; j< merged_hallway_groups.size(); j++) {
                  Aggregate group = Aggregate(merged_hallway_groups.at(j), i);
                  all_aggregates.push_back(group);
              }
              cout << "Num of hallways " << merged_hallway_groups.size() << endl;
              merged_hallway_groups.clear();
            }
            segments_data.clear();
            segments_similarities.clear();
            most_similar_segments.clear();
            mean_segments.clear();
            initial_hallway_groups.clear();
          }
          cout << "done proccessing " << hallway_names[i] << endl;
        }
        //cout << "finished map" << endl;
        if(all_aggregates.size() > 0){
          //cout << "finding connections between hallways" << endl;
          for (int i = 0; i < all_aggregates.size()-1; i++){
            for (int j = i + 1; j < all_aggregates.size(); j++){
              if(all_aggregates[i].getHallwayType() != all_aggregates[j].getHallwayType()){
                all_aggregates[i].findConnection(all_aggregates[j],i,j);
              }
            }
          }
          for (int i = 0; i < all_aggregates.size(); i++){
            all_aggregates[i].numConnections();
          }
        }

        hallways = all_aggregates;
    }

private:
    vector<Aggregate> hallways;
    vector<Aggregate> initial_hallways;
    vector<vector<int> > interpolate;
    AgentState *agent_state;
    vector<CartesianPoint> trails_coordinates;
    vector<vector<CartesianPoint> > laser_history;
    vector<vector<Segment> > hallway_sections;
    double threshold;
    double step;
    vector<string> hallway_names;

    int map_height_;
    int map_width_;

    void CreateSegments(vector<Segment> &segments, const vector<CartesianPoint> &trails, const vector < vector <CartesianPoint> > &laser_history);

    void NormalizeVector(vector<vector<double> > &normalized_segments, const vector<Segment> &segments);
    void ConvertSegmentsToDouble(vector<vector<double> > &data, const vector<Segment> &segments);
    double Normalize(const double original_val, const double min_val, const double max_val);
    double FindMin(const vector<vector<double> > &segments, const int pos);
    double FindMax(const vector<vector<double> > &segments, const int pos);

    void ListSimilarities(vector<vector<double> > &similarities,const vector<vector<double> > &normalized_segments);
    double ComputeDistance(const vector<double> first_segment, const vector<double> second_segment);

    void FindMostSimilarSegments(vector<vector<double> > &most_similar,const vector<vector<double> > &similarities);

    void CreateMeanSegments(vector<Segment> &averaged_segments,const vector<vector<double> > &most_similar,const vector<Segment> &segments,double step);

    vector<vector<CartesianPoint> > ProcessHallwayData(const vector<Segment> &hallway_group, int width, int height, double threshold);
    vector<vector<CartesianPoint> > MergeNearbyHallways(const vector<vector<CartesianPoint> > initial_hallway_groups, const vector<CartesianPoint> &trails, const vector < vector <CartesianPoint> > &laser_history, int hallway_type, double step, int width, int height, double threshold);
    vector<vector<CartesianPoint> > FillHallways(const vector<vector<CartesianPoint> > initial_hallway_groups, const vector<CartesianPoint> &trails, const vector < vector <CartesianPoint> > &laser_history, int hallway_type, double step, int width, int height, double threshold);
    void UpdateMap(vector<vector<double> > &frequency_map, const vector<Segment> &segments);
    void SmoothMap(vector<vector<double> > &frequency_map, const vector<vector<double> > &heat_map, double threshold);
    void Interpolate(vector<vector<double> > &frequency_map,double left_x, double left_y, double right_x, double right_y);
    void BinarizeImage(vector<vector<int> > &binarized,const vector<vector<double> > &original,  double threshold);
    //void ConvertMatrixToImage(const vector<vector<int> > &binary_map, string image_name);
    void doUnion(int x, int y, int x2, int y2, vector<vector<int> > &labeled_image);
    void unionCoords(int x, int y, int x2, int y2, const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image);
    void LabelImage(const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image);
    void ListGroups(vector<vector< pair<int,int> > > &aggregates_and_points, const vector<vector<int> > &labeled_image);
    void ConvertPairToCartesianPoint(vector<vector<CartesianPoint> > &trails, const vector<vector< pair<int,int> > > &input);
};

#endif
