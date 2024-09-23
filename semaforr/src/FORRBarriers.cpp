#include<FORRBarriers.h>

using namespace std;



//----------------------//------------------------//



void FORRBarriers::CreateSegments(vector<LineSegment> &segments, vector<CartesianPoint> position_history, vector < vector <CartesianPoint> > laser_history) {
  cout << "num of pos history " << position_history.size() << " num of laser history " << laser_history.size() << endl;
  for (int i = 0; i < laser_history.size(); i+=1){
    cout << "num of laser sensors " << laser_history[i].size() << endl;
    for (int j = 0; j < laser_history[i].size()-1; j++){
      LineSegment current_segment = LineSegment(laser_history[i][j], laser_history[i][j+1]);
      if(laser_history[i][j].get_distance(position_history[i]) <= 10 and laser_history[i][j+1].get_distance(position_history[i]) <= 10 and laser_history[i][j].get_distance(laser_history[i][j+1]) <= 0.5 and laser_history[i][j].get_distance(laser_history[i][j+1]) >= 0.1){
        segments.push_back(current_segment);
      }
    }
  }
}



//----------------------//------------------------//

void FORRBarriers::ListSimilarities(vector<vector<double> > &similarities, const vector<LineSegment> &laser_segments) {
  vector<double> segments_and_similarities;

  for(int i = 0; i < laser_segments.size() - 1; i++) {
    for(int j = i + 1; j < laser_segments.size(); j++) {
      segments_and_similarities.push_back(double(i));
      segments_and_similarities.push_back(double(j));
      //cout << i << " " << j << endl;
      segments_and_similarities.push_back(ComputeDistance(laser_segments[i], laser_segments[j]));
      if(segments_and_similarities[2] <= 0.5){
        similarities.push_back(segments_and_similarities);
      }
      segments_and_similarities.clear();
    }
  }
}

double FORRBarriers::ComputeDistance(LineSegment first_segment, LineSegment second_segment) {
  /*CartesianPoint intersection_point;
  //cout << "First = " << first_segment.get_endpoints().first << " " << first_segment.get_endpoints().second << " Second = " << second_segment.get_endpoints().first << " " << second_segment.get_endpoints().second << endl;
  double dist;
  if(do_intersect(first_segment, second_segment, intersection_point)){
    dist = 0;
    //cout << "Segments intersect dist = 0" << endl;
  }
  else{
    double first_left_dist = distance(first_segment.get_endpoints().first, second_segment);
    double first_right_dist = distance(first_segment.get_endpoints().second, second_segment);
    double second_left_dist = distance(second_segment.get_endpoints().first, first_segment);
    double second_right_dist = distance(second_segment.get_endpoints().second, first_segment);
    //cout << first_left_dist << " " << first_right_dist << " " << second_left_dist << " " << second_right_dist << endl;
    dist = min(min(first_left_dist,first_right_dist),min(second_left_dist,second_right_dist));
    //cout << "Segments don't intersect dist = " << dist << endl;
  }*/
  double first_left_dist = distance(first_segment.get_endpoints().first, second_segment);
  double first_right_dist = distance(first_segment.get_endpoints().second, second_segment);
  double second_left_dist = distance(second_segment.get_endpoints().first, first_segment);
  double second_right_dist = distance(second_segment.get_endpoints().second, first_segment);
  double dist = min(min(first_left_dist,first_right_dist),min(second_left_dist,second_right_dist));

  double angledist = atan(first_segment.get_slope()) - atan(second_segment.get_slope());
  //cout << "Initial angledist = " << angledist << endl;
  if(angledist > M_PI/2){
    angledist = angledist - M_PI;
  }
  else if(angledist < -M_PI/2){
    angledist = angledist + M_PI;
  }
  angledist = abs(angledist);
  //cout << "Final angledist = " << angledist << endl;
  double sum = (dist + angledist);
  //cout << "Sum = " << sum << endl;
  return sum;
}



//----------------------//------------------------//



void FORRBarriers::FindMostSimilarSegments(vector<vector<double> > &most_similar, const vector<vector<double> > &similarities) {
  double average_of_distances, sum_of_distances, std, sum_of_squared_differences, threshold = 0;

  /*for(int i = 0; i < similarities.size(); i++) {
    sum_of_distances += similarities[i][2]; // similarity score
  }
  average_of_distances = sum_of_distances / similarities.size();
  cout << "average_of_distances = " << average_of_distances << endl;
  for(int i = 0; i < similarities.size(); i++) {
    sum_of_squared_differences += pow((similarities[i][2]-average_of_distances), 2);
  }

  double normalized_sum_of_squared_differences = sum_of_squared_differences/(similarities.size());
  std = pow(normalized_sum_of_squared_differences, .5); // square root of squared difference sum
  cout << "std = " << std << endl;
  double deviations = 3.5;
  if(isinf(std) == false){
    while(most_similar.size() == 0 and deviations > 0){
      deviations = deviations-0.1;
      threshold = average_of_distances - (deviations*std);
      cout << "threshold " << threshold << endl;
      for(int i = 0; i < similarities.size(); i++) {
        //cout << similarities[i][2] << endl;
        if(similarities[i][2] <= threshold) {
          vector<double> similar_pairing;
          similar_pairing.push_back(similarities[i][0]);
          similar_pairing.push_back(similarities[i][1]);
          most_similar.push_back(similar_pairing);
          //cout << similarities[i][0] << " " << similarities[i][1] << " " << similarities[i][2] << endl;
          similar_pairing.clear();
        }
      }
    }
  }*/

  threshold = 0.01;
  for(int i = 0; i < similarities.size(); i++) {
    //cout << similarities[i][2] << endl;
    if(similarities[i][2] <= threshold) {
      vector<double> similar_pairing;
      similar_pairing.push_back(similarities[i][0]);
      similar_pairing.push_back(similarities[i][1]);
      most_similar.push_back(similar_pairing);
      //cout << similarities[i][0] << " " << similarities[i][1] << " " << similarities[i][2] << endl;
      similar_pairing.clear();
    }
  }
}



//----------------------//------------------------//



void FORRBarriers::CreateInitialSegments(vector<LineSegment> &initial_barriers,const vector<vector<double> > &most_similar,const vector<LineSegment> &segments) {
  vector<set<double> > all_similar;
  set<int> used_inds;
  for(int i = 0; i < most_similar.size() - 1; i++) {
    if(used_inds.count(i) == 0){
      //cout << most_similar[i][0] << " " << most_similar[i][1] << endl;
      set<double> associated_values;
      associated_values.insert(most_similar[i][0]);
      associated_values.insert(most_similar[i][1]);
      vector<double> additions;
      additions.push_back(most_similar[i][0]);
      additions.push_back(most_similar[i][1]);
      used_inds.insert(i);
      //cout << "remaining additions " << additions.size() << endl;
      while(additions.size()>0){
        for(int j = i + 1; j < most_similar.size(); j++) {
          if(used_inds.count(j) == 0){
            if(most_similar[j][0] == additions[0]){
              //cout << most_similar[j][1] << endl;
              if(associated_values.count(most_similar[j][1]) == 0){
                additions.push_back(most_similar[j][1]);
                associated_values.insert(most_similar[j][1]);
                used_inds.insert(j);
              }
            }
            if(most_similar[j][1] == additions[0]){
              //cout << most_similar[j][0] << endl;
              if(associated_values.count(most_similar[j][0]) == 0){
                additions.push_back(most_similar[j][0]);
                associated_values.insert(most_similar[j][0]);
                used_inds.insert(j);
              }
            }
          }
        }
        additions.erase(additions.begin());
        //cout << "remaining additions " << additions.size() << endl;
      }
      all_similar.push_back(associated_values);
      associated_values.clear();
      additions.clear();
    }
  }
  cout << "all_similar size " << all_similar.size() << " used_inds size " << used_inds.size() << endl;
  for(int i = 0; i < all_similar.size(); i++){
    vector<LineSegment> similar_segments;
    std::set<double>::iterator it;
    for(it = all_similar[i].begin(); it != all_similar[i].end(); it++){
      similar_segments.push_back(segments[*it]);
    }
    if(similar_segments.size() >= 10){
      cout << "num of segments to be merged " << similar_segments.size() << endl;
      initial_barriers.push_back(MergeSegments(similar_segments));
    }
    similar_segments.clear();
  }
}


//----------------------//------------------------//


LineSegment FORRBarriers::MergeSegments(vector<LineSegment> segments) {
  double num_of_segments = segments.size();
  double centroid_x, centroid_y, total_length = 0;
  vector<double> angles_of_segments;
  for(int i = 0; i < segments.size(); i++){
    //cout << segments[i].get_endpoints().first.get_x() << " " << segments[i].get_endpoints().first.get_y() << ", " << segments[i].get_endpoints().second.get_x() << " " << segments[i].get_endpoints().second.get_y() << endl;
    double length = segments[i].get_length();
    centroid_x +=  length * (segments[i].get_endpoints().first.get_x() + segments[i].get_endpoints().second.get_x());
    centroid_y +=  length * (segments[i].get_endpoints().first.get_y() + segments[i].get_endpoints().second.get_y());
    total_length += length;
    angles_of_segments.push_back(atan(segments[i].get_slope()));
    //cout << length << " " << segments[i].get_slope() << " " << atan(segments[i].get_slope()) << endl;
  }
  //cout << centroid_x << " " << centroid_y << " " << total_length << endl;
  centroid_x = centroid_x / (2.0 * total_length);
  centroid_y = centroid_y / (2.0 * total_length);
  //cout << centroid_x << " " << centroid_y << endl;
  int num_negatives, num_positives, num_verticals = 0;
  for(int i = 0; i < angles_of_segments.size(); i++){
    //cout << angles_of_segments[i] << endl;
    if(angles_of_segments[i] < 0){
      num_negatives++;
    }
    if(angles_of_segments[i] >= 0){
      num_positives++;
    }
    if(abs(angles_of_segments[i]) > M_PI/2 - 0.3927){
      num_verticals++;
    }
  }
  //cout << num_negatives << " " << num_positives << " " << num_verticals << endl;
  double centroid_angle = 0;
  if(num_negatives > 0 and num_positives > 0 and num_verticals > 0){
    for(int i = 0; i < angles_of_segments.size(); i++){
      //cout << segments[i].get_length() << " " << abs(angles_of_segments[i]) << endl;
      centroid_angle += segments[i].get_length() * abs(angles_of_segments[i]);
    }
  }
  else{
    for(int i = 0; i < angles_of_segments.size(); i++){
      //cout << segments[i].get_length() << " " << angles_of_segments[i] << endl;
      centroid_angle += segments[i].get_length() * angles_of_segments[i];
    }
  }
  //cout << centroid_angle << " " << total_length << endl;
  centroid_angle = centroid_angle / total_length;
  //cout << centroid_angle << endl;
  double centroid_slope = tan(centroid_angle);
  double centroid_b = centroid_y - centroid_slope * centroid_x;
  double perp_slope = -1.0 / centroid_slope;
  //cout << centroid_slope << " " << centroid_b << " " << perp_slope << endl;
  vector<CartesianPoint> potential_points;
  for(int i = 0; i < segments.size(); i++){
    double b1 = segments[i].get_endpoints().first.get_y() - perp_slope * segments[i].get_endpoints().first.get_x();
    double x1 = (b1 - centroid_b) / (centroid_slope - perp_slope);
    double y1 = perp_slope * x1 + b1;
    //cout << b1 << " " << x1 << " " << y1 << endl;
    potential_points.push_back(CartesianPoint(x1, y1));
    double b2 = segments[i].get_endpoints().second.get_y() - perp_slope * segments[i].get_endpoints().second.get_x();
    double x2 = (b2 - centroid_b) / (centroid_slope - perp_slope);
    double y2 = perp_slope * x2 + b2;
    //cout << b2 << " " << x2 << " " << y2 << endl;
    potential_points.push_back(CartesianPoint(x2, y2));
  }
  double max_distance = 0;
  LineSegment merged_segment;
  for(int i = 0; i < potential_points.size() - 1; i++) {
    for(int j = i + 1; j < potential_points.size(); j++) {
      double dist = potential_points[i].get_distance(potential_points[j]);
      //cout << dist << " " << potential_points[i].get_x() << " " << potential_points[i].get_y() << ", " << potential_points[j].get_x() << " " << potential_points[j].get_y() << endl;
      if(dist > max_distance){
        max_distance = dist;
        //cout << max_distance << endl;
        merged_segment = LineSegment(potential_points[i],potential_points[j]);
      }
    }
  }
  //cout << "merged_segment = (" << merged_segment.get_endpoints().first.get_x() << ", " << merged_segment.get_endpoints().first.get_y() << "), (" << merged_segment.get_endpoints().second.get_x() << ", " << merged_segment.get_endpoints().second.get_y() << ")" << endl;
  return merged_segment;
}


//----------------------//------------------------//

void FORRBarriers::MergeNearbyBarriers(vector<LineSegment> &merged_barriers, const vector<LineSegment> &initial_barriers) {
  double num_mergers = initial_barriers.size();
  vector<LineSegment> barriers_to_merge = initial_barriers;
  vector<vector<double> > segments_similarities;
  ListSimilarities(segments_similarities, barriers_to_merge);
  cout << "num of segments similarities " << segments_similarities.size() << endl;

  vector<vector<double> > most_similar_segments;
  FindMostSimilarSegments(most_similar_segments, segments_similarities);
  cout << "num of most similar segments " << most_similar_segments.size() << endl;

  vector<LineSegment> final_barriers;
  CreateInitialSegments(final_barriers, most_similar_segments, barriers_to_merge);
  //cout << "num of final_barriers " << final_barriers.size() << endl;
  barriers_to_merge = final_barriers;

  /*cout << "num_mergers " << num_mergers << endl;
  while(num_mergers > 0){
    vector<vector<double> > segments_similarities;
    ListSimilarities(segments_similarities, barriers_to_merge);
    cout << "num of segments similarities " << segments_similarities.size() << endl;

    vector<vector<double> > most_similar_segments;
    FindMostSimilarSegments(most_similar_segments, segments_similarities);
    cout << "num of most similar segments " << most_similar_segments.size() << endl;

    vector<LineSegment> final_barriers;
    CreateInitialSegments(final_barriers, most_similar_segments, barriers_to_merge);
    cout << "num of final_barriers " << final_barriers.size() << endl;
    if(num_mergers == final_barriers.size()){
      num_mergers = 0;
    }
    else{
      num_mergers = final_barriers.size();
    }
    barriers_to_merge = final_barriers;
    cout << "num_mergers " << num_mergers << endl;
  }*/
  merged_barriers = barriers_to_merge;
}