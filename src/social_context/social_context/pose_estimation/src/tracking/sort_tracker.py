"""
This module implements the SORT (Simple Online and Realtime Tracking) algorithm for tracking multiple objects
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from .kalman_tracker import KalmanPersonTracker

class SortTracker():
    def __init__(self, distance_threshold = 2.0, missed_threshold = 15, tentative_missed_threshold = 7, hits_threshold = 3, combined_cost_threshold = 0.8, missed_grace_multiplier = 1.35):
        """
        Initialize the SORT tracker

        Args: distance_threshold: Maximum distance to associate detections to existing trackers
              missed_threshold: Number of consecutive misses before a tracker is deleted
              tentative_missed_threshold: Number of consecutive misses before a tentative tracker is deleted
              hits_threshold: Number of consecutive hits before a tracker is confirmed
              combined_cost_threshold: Maximum combined cost (distance + appearance) to associate detections to existing trackers
              missed_grace_multiplier: Widens the match radius by this factor for a tracker that has missed at least one detection
        """
        self.trackers = []
        self.distance_threshold = distance_threshold
        self.missed_threshold = missed_threshold
        self.tentative_missed_threshold = tentative_missed_threshold
        self.hits_threshold = hits_threshold
        self.combined_cost_threshold = combined_cost_threshold
        self.missed_grace_multiplier = missed_grace_multiplier
        self.frame_count = 0


    def get_confirmed_tracks(self):
        """
        Get the list of confirmed tracks

        Returns:
            List of (id, position) for confirmed tracks
        """
        confirmed_tracks = []
        for track in self.trackers:
            if track.state == 'CONFIRMED':
                confirmed_tracks.append({
                    'id': track.id, 
                    'position': track.get_position(), 
                    'history': track.history,
                    'confidence': track.confidence,
                    'orientation': track.orientation
                    })
        return confirmed_tracks


    def match_radius(self, tracker_index, threshold=None):
        """
        Effective match radius for a given tracker, widened once it's missed at
        least one detection

        Args:
            tracker_index: index into self.trackers
            threshold: base radius to widen. Defaults to self.distance_threshold.
        """
        if threshold is None:
            threshold = self.distance_threshold
        if self.trackers[tracker_index].missed >= 1:
            return threshold * self.missed_grace_multiplier
        return threshold


    def is_ambiguous(self, item1, item2, threshold=None):
        """
        Helper function
        Check if two detections or track predictions are close enough to cause matching confusion

        Args:
            item1 (detection or prediction): (x, y)
            item2 (detection or prediction): (x, y)
            threshold: distance below which matching becomes ambiguous.
                Defaults to self.distance_threshold, so anything close enough
                to be a plausible match candidate is also close enough to be
                treated as ambiguous.

        Returns:
            True if appearance features are needed
            False - no ambiguity, can match based on distance alone
        """
        if threshold is None:
            threshold = self.distance_threshold
        pos1 = np.array([item1[0], item1[1]])
        pos2 = np.array([item2[0], item2[1]])
        distance = np.linalg.norm(pos1 - pos2)
        return distance < threshold


    def get_ambiguous_items(self, detections, predicted_positions, threshold=None):
        """
        Get detections that are close enough to cause matching confusion

        Args:
            detections: list of (x, y)
            predicted_positions: list of (x, y)
            threshold: distance below which matching becomes ambiguous.
                Defaults to self.distance_threshold.

        Returns:
            Set of detection indices for ambiguous pairs
        """
        if threshold is None:
            threshold = self.distance_threshold
        ambiguous_detections = set()

        for i in range (len(detections)):
            for j in range(i+1, len(detections)):
                if self.is_ambiguous(detections[i], detections[j], threshold):
                    ambiguous_detections.add(i)
                    ambiguous_detections.add(j)

        # Adds detections that are near the ambiguous predictions
        for i in range (len(predicted_positions)):
            for j in range(i+1, len(predicted_positions)):
                if self.is_ambiguous(predicted_positions[i], predicted_positions[j], threshold):
                    for k in range(len(detections)):
                        if self.is_ambiguous(predicted_positions[i], detections[k], threshold) or self.is_ambiguous(predicted_positions[j], detections[k], threshold):
                            ambiguous_detections.add(k)

        
        # Adds detections that are near the predicted positions of trackers that have already missed a detection
        for i in range(len(predicted_positions)):
            if self.trackers[i].missed > 0:
                for k in range(len(detections)):
                    if self.is_ambiguous(predicted_positions[i], detections[k], threshold):
                        ambiguous_detections.add(k)

        return ambiguous_detections
    

    def match_positions(self, predicted_positions, positions, confidences, orientations, pixel_positions, image, feature_extractor):
        """
        Match predicted positions of existing trackers to new detections using the Hungarian algorithm

        Args:
            predicted_positions: list of (x, y) from existing trackers
            positions: list of (x, y) from new detections
            confidences: list of confidence values for the new detections
            orientations: list of orientation values for the new detections
            pixel_positions: list of (pixel_x, pixel_y) for the new detections
            image: latest camera image (numpy array) for appearance feature extraction
            feature_extractor: instance of FeatureExtractor for appearance feature extraction

        Returns:
            accepted_assignments: set of indices of detections that were matched to existing trackers
        """
        cost_matrix = np.zeros((len(predicted_positions), len(positions)))
        for i, pred in enumerate(predicted_positions):
            for j, pos in enumerate(positions):
                cost_matrix[i, j] = np.linalg.norm(np.array(pred) - np.array(pos))

        # Solve the assignment problem
        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        accepted_assignments = set()

        # Update existing trackers or create new ones
        for i, j in zip(row_indices, col_indices):
            if cost_matrix[i, j] < self.match_radius(i):
                self.trackers[i].update(
                    positions[j], confidences[j], 
                    self.hits_threshold, orientations[j])
                accepted_assignments.add(j)

                if (self.trackers[i].appearance_feature is None and image is not None and feature_extractor is not None):
                    # Extract appearance feature for the tracker if it doesn't have one yet
                    px, py = pixel_positions[j]
                    feature = feature_extractor.extract_features(image, px, py)
                    if feature is not None:
                        self.trackers[i].update_appearance_feature(feature)


            else:
                self.trackers[i].mark_missed(self.missed_threshold, self.tentative_missed_threshold)

        # mark unmatched trackers as missed
        for i in range(len(self.trackers)):
            if i not in row_indices:
                self.trackers[i].mark_missed(self.missed_threshold, self.tentative_missed_threshold)
        
        return accepted_assignments
    

    def match_appearance(self, predicted_positions, positions, confidences, orientations, ambiguous_detections, pixel_positions, image, feature_extractor):
        """
        Match predicted positions of existing trackers to new detections using appearance features for ambiguous cases

        Args:
            predicted_positions: list of (x, y) from existing trackers
            positions: list of (x, y) from new detections
            confidences: list of confidence values for the new detections
            orientations: list of orientation values for the new detections
            ambiguous_detections: set of indices of detections that are ambiguous
            pixel_positions: list of (pixel_x, pixel_y) for the new detections
            image: latest camera image (numpy array) for appearance feature extraction
            feature_extractor: instance of FeatureExtractor for appearance feature extraction

        Returns:
            accepted_assignments: set of indices of detections that were matched to existing trackers
        """

        detection_features = [None] * len(positions)
        if image is not None and feature_extractor is not None:
            for j in ambiguous_detections:
                px, py = pixel_positions[j]
                detection_features[j] = feature_extractor.extract_features(image, px, py)


        """
        Cost_matrix layout:
        Rows: Trackers (predicted positions)
        Columns: Detections (new positions)

                    detection_0     detection_1     detection_2
        tracker_0
        tracker_1
        tracker_2
        """
        cost_matrix = np.zeros((len(predicted_positions), len(positions)))
        for i, pred in enumerate(predicted_positions):
            for j, pos in enumerate(positions):
                distance_cost = np.linalg.norm(np.array(pred) - np.array(pos))
                distance_cost_normalized = min(distance_cost / self.match_radius(i), 1.0) # Normalize distance cost, so it can be combined with with appearance cost
                if j in ambiguous_detections and detection_features[j] is not None and self.trackers[i].appearance_feature is not None:
                    similarity = feature_extractor.compute_similarity(self.trackers[i].appearance_feature, detection_features[j])
                    appearance_cost = 1 - similarity
                    cost_matrix[i, j] = 0.7*distance_cost_normalized + 0.3*appearance_cost
                else:
                    #No ambiguity, use distance cost only
                    cost_matrix[i, j] = distance_cost_normalized

        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        accepted_assignments = set()

        for i, j in zip(row_indices, col_indices):
            if cost_matrix[i, j] < self.combined_cost_threshold:
                self.trackers[i].update(
                    positions[j], confidences[j], 
                    self.hits_threshold, orientations[j])
                if detection_features[j] is not None:
                    self.trackers[i].update_appearance_feature(detection_features[j])

                elif (self.trackers[i].appearance_feature is None and image is not None and feature_extractor is not None):
                    # Extract appearance feature for the tracker if it doesn't have one yet
                    px, py = pixel_positions[j]
                    feature = feature_extractor.extract_features(image, px, py)
                    if feature is not None:
                        self.trackers[i].update_appearance_feature(feature)
                accepted_assignments.add(j)
            else:
                self.trackers[i].mark_missed(self.missed_threshold, self.tentative_missed_threshold)
        
        for i in range(len(self.trackers)):
            if i not in row_indices:
                self.trackers[i].mark_missed(self.missed_threshold, self.tentative_missed_threshold)

        return accepted_assignments
    
    def update(self, detections, image = None, feature_extractor = None, timestamp = None):
        """
        Update the tracker with new detections.

        Args:
            detections: list of (x,y,confidence) from /human_poses_3d
            image: latest camera image (numpy array) for appearance feature extraction
            feature_extractor: instance of FeatureExtractor for appearance feature extraction
        """
        self.frame_count += 1

        positions = [det[:2] for det in detections]
        confidences = [det[2] for det in detections]
        orientations = [det[3] for det in detections]
        pixel_positions = [det[4:6] for det in detections]

        predicted_positions = []
        for track in self.trackers:
            predicted_pos = track.predict(timestamp)
            predicted_positions.append(predicted_pos)
        
        #No existing trackers, create new ones for all detections
        if len(self.trackers) == 0:
            for i, pos in enumerate(positions):
                new_tracker = KalmanPersonTracker(pos, timestamp)
                new_tracker.confidence = confidences[i]
                new_tracker.orientation = orientations[i]
                if image is not None and feature_extractor is not None:
                    px, py = pixel_positions[i]
                    feature = feature_extractor.extract_features(image, px, py)
                    if feature is not None:
                        new_tracker.update_appearance_feature(feature)

                self.trackers.append(new_tracker)
            return self.get_confirmed_tracks()
        
        # No detections, mark all trackers as missed
        if len(detections) == 0:
            for track in self.trackers:
                track.mark_missed(self.missed_threshold, self.tentative_missed_threshold)
            self.trackers = [t for t in self.trackers if t.state != 'DELETED']
            return self.get_confirmed_tracks()

        
        ambiguous_detections = self.get_ambiguous_items(positions, predicted_positions)

        tracker_ids = [t.id for t in self.trackers]
        tracker_missed = [t.missed for t in self.trackers]
        print(f"[MATCH] t={timestamp} trackers={tracker_ids} missed={tracker_missed} "
              f"predicted={predicted_positions} "
              f"detections={positions} ambiguous_detections={ambiguous_detections} "
              f"branch={'match_appearance' if ambiguous_detections else 'match_positions'}")

        if len(ambiguous_detections) == 0:
            accepted_assignments = self.match_positions(predicted_positions, positions, confidences, orientations, pixel_positions, image, feature_extractor)
        else:
            accepted_assignments = self.match_appearance(predicted_positions, positions, confidences, orientations, ambiguous_detections, pixel_positions, image, feature_extractor)

        # Create new trackers for unmatched detections
        for j in range(len(detections)):
            if j not in accepted_assignments:
                new_tracker = KalmanPersonTracker(positions[j], timestamp)
                new_tracker.confidence = confidences[j]
                new_tracker.orientation = orientations[j]
                if image is not None and feature_extractor is not None:
                    px, py = pixel_positions[j]
                    feature = feature_extractor.extract_features(image, px, py)
                    if feature is not None:
                        new_tracker.update_appearance_feature(feature)
                self.trackers.append(new_tracker)

        # Remove deleted trackers
        self.trackers = [t for t in self.trackers if t.state != 'DELETED']
        return self.get_confirmed_tracks()
    
