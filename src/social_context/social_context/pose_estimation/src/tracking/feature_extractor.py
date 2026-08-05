import cv2
import numpy as np
from torchreid.reid.utils import FeatureExtractor as OSNetExtractor

class FeatureExtractor():
    def __init__(self, bbox_width=100, bbox_height=200, model_name='osnet_x0_5', device='cpu'):
        """
        Initialize the feature extractor

        Args:
            bbox_width: Width of the bounding box for feature extraction
            bbox_height: Height of the bounding box for feature extraction
            model_name: Name of the pre-trained model to use for feature extraction
            device: Device to run the model on ('cpu' or 'cuda')
        """
        self.bbox_width = bbox_width
        self.bbox_height = bbox_height

        self.model = OSNetExtractor(model_name=model_name, device=device)


    def extract_features(self, image, center_x, center_y):
        """
        Crop a person region and extract features from the image using the pre-trained model

        Args:
            image: Input image -> numpy array in BGR format
            center_x: X coordinate of the center of the bounding box
            center_y: Y coordinate of the center of the bounding box

        Returns:
            512-dimensional feature vector for the person region
        """
        if image is None:
            return None
        
        h, w = image.shape[:2]

        half_w = self.bbox_width // 2
        half_h = self.bbox_height // 2

        # Calculate the bounding box coordinates
        x1 = int(max(center_x - half_w, 0))
        y1 = int(max(center_y - half_h, 0))
        x2 = int(min(center_x + half_w, w))
        y2 = int(min(center_y + half_h, h))

        # invalid box
        if x2 <= x1 or y2 <= y1:
            return None 

        min_height_fraction = 0.6
        if (y2 - y1) < self.bbox_height * min_height_fraction:
            return None  # Skip if the bounding box height is too small

        

        crop = image[y1:y2, x1:x2]

        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)

        features = self.model([crop_rgb]) 

        return features[0].cpu().numpy()

       

    def compute_similarity(self, feature1, feature2):
        """
        Compute the similarity between two feature vectors using cosine similarity

        Args:
            feature1: First feature vector
            feature2: Second feature vector

        Returns:
            Similarity score (0.0 - 1.0), where 1.0 means identical and 0.0 means completely different
        """
        
        if feature1 is None or feature2 is None:
            return 0.0

        # Convert features to numpy arrays and normalize them
        feature1 = np.asarray(feature1)
        feature2 = np.asarray(feature2)

        norm1 = np.linalg.norm(feature1)
        norm2 = np.linalg.norm(feature2)

        if norm1 == 0 or norm2 == 0:
            return 0.0
        
        # compute cosine similarity between the two feature vectors
        cosine_similarity = np.dot(feature1, feature2) / (norm1 * norm2)
        cosine_similarity = np.clip(cosine_similarity, -1.0, 1.0)
        return float((cosine_similarity + 1.0) / 2.0 )
        