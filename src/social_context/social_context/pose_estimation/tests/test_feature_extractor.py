import cv2
import numpy as np
import sys
from feature_extractor import FeatureExtractor

extractor = FeatureExtractor()

# Red image
red = np.zeros((480, 640, 3), dtype=np.uint8)
red[:, :] = [0, 0, 255]   # BGR red

# Blue image
blue = np.zeros((480, 640, 3), dtype=np.uint8)
blue[:, :] = [255, 0, 0]  # BGR blue

# Green image
green = np.zeros((480, 640, 3), dtype=np.uint8)
green[:, :] = [0, 255, 0]  # BGR green

# Extract features
f_red1  = extractor.extract_features(red,   320, 240)
f_red2  = extractor.extract_features(red,   320, 240)
f_blue  = extractor.extract_features(blue,  320, 240)
f_green = extractor.extract_features(green, 320, 240)

# Compare
print("=== Similarity Tests ===")
print(f"Red vs Red:   {extractor.compute_similarity(f_red1, f_red2):.3f}  ← should be 1.0")
print(f"Red vs Blue:  {extractor.compute_similarity(f_red1, f_blue):.3f}  ← should be low")
print(f"Red vs Green: {extractor.compute_similarity(f_red1, f_green):.3f} ← should be low")
print(f"Blue vs Green:{extractor.compute_similarity(f_blue, f_green):.3f} ← should be low")

# Edge cases
print("\n=== Edge Case Tests ===")
print(f"None vs None: {extractor.compute_similarity(None, None):.3f}  ← should be 0.0")
print(f"None vs feat: {extractor.compute_similarity(None, f_red1):.3f} ← should be 0.0")

f_edge = extractor.extract_features(None, 320, 240)
print(f"None image:   {f_edge}  ← should be None")

f_oob = extractor.extract_features(red, 0, 0)
print(f"Corner crop:  {f_oob is not None}  ← should be True (clamped to boundary)")