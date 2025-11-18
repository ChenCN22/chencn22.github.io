---
layout: post
title: "Stereo Depth → Point Cloud Reconstruction Workflow"
date: 2025-11-14
categories: [Computer Vision]
---

# Stereo 3D Reconstruction Workflow  
*A practical pipeline from image capture to 3D point-cloud visualization.*

---

## 1. Capture Stereo Image Pair
### Objective
Record two images of the same scene from slightly different horizontal viewpoints.

### Notes to Add
- Camera rig description  
- Baseline distance  
- Lighting conditions  
- Any challenges during acquisition  

### Insert Images

---

## 2. Preprocess the Images
### Goal
Convert the raw stereo pair into a form ready for disparity computation.

### Typical Steps (you fill in details)
- Resize / crop  
- Convert to grayscale  
- Optional histogram equalization  
- Optional noise reduction (Gaussian / bilateral)  

### Code Placeholder

# load & preprocess example
left  = cv2.imread('left.png')
right = cv2.imread('right.png')

left_gray  = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)


---

## 3\. Compute the Disparity Map (Stereo Matching)

### Goal

Estimate pixel-wise disparity using Semi-Global Block Matching (SGBM).

### Add Your Chosen Parameters

-   minDisparity
    
-   numDisparities
    
-   blockSize
    
-   P1 / P2 (smoothness penalties)
    
-   speckle filtering parameters
    

### Code Placeholder

```python
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,
    blockSize=7,
    # 8*number_of_image_channels*blockSize*blockSize \
    # and 32*number_of_image_channels*blockSize*blockSize
    P1=8 * 3 * 7**2,
    P2=32 * 3 * 7**2,
)
disparity = stereo.compute(left_gray, right_gray) / 16.0
```

### Insert Disparity Visualization

![Disparity](/assets/img/baby-disparity.png)

---

## 4\. Depth Conversion

### Goal

Convert disparity values into a **pseudo-depth Z** based on

$$
Z \propto \frac{1}{d}
$$

### Add Notes

-   No camera calibration → using relative depth
    
-   Role of scale factor $k = b \cdot f$
    
-   Handling noise, invalid pixels
    

### Code Placeholder

```python
k = 20000.0
Z = k / disparity[y, x]
```

---

## 5\. Construct the Point Cloud

### Goal

For each valid disparity pixel, generate a 3D point (X, Y, Z) with color (R,G,B).

### Add Notes

-   Why using pixel coordinates for X,Y
    
-   How Z affects perceived geometry
    
-   Optional filtering of tiny / extreme disparities
    

### Code Placeholder

```python
points.append([X, Y, Z, R, G, B])
```

---

## 6\. Export to PLY File

### Goal

Write the 3D points into an ASCII PLY file.

### Placeholder

```python
with open("scene.ply", "w") as f:
    f.write("ply\nformat ascii 1.0\n")
    f.write(f"element vertex {len(points)}\n")
    ...
```

---

## 7\. Visualize in Open3D

### Goal

Display the reconstructed point cloud.

### Code Placeholder

```python
import open3d as o3d

pc = o3d.io.read_point_cloud("scene.ply")
o3d.visualization.draw_geometries([pc])
```

### Insert Screenshot


![Point cloud result](path/to/pc.png)


---

## 8\. Discussion & Observations

### Prompts for You to Fill

-   Quality of disparity
    
-   Regions with noise / holes
    
-   Effect of blockSize
    
-   Influence of texture richness
    
-   How different scenes behave differently
    
-   Limitations due to lack of camera matrix
    

---

## 9\. Conclusion

### Prompts

-   What worked well
    
-   What failed
    
-   Future improvements (e.g., calibration, better SGBM tuning, post-processing, Poisson surface reconstruction)
    

---

## Appendix: Full Code (Optional)

Add the cleaned version of your final script here.