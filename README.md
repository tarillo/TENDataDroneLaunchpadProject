# Senior Design Project - Multi-Drone Route Optimization via K-Means Clustering
This project focuses on optimizing multi-drone route planning for a pheromone delivery system. It reads location data, applies K-means clustering to assign regions to up to four drones, and computes efficient routes using a nearest neighbor heuristic with iterative improvement. The program outputs optimized launch pad locations, total travel distances, and visualizations to help select the most efficient drone configuration within a fixed time constraint.

# Contributors:
### Neha Gutapalli
### Tanya Carillo
### Eric Via
### Devin Alexandre

# Software Architecture:
* C++ for high-performance computation and algorithm implementation
* Custom K-Means class for clustering delivery locations
* Nearest Neighbor + probabilistic variant for route optimization
* Chrono library for enforcing time-constrained optimization (contract algorithm)
* File I/O for reading input datasets and writing solution outputs
* signalsmith::plot library for generating route visualizations (SVG output)
* Cross-platform terminal handling (Windows/Linux/macOS) for user interaction

# Implementation
Main Data Structure: Vectors / Nested Vectors of Tuples

      Vector (All Coordinates)
         |__ xCoords (vector<double>)
         |__ yCoords (vector<double>)
      
      Vector (Clusters per Drone Configuration)
         |__ Cluster 1 (vector<tuple<index, x, y>>)
         |__ Cluster 2 (vector<tuple<index, x, y>>)
         |__ Cluster 3 (vector<tuple<index, x, y>>)
         |__ Cluster 4 (vector<tuple<index, x, y>>)
      
      Vector (Routes per Cluster)
         |__ Route (ordered sequence of nodes)
                |__ Start (Launch Pad)
                |__ Intermediate Nodes
                |__ Return to Start

