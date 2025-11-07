#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime> // for random
using namespace std;

class k_means{
   private:
        int k;      //number of drones
        int numIterations;
        int num_points;
        double coordinates[4096][2];
        int route[4096];
        double bestRouteDistance = 1000000;
        int bestCluster[4096];
        double OF;
        vector<vector<int>> clusterRoute; 
        vector<double> clusterDistances;

        
   public:
    k_means() : num_points(0), k(0), numIterations(10) {}
    k_means(int num_drones) : num_points(0), k(num_drones), numIterations(10){}
    int get_size() const { return num_points; }
    vector<vector<int>> get_route() const;  
    void load_data(const string &filename);         //updates coordinate array by retrieving coordinates from file    
    //double euclidean(int i, int j);
    void write_route_to_file(const string &inputFilename, int choosenNumClusters);
    void nearest_neighbor_distance(vector<vector<pair<double, double>>> IndividualClusters);
    //double modified_nearest_neighbor_distance(double p);
    
    void kMeansClustering();

};   

void k_means::kMeansClustering()    {
    srand(time(NULL));
    
    

    double bestDistanceOverall = 100000;
    
    
    // number of times to run k means to get BEST clusters
    for(int l = 0; l < numIterations; ++l){
        vector<vector<pair<double, double>>> IndividualClusters(k);
        bestRouteDistance = 1e9;
        double centers[k][2];

        // throwing darts part
        for(int i = 0; i < k; ++i){
            int randCenter = rand() % num_points;
            centers[i][0] = coordinates[randCenter][0];         //x coord
            centers[i][1] = coordinates[randCenter][1];         //y coord    
        }

        int clusters[4096] = {0}; 
        bool changed; 
    
        do {
            changed = false;
            
            // assign every point to nearest cluster center
            
            for (auto &cluster : IndividualClusters) cluster.clear();
            
            for(int j = 0; j < num_points; ++j){
                
                double minDist = 100000;
                int nearestCluster = -1;

                for(int cIndex = 0; cIndex < k; ++cIndex){                  // calculates distance from point j to cluster center cIndex
                    double x = coordinates[j][0] - centers[cIndex][0];
                    double y = coordinates[j][1] - centers[cIndex][1];
                    double dist = sqrt(x*x + y*y);

                    if(dist < minDist){
                        minDist = dist;
                        nearestCluster = cIndex;
                    }
                }
                
                if(clusters[j] != nearestCluster){
                    changed = true;
                    clusters[j] = nearestCluster; // assign point j to nearest cluster
                }

                IndividualClusters[nearestCluster].push_back({coordinates[j][0], coordinates[j][1]});
                
            }

            double newCenters[k][2];
            for (int i = 0; i < k; ++i) {
                newCenters[i][0] = 0.0;
                newCenters[i][1] = 0.0;
            }
            
            
            int pointsInCluster[k];
            for (int i = 0; i < k; ++i) {
                pointsInCluster[i] = 0;           
            } 
        
            // movement of cluster centers
            for(int t = 0; t < num_points; ++t){ //note: iffy
                int cIndex = clusters[t];
                newCenters[cIndex][0] += coordinates[t][0];
                newCenters[cIndex][1] += coordinates[t][1];
                pointsInCluster[cIndex] += 1;
            }

            for(int cIndex = 0; cIndex < k; ++cIndex){
                if(pointsInCluster[cIndex] > 0){
                    centers[cIndex][0] = newCenters[cIndex][0] / pointsInCluster[cIndex];
                    centers[cIndex][1] = newCenters[cIndex][1] / pointsInCluster[cIndex];
                }

                IndividualClusters[cIndex].insert(IndividualClusters[cIndex].begin(), {centers[cIndex][0], centers[cIndex][1]});
            }


        } while(changed);
        
        nearest_neighbor_distance(IndividualClusters);
        if (bestRouteDistance < bestDistanceOverall) {
            bestDistanceOverall = bestRouteDistance;
        }
        
        
        

        
    }

    // for(int i = 0; i < num_points; ++i){
    //     route[i] = bestCluster[i];
    // }
    
    //need to fix. clusters was defined in loop above so it runs into issues here. Same thing with centers 
    // double currentOF = 0.0;
    
    // for(int b = 0; b < num_points; b++){
    //     int cIndex = clusters[b];
    //     double dist_x = coordinates[b][0] - centers[cIndex][0];
    //     double dist_y = coordinates[b][1] - centers[cIndex][1];
    //     currentOF = dist_x*dist_x + dist_y*dist_y;
    // }
}




vector<vector<int>> k_means::get_route() const  {
    return clusterRoute;
}


void k_means::load_data(const string &filename) {

    ifstream dataPoints;
    dataPoints.open(filename);

    if (!dataPoints) {

        cout << "Error with opening file " << endl;
        exit(1); 
    }
    
    num_points = 0;
    string coordinate_line;

    while(getline(dataPoints, coordinate_line)) {

        stringstream ss(coordinate_line);
        double x, y;
        double extra; // used to check for extra invalid data

        // Error check : valid file format
        if (!(ss >> x >> y) || (ss >> x >> y >> extra)) {

            cout << "Error: Invalid file format" << endl;
            exit(1);
        }

        ss >> x >> y;
        coordinates[num_points][0] = x;
        coordinates[num_points][1] = y;
        
        //Error check : valid number of locations
        if(num_points > 4096) {

            cout << "Error: number of locations is greater than 4096" << endl;
            exit(1);
        }

        num_points++;
    }

    dataPoints.close();  
}        


// Euclidean distance between two points by index
// double k_means::euclidean(int i, int j) {
//     double dx = coordinates[i][0] - coordinates[j][0];
//     double dy = coordinates[i][1] - coordinates[j][1];
//     return sqrt(dx * dx + dy * dy); 
// }




void k_means::write_route_to_file(const string &inputFilename, int choosenNumClusters) {
    for (int c = 0; c < choosenNumClusters; ++c) {         // iterate over the vector
        ostringstream oss;
        oss.precision(0);
        oss << fixed << clusterDistances[c];
        string fileName = inputFilename + "_" + to_string(c+1) + "_SOLUTION_" + oss.str() + ".txt"; 
        ofstream fout(fileName);
        if (!fout) {
        cerr << "Error: Could not write to " << fileName << endl;
        continue;
        }
        for(int i = 0; i < clusterRoute[c].size(); ++i){
            fout << clusterRoute[c][i] << endl;
        }
        fout.close();
    }
    //In the test.cpp have a for loop that prints out: Writing fileName1, fileName2, fileName3, etc to disk
}

void k_means::nearest_neighbor_distance(vector<vector<pair<double, double>>> IndividualClusters) {

    // variables initialization
    double total_distance_all_clusters = 0.0;
    vector<vector<int>> tempRoute;
    clusterDistances.clear();
    for(int c = 0; c < IndividualClusters.size(); ++c){
        vector<pair<double, double>> cluster = IndividualClusters[c];
        int cluster_size = cluster.size();

        if(cluster_size > 0){
            bool visited[4096] = {false};    
            double cluster_distance = 0.0;
            int cluster_route[4096];
            int current_tree = 0;      // start at the first coordinate
            cluster_route[0] = current_tree;
            visited[current_tree] = true;

            // process: every tree gets checked and compared with other trees
            for (int i = 1; i < cluster_size; ++i) {

                int next_tree = -1;
                double minDist = 100000;

                // the trees being compared to
                for (int j = 0; j < cluster_size; ++j) {
                    
                    if (!visited[j]) { // not interested in nodes already in recorded route
                        double dx = cluster[current_tree].first - cluster[j].first;
                        double dy = cluster[current_tree].second - cluster[j].second;
                        double dist = sqrt(dx * dx + dy * dy); 

                        // finding closest tree and compares to previous closest tree
                        if (dist < minDist) {
                            minDist = dist;
                            next_tree = j;
                        }
                    }
                }  
                // updates route as close tree found and removes from being checked again
                if (next_tree != -1) {
                    cluster_distance += minDist;
                    cluster_route[i] = next_tree;
                    visited[next_tree] = true;
                    current_tree = next_tree;
                }
            }
            double dx = cluster[current_tree].first - cluster[0].first; // this part in the wrong place???
            double dy = cluster[current_tree].second - cluster[0].second;
            cluster_distance += sqrt(dx * dx + dy * dy); 
            cluster_route[cluster_size] = 0;
            total_distance_all_clusters += cluster_distance; 
            tempRoute.push_back(vector<int>(cluster_route, cluster_route + cluster_size + 1));
            clusterDistances.push_back(cluster_distance);
        }
    }   
   
    if(bestRouteDistance > total_distance_all_clusters){
        bestRouteDistance = total_distance_all_clusters;
        clusterRoute = tempRoute;
            
    }
}




// double k_means::modified_nearest_neighbor_distance(double p) {

//     // variables initialization
//     bool visited[4096] = {false};
//     double total_distance = 0.0;
//     int current_tree = 0;
//     int temp_route[4096];
//     temp_route[0] = current_tree;
//     visited[current_tree] = true;

//     // process: every tree gets checked and compared with other trees
//     for (int i = 1; i < num_points; ++i) {

//         int bestNextTree1 = -1;  //Indices of the 2 closest trees
//         int bestNextTree2 = -1;

//         double bestDistance = 100000;  //Distance to the 2 closest trees
//         double bestDistance2nd = 100000;

//         // the trees being compared to
//         for (int j = 0; j < num_points; ++j) {

//             if (!visited[j]) { // not interested in nodes already in recorded route

//                 double d = euclidean(current_tree, j);
                
//                 if (d < bestDistance) {

//                     //If we find a closer tree we move the current best distance to the 2nd best distance 
//                     bestDistance2nd = bestDistance; 
//                     bestNextTree2 = bestNextTree1;

//                     //and update the best distance
//                     bestDistance = d; 
//                     bestNextTree1 = j;

//                 } else if (d < bestDistance2nd) {
                    
//                     bestDistance2nd = d; 
//                     bestNextTree2 = j;
//                 }
//             }
//         }

//         int next_tree = bestNextTree1;

//         // there's a gamble of whether your first best tree gets replaced by the 2nd best tree w/ given probability of p
//         if (bestNextTree2 != -1 && ((double)rand() / RAND_MAX) < p) {

//             next_tree = bestNextTree2;

//         }

//         // updates chosen tree to route and removes from being checked again
//         total_distance += euclidean(current_tree, next_tree);
//         temp_route[i] = next_tree;
//         visited[next_tree] = true;
//         current_tree = next_tree;
//     }

//     // Return to the start
//     total_distance += euclidean(current_tree, 0);
//     temp_route[num_points] = 0;

//     // saves route to be official route
//     if(bestRouteDistance > total_distance){

//         bestRouteDistance = total_distance;

//         for (int i = 0; i <= num_points; ++i){

//             route[i] = temp_route[i];
//         }
//     }
    
    
//     return total_distance;
// }