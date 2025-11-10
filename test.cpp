
#include "plot.h"
#include "NN.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <sstream>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <ctime>
#ifdef _WIN32
    #include <conio.h>    // for _kbhit() and _getch() on Windows
#else
    #include <termios.h>  // for terminal control on macOS/Linux
    #include <unistd.h>   // for STDIN_FILENO
    #include <fcntl.h>    // for fcntl() non-blocking mode
#endif
#include <stdio.h>

using namespace std;
using namespace chrono;

#ifdef _WIN32
    #include <conio.h>    // for _kbhit() and _getch() on Windows
#else
    #include <termios.h>  // for terminal control on macOS/Linux
    #include <unistd.h>   // for STDIN_FILENO
    #include <fcntl.h>    // for fcntl() non-blocking mode
#endif

bool enterPressed() {
#ifdef _WIN32
    if (_kbhit()) {            // check if a key was pressed
        char c = _getch();     // get the pressed key
        return c == '\r';      // '\r' is Enter on Windows
    }
    return false;
#else
    struct termios oldt, newt;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);         // get current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);       // disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0); // get current flags
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // non-blocking input

    int ch = getchar();                     // try reading a character

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore settings
    fcntl(STDIN_FILENO, F_SETFL, oldf);      // restore flags

    return ch == '\n';  // '\n' is Enter on Unix/macOS
#endif
}


// changes scientific notation to double
double normalize(string normalizeInput) {

    double base = stod(normalizeInput.substr(0,normalizeInput.find('e')+1));
    int exponent = stoi(normalizeInput.substr(normalizeInput.find('e')+1));

    if(exponent != 0)

        return base * pow(10,exponent);

    else 

        return base;
}

int main() {
    
	// variables initialization	
	string inputData;
	string filename;
	ifstream inFS;
	ofstream outFS;
	vector<double> xCoords;
	vector<double> yCoords;
	double p = 0.10;
	
	// process: files intake
	cout << "Enter the name of file: ";
	cin >> filename;

	inFS.open(filename);

	if (!inFS.is_open()) {

		cout << "Could not open file " << filename << endl;
		return 1;
	}

	getline(inFS,inputData);

    // more variable initialization
	string xInput, yInput;
	double highestRange = 0;
	double biggestX = 0.0;
	double biggestY = 0.0;

    // reading input data from file
    while(!inFS.fail()) {

        stringstream ss(inputData);

        while(ss >> xInput >> yInput) {

            // making x and y values to be doubles
            double normalizeXInput = normalize(xInput);
			double normalizeYInput = normalize(yInput);

            // gets max values for axes range
			if (biggestX < normalizeXInput)

				biggestX = normalizeXInput;

			if (biggestY < normalizeYInput)

				biggestY = normalizeYInput;

            xCoords.push_back(normalizeXInput);
			yCoords.push_back(normalizeYInput);
        }
        getline(inFS,inputData);
    }

    // decides which is larger to create a square plot
	if (biggestX > biggestY)

		highestRange = biggestX;

	else 

		highestRange = biggestY;

    // to do ticks of 10 for graph display
	if ((int)highestRange % 10 != 0)

		highestRange += 10 - ((int)highestRange % 10);

	inFS.close();

	// variable initialization
	double distance = 0.0;

    // will get info for each number of drones
    // for(int i = 1; i <= 4; ++i) {
    //     k_means drone(i);
    //     drone.load_data(filename);

    //     if(i = 1)   {
    //         //clock stuff
            
    //     }
    //     drone.kMeansClustering();

        





        
    //     //finish
    // }
    
    //calculate distances with 1,2,3,4 drones
    //double drone1_distance = round(drone1.nearest_neighbor_distance()*10)/10;

	// prints to UI
	// cout << "1) If you use 1 drone(s), the total route will be " << drone1.get_size() << " meters" << endl;
	// cout << "    i. Landing Pad 1 should be at [cooridinates], serving " << drone1.get_size() << " locations, route is " << drone1_distance << " meters" << endl;
	k_means drone1(1);
    k_means drone2(2);
    k_means drone3(3);
    k_means drone4(4);
	drone1.load_data(filename); // re reads info 
    drone2.load_data(filename); // re reads info 
    drone3.load_data(filename); // re reads info 
    drone4.load_data(filename); // re reads info 
    time_t currentTime = time(nullptr);
    currentTime += 5 * 60;
    struct tm* localTime = localtime(&currentTime);
    int Hour = localTime->tm_hour;
    int Min = localTime ->tm_min;
    string amPM = "";
    string min_str = "";
    if(Hour == 0){
        Hour = 12;
        amPM = "am";
    }
    else if(Hour < 12){
        amPM = "am";
    }
    else if (Hour == 12){
        amPM = "pm";
    }
    else{
        Hour = Hour%12;
        amPM = "pm";
    }
    if(Min < 10){
        min_str = "0" + to_string(Min);
    } else {
        min_str = to_string(Min);
    }

    cout << "There are " << drone1.get_size() << " nodes: Solution will be available by " << Hour << ":" << min_str << "" << amPM << endl;
    
    drone1.kMeansClustering();
    cout << "clear" << endl;


    //calculate distances with 1,2,3,4 drones
    // double drone1_distance = round(drone1.nearest_neighbor_distance()*10)/10;

	// prints to UI
	cout << "1) If you use 1 drone(s), the total route will be " << drone1.getClusterDistances().at(0) << " meters" << endl;
	cout << "    i.   Landing Pad 1 should be at [cooridinates], serving " << drone1.get_size() << " locations, route is " << drone1.getClusterDistances().at(0) << " meters" << endl;

    drone2.kMeansClustering();
        cout << "clear" << endl;

    cout << "1) If you use 2 drone(s), the total route will be " << drone2.getSumOfDistances() << " meters" << endl;
	cout << "    i.   Landing Pad 1 should be at [cooridinates], serving " << drone2.getIndividualClusterSet().at(0).size() << " locations, route is " << drone2.getClusterDistances().at(0)-1 << " meters" << endl;
    cout << "    ii.  Landing Pad 2 should be at [cooridinates], serving " << drone2.getIndividualClusterSet().at(1).size() << " locations, route is " << drone2.getClusterDistances().at(1)-1 << " meters" << endl;
    
    drone3.kMeansClustering();
        cout << "clear" << endl;

    cout << "1) If you use 3 drone(s), the total route will be " << drone3.getSumOfDistances() << " meters" << endl;
	cout << "    i.   Landing Pad 1 should be at [cooridinates], serving " << drone3.getIndividualClusterSet().at(0).size() << " locations, route is " << drone3.getClusterDistances().at(0)-1 << " meters" << endl;
    cout << "    ii.  Landing Pad 2 should be at [cooridinates], serving " << drone3.getIndividualClusterSet().at(1).size() << " locations, route is " << drone3.getClusterDistances().at(1)-1 << " meters" << endl;
    cout << "    iii. Landing Pad 3 should be at [cooridinates], serving " << drone3.getIndividualClusterSet().at(2).size() << " locations, route is " << drone3.getClusterDistances().at(2)-1 << " meters" << endl;

    drone4.kMeansClustering();
        cout << "clear" << endl;
	
    cout << "1) If you use 4 drone(s), the total route will be " << drone4.getSumOfDistances() << " meters" << endl;
	cout << "    i.   Landing Pad 1 should be at [cooridinates], serving " << drone4.getIndividualClusterSet().at(0).size() << " locations, route is " << drone4.getClusterDistances().at(0)-1 << " meters" << endl;
    cout << "    ii.  Landing Pad 2 should be at [cooridinates], serving " << drone4.getIndividualClusterSet().at(1).size() << " locations, route is " << drone4.getClusterDistances().at(1)-1 << " meters" << endl;
    cout << "    iii. Landing Pad 3 should be at [cooridinates], serving " << drone4.getIndividualClusterSet().at(2).size() << " locations, route is " << drone4.getClusterDistances().at(2)-1 << " meters" << endl;
    cout << "    iv.  Landing Pad 4 should be at [cooridinates], serving " << drone4.getIndividualClusterSet().at(3).size() << " locations, route is " << drone4.getClusterDistances().at(3)-1 << " meters" << endl;
        
        // cout << "		" << distance << endl;
	double BSF = distance;
	string fileNameAdjusted = " ";;
    
	cin.ignore();
	srand(time(NULL));
    
    // getting improved distances
// 	while (true) {

    //double new_distance = round(drone1.modified_nearest_neighbor_distance(p)*10)/10;
        
//         if (new_distance < BSF) {

//             BSF = new_distance;
//             cout << "		" << BSF << endl;
//         }

//         if (enterPressed()) break;  

//    }
   int numOfDrones;
   k_means chosenDroneGroup;

   cout << "Enter a choice" << endl;
   cin >> numOfDrones;
	ostringstream dist;
    fileNameAdjusted = filename.substr(0, filename.find('.'));

    dist << fixed << setprecision(0) << BSF;

    // over 6k distance warning
    if (BSF > 6000) {
        cout << "Warning: Solution is " << BSF << ", greater than the 6000-meter contraint." << endl;
    }

    // writing route to file
	string outputFilename = filename + "_SOLUTION_" + dist.str() + ".txt";
	// drone1.write_route_to_file(outputFilename);	

    // graph plotting process
    signalsmith::plot::Plot2D plot(1920, 1920);
	plot.x.label("X-Axis");
	plot.y.label("Y-Axis");
	plot.x.major(0);
	plot.y.major(0);

	for(int i = 0; i <= highestRange; i += 10) {

		plot.x.minor(i);
		plot.y.minor(i);
	}
    vector<vector<tuple<int,double,double>>> route;
    if(numOfDrones == 1) {
        route = drone1.getIndividualClusterSet();
        drone1.write_route_to_file(fileNameAdjusted,1);
    }
    else if(numOfDrones == 2) {
        route = drone2.getIndividualClusterSet();
        drone2.write_route_to_file(fileNameAdjusted,2);
    }
    else if(numOfDrones == 3) {
        route = drone3.getIndividualClusterSet();
        drone3.write_route_to_file(fileNameAdjusted,3);
    }
    else {
        route = drone4.getIndividualClusterSet();
        drone4.write_route_to_file(fileNameAdjusted,4);
    }

    auto &scatter = plot.lineFill();
    for (int i = 0; i < numOfDrones; i++) {
        vector<tuple<int,double,double>> currRoute = route.at(i);
        tuple<int,double,double> landingZoneIndex = currRoute.at(0);
        auto &currline = plot.line();

        currline.add(get<1>(landingZoneIndex),get<2>(landingZoneIndex));
        currline.marker(get<1>(landingZoneIndex),get<2>(landingZoneIndex),0);
        //currline2.marker(xCoords[landingZoneIndex],yCoords[landingZoneIndex]);
        for(int j = 1; j < currRoute.size(); j++) {
            tuple<int,double,double> index = currRoute.at(j);
            
            currline.marker(get<1>(index),get<2>(index),0);
            currline.add(get<1>(index),get<2>(index));
        }

        currline.add(get<1>(landingZoneIndex),get<2>(landingZoneIndex));
        
        scatter.dot(get<1>(landingZoneIndex),get<2>(landingZoneIndex),6,1);
        
        
    }

    // starting point
    // returning to starting point
	// currline.dot(xCoords[route[0]],yCoords[route[0]],4,1);
	// currline.add(xCoords[route[0]], yCoords[route[0]]);
	// currline2.marker(xCoords[route[0]],yCoords[route[0]]);

    //note:add creates the lines and marker makes the points

    // saves plot to svg file
	string pngFilename = fileNameAdjusted + "_OVERALL_SOLUTION.svg";
	plot.write(pngFilename);	
}
