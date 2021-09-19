#include "path.h"

using namespace std;

path::path(){       //default constructor
    XValues = {0};
    ZValues = {0};
    segments = 0;
    distances = {0};
    param = {0};
}

path::~path(){      //destructor
    XValues = {0};
    ZValues = {0};
}

void path::setPathValues(int a) {       //function to call to set values for paths

    if(a == 1){                         //start of list to expand on for more cases in future iterations
        XValues = {75, 45, 15, -15, -45, -75};
        ZValues = {-20, 20, -40, 40, -60, 60};
    }
    else{                               //if not a valid input set to one point at (0,0)
        XValues = {0};
        ZValues = {0};
    }
}

void path::pathPrep(){

    segments = (XValues.size() - 1);    //there are one less segments than points in any graph

    distances.resize(segments+1 , 0);     //initializing distance vector to hold values for all points on path - init to 0
    for (int i = 1; i < segments+1; i++){       //as long as there are points left on the path calculate distance from start to end of path

        pointOne = {XValues[i - 1], ZValues[i - 1]};    //if moving along find distance between current point and last point touched
        pointTwo = {XValues[i], ZValues[i]};

        distances[i] = (distances[i-1] + distancePointPoint(pointOne, pointTwo));   //add the previous distance to the current distance then store into vector
    }

    param.resize(segments+1, 0);    //initializing path param to total amount of points on path - init to 0
    for (int j = 0; j < segments+1; j++){
        param[j] = (distances[j] / distances[segments]);
    }
}

float path::distancePointPoint(vector<float> A, vector<float> B){       //calculate distance along path by passing two vectors in
    return sqrt(pow((B[0] - A[0]), 2) + pow((B[1] - A[1]),2));      //return distance between the points using distance formula
}

