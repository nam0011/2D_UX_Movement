#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <algorithm>

using namespace std;

class path {

public:
    path();
    ~path();

    void setPathValues(int a);
    void pathPrep();
    static float distancePointPoint(vector<float> A, vector<float> B);

    vector<float> XValues;
    vector<float> ZValues;
    int segments;
    vector<float> distances;
    vector<float> param;
    vector<float> pointOne;
    vector<float> pointTwo;

};