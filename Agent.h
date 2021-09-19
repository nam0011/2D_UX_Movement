#include "path.h"

using namespace std;

class Agent{

public:

    Agent();        //constructor
    ~Agent();       //destructor
    void SetValues(int a);  //takes an integer and sets character initial values based on choice
    void SetTarget(Agent a); //takes target agent as input, sets characters target point

    vector<float> getPosition();
    vector<float> position;
    float getVectorLength(std::vector<float> a);
    float dotProduct(std::vector<float> A, std::vector<float> B);
    float pathGetParam(path pathOne);
    vector<float> normalizeVector(std::vector<float> a);
    std::vector<float> closetPointSegment(std::vector<float> Q, std::vector<float> A, std::vector<float> B);
    std::vector<float> pathGetPosition(path pathOne, float targetParam);
    float vectorLength(std::vector<float> A);
    Agent getSteeringSeek(Agent target);
    Agent getSteeringFlee(Agent target);
    Agent getSteeringArrive(Agent target);
    Agent getSteeringStop(Agent character);
    Agent getSteeringFollowPath(class path);
    void DynamicUpdate(Agent Steering, float deltaTime);

    float pi = 3.14159;         //public constant variables
    float deltaTime = 0.5;
    float stopSpeed = 0.01;
    float stopRotate = 0.01;

    float steer;
    int id;
    vector<float> velocity;
    vector<float> linear;
    float orientation;
    float rotation;


    bool align;
private:
    vector<float> targetPosition;
    vector<float> vector;
    std::vector<float> A;
    std::vector<float> B;
    float currentParam;
    float targetParam;
    std::vector<float> nullVector;
    std::vector<float> checkPoint;
    std::vector<float> closestPoint;
    float thisParam;
    int closestSegment;
    float closestDistance;
    float checkDistance;
    string steerBehavior;
    float dotOverDot;
    float angular;
    float maxSpeed;
    float maxAcceleration;
    float targetRadius;
    float slowRadius;
    float timeToTarget;
    float collideRadius;
    float avoidRadius;
    int pathNum;
    float offset;
    float maxPrediction;
    float max;
};



