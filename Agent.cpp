#include "Agent.h"


using namespace std;

Agent::Agent(){             //set initial values to 0
    id = 0;
    steerBehavior = "";
    position = {0,0};
    velocity = {0,0};
    linear = {0,0};
    orientation = 0;
    rotation = 0;
    angular = 0;
    maxSpeed = 0;
    maxAcceleration = 0;
    targetRadius = 0;
    slowRadius = 0;
    timeToTarget = 0;
    align = false;
    collideRadius = 0;
    avoidRadius = 0;
    targetPosition = {0,0};
    steer = 0;
}

Agent::~Agent()= default;

void Agent::SetValues(int behavior) {  //takes an integer and sets character initial values based on choice

    switch(behavior) {
        case 1:                 //behavior 1 = stop
            id = 161;
            steerBehavior = "STOP";
            position = {0, 0};
            velocity = {0, 0};
            linear = {0, 0};
            orientation = 0;
            rotation = 0;
            angular = 0;
            maxSpeed = 0;
            maxAcceleration = 0;
            targetRadius = 0;
            slowRadius = 0;
            timeToTarget = 0;
            align = false;
            collideRadius = 0.5;
            avoidRadius = 2;
            targetPosition = {0, 0};
            steer = 1;
            break;
        case 2:                 //behavior 2 = flee
            id = 162;
            steerBehavior = "FLEE";
            position = {-25, 50};
            velocity = {0, -8};
            linear = {0, 0};
            orientation = pi / 4;
            rotation = 0;
            angular = 0;
            maxSpeed = 10;
            maxAcceleration = 2;
            targetRadius = 0;
            slowRadius = 0;
            timeToTarget = 1;
            align = false;
            collideRadius = 0.5;
            avoidRadius = 2;
            targetPosition = {0, 0};
            steer = 4;
            break;
        case 3:                 //behavior 3 = seek
            id = 163;
            steerBehavior = "SEEK";
            position = {50, -25};
            velocity = {0, -8};
            linear = {0, 0};
            orientation = (3 * pi) / 2;
            rotation = 0;
            angular = 0;
            maxSpeed = 8;
            maxAcceleration = 2;
            targetRadius = 0;
            slowRadius = 10;
            timeToTarget = 1;
            align = true;
            collideRadius = 0.5;
            avoidRadius = 2;
            targetPosition = {0, 0};
            steer = 3;
            break;
        case 4:                 //behavior 4 = arrive
            id = 164;
            steerBehavior = "ARRIVE";
            position = {-50, -75};
            velocity = {-6, -4};
            linear = {0, 0};
            orientation = pi;
            rotation = 0;
            angular = 0;
            maxSpeed = 8;
            maxAcceleration = 2;
            targetRadius = 0;
            slowRadius = 20;
            timeToTarget = 1;
            align = true;
            collideRadius = 0;
            avoidRadius = 0;
            targetPosition = {0, 0};
            steer = 5;
            break;
        case 5:                //behavior 5 = path follow
            id = 171;
            steerBehavior = "PATH FOLLOW";
            position = {70, -40};
            velocity = {0, 0};
            linear = {0, 0};
            orientation = 0;
            rotation = 0;
            angular = 0;
            maxSpeed = 4;
            maxAcceleration = 2;
            targetRadius = 0;
            slowRadius = 5;
            timeToTarget = 0;
            align = true;
            collideRadius = 0.5;
            avoidRadius = 0;
            steer = 8;
            offset = 0.05;
            break;
    }
}

/**************************
 GENERAL FUNCTIONS              -- NEED TO APPLY GETTERS FOR MAIN TO USE FOR SECURITY IN LATER UPDATE
**************************/

void Agent::SetTarget(Agent target) { //takes target agent as input, sets characters target point
        targetPosition = target.getPosition();
}

vector<float> Agent::getPosition(){ //returns character position vector
    return position;
}

float Agent::getVectorLength(std::vector<float> a){     //takes in any vector, calculates length and returns length value
    float length;
    length = sqrt(pow(a[0],2)+pow(a[1],2));
    return length;
}

vector<float> Agent::normalizeVector(std::vector<float> a){     //takes in any vector, returns normalized values
    float length = getVectorLength(a);

    if(a != nullVector) {     //if the vector is not null
        if (length != 0) {      //check if the length is greater than zero so we dont divide by 0
            if (a[0] == 0) {    //check if we have a null x
                a[0] = 0;       //x = 0 if we do
                a[1] = a[1] / length; //normalize z component
            } else if (a[1] == 0) { //check if we have null z
                a[0] = a[0] / length; //normalize x
                a[1] = 0;   //z = 0
            } else {        //if neither are null
                a[0] = a[0] / length; //normalize the vector completely
                a[1] = a[1] /length;
            }
        } else {
            cout << "cannot normalize null vector" << endl; //if vector fails this test alert the user
        }
    }
    return a;
}


/**************************
PATH FOLLOWING FUNCTIONS
**************************/

std::vector<float> Agent::pathGetPosition(path pathOne, float targetParam){

    std::vector<float> A;                   //create some local variables -- need to do this even though some are shadowing Agent.h variables
    std::vector<float> B;
    std::vector<float> S;
    std::vector<float> minParams;
    int i;
    float T;

    for (int i = 0; i < pathOne.param.size(); i++) {  //while not at the end of the path parameters list
        if (targetParam > pathOne.param[i])     //if the target incoming target parameter is larger than the path paramter at the ith index
            minParams.push_back(pathOne.param[i]);  //push the value of that path parameter to the minimum parameters list
    }

    i = minParams.size() - 1;   //i is the size of the minimum parameters list - use -1 because start at 0

    A = { pathOne.XValues[i],pathOne.ZValues[i] };          //do some math to eventually find S

    B = { pathOne.XValues[i + 1],pathOne.ZValues[i + 1] };

    T = (targetParam - pathOne.param[i]) / (pathOne.param[i + 1] - pathOne.param[i]);

    S = { A[0] + (B[0] - A[0]) * T, A[1] + (B[1] - A[1]) * T };

    return S;       //return S
}


float Agent::pathGetParam(path pathOne){

    closestDistance = infinityf();      //set initial closestDistance to the highest number your architecture can provide

    for(int i = 0; i < pathOne.segments; i++){              //as long as the we aren't out of pathOne segments bounds
        A = { pathOne.XValues[i], pathOne.ZValues[i]};      //set A to current index values
        B = {pathOne.XValues[i+1], pathOne.ZValues[i+1]};  //set B to the next index values
        checkPoint = closetPointSegment(position, A, B);   //set the check point by finding the closest point of the segment we are closest to
        checkDistance = path::distancePointPoint(position, checkPoint);     //use the closest point to our point to check our distance from the path

        if(checkDistance < closestDistance){        //if the check distance is closer than the closest Distance we should fix some things
            closestPoint = checkPoint;  //the closest point becomes the check point
            closestDistance = checkDistance;    //the closest distance becomes the check distance
            closestSegment = i; //the closest segment is represented by the index of the segment we are currently moving on
        }
    }

    Agent A;        //here we need a few dummy agents to hold some values for math for us and take use of their existing vectors and variables
    Agent B;        //this isnt space friendly but the thisParam call can only be made on Agents in my code
    Agent C;

    A.position = {pathOne.XValues[closestSegment] , pathOne.ZValues[closestSegment]};   //set some values for agent A using current index
    A.thisParam = pathOne.param[closestSegment];

    B.position = {pathOne.XValues[closestSegment+1] , pathOne.ZValues[closestSegment+1]};   //set some values for agent B using next index
    B.thisParam = pathOne.param[closestSegment+1];

    C.closestPoint = closestPoint;      //have agent C closestPoint be the point we calculated earlier

    std::vector<float> CminusA;         //easier to do math if we set some temp vectors and then split the math up first
    std::vector<float> BminusA;

    CminusA = {C.closestPoint[0] - A.position[0] , C.closestPoint[1] - A.position[1]};  //do some math for later use
    BminusA = {B.position[0] - A.position[0] , B.position[1] - A.position[1]};


    float lengthOverLength; //use the above math to calculate the ratio between the to distances
    lengthOverLength = getVectorLength(CminusA) / getVectorLength((BminusA));

    C.thisParam = A.thisParam + (lengthOverLength*(B.thisParam - A.thisParam)); //calculate the parameters to pass back

    return C.thisParam;     //return the parameters
}

std::vector<float> Agent::closetPointSegment(std::vector<float> Q, std::vector<float> A, std::vector<float> B) {
    std::vector<float> QMinusA = {Q[0]-A[0], Q[1] - A[1]};      //some math for later use
    std::vector<float> BMinusA = {B[0]-A[0], B[1] - A[1]};


    dotOverDot = dotProduct(QMinusA, BMinusA) / dotProduct(BMinusA, BMinusA);       //ratio between dot products
    std::vector<float> AplusTBminusA  = {(A[0]+ (dotOverDot*(B[0]-A[0]))), (A[1]+ (dotOverDot*(B[1]-A[1])))};     //point made distance from b and a
                                                                                                                    //multiplied by ratio of dot product
                                                                                                                    //then added to vector A
    if(dotOverDot <= 0){        //if ratio between dot products is less than 0
        return A;               // vector A is closest
    }else if (dotOverDot >= 1){ //if over or equal to one
        return B;               //B is closest
    }else{                      //otherwise
        return AplusTBminusA;   //this point is closest
    }
}

float Agent::dotProduct(std::vector<float> A, std::vector<float> B){    //pass two vectors with two values to find dot product
    return (A[0]*B[0])+(A[1]*B[1]);     //return dot product
}

/**************************
AGENT STEERING FUNCTIONS
**************************/


Agent Agent::getSteeringFlee(Agent target) {  //takes in target agent returns agent with new steering values
    Agent result;       //create a result agent to hold new values

    result.linear[0] = position[0] - target.position[0];    //linear = character position - target position
    result.linear[1] = position[1] -  target.position[1];

    result.linear = normalizeVector(result.linear);     //normalize the vector

    result.linear[0] = result.linear[0] * maxAcceleration;     //multiply by characters max acceleration
    result.linear[1] = result.linear[1] * maxAcceleration;

    angular = 0;

    return result;       //return the resulting agent

}

Agent Agent::getSteeringSeek(Agent target) {  //takes in target agent returns
    Agent result;       //create a result agent to hold new values

    result.linear[0] = target.position[0] - position[0];    //linear = character position - target position
    result.linear[1] = target.position[1] - position[1];

    result.linear = normalizeVector(result.linear);     //normalize the vector

    result.linear[0] = result.linear[0] * maxAcceleration;     //multiply by characters max acceleration
    result.linear[1] = result.linear[1] * maxAcceleration;

    angular = 0;

    return result;       //return the vector

}


Agent Agent::getSteeringArrive(Agent target){   //takes in target agent returns
    Agent result;       //create a result agent to hold new values

    std::vector<float> direction = {0,0};
    direction[0]  = target.position[0] - position[0];   //linear = character position - target position
    direction[1]  = target.position[1] - position[1];

    float distance;                         //get the distance from the target
    distance = getVectorLength(direction);

    float targetSpeed;
    if(distance < targetRadius){    //if the distance is less than the target radius turn speed to 0
        targetSpeed = 0;
    }else if(distance > slowRadius){    //if distance is greater than slow radius continue at max speed
        targetSpeed = maxSpeed;
    }else{
        targetSpeed = (maxSpeed * distance) /slowRadius;    //otherwise decrement speed based on radius
    }

    std::vector<float> targetVelocity;                      //get linear velocity
    targetVelocity = normalizeVector(direction);
    targetVelocity[0] = targetVelocity[0] * targetSpeed;
    targetVelocity[1] = targetVelocity[1] * targetSpeed;

    result.linear[0] = targetVelocity[0] - velocity[0];
    result.linear[1] = targetVelocity[1] - velocity[1];
    result.linear[0] = result.linear[0] / timeToTarget;
    result.linear[1] = result.linear[1] / timeToTarget;

    if(getVectorLength(result.linear) > maxAcceleration){
        result.linear = normalizeVector(result.linear);
        result.linear[0] = result.linear[0] * maxAcceleration;
        result.linear[1] = result.linear[1] * maxAcceleration;
    }

    result.angular = 0;         //angular velocity to 0

    return result;      //return result Agent
}

Agent Agent::getSteeringFollowPath(path pathOne) {      //update steering for path following by passing in a path to follow

    currentParam = pathGetParam(pathOne);           //get the current parameters for the agent
    targetParam = min((float)1, currentParam + offset);     //get the target parameters by taking the min of 1 or the currentParam and the offset of the agent

    if (targetParam > (currentParam + offset))      //if target param is larger than the current+off calculate new agent variables
        targetParam = currentParam + offset;        //set target parameters to current + offset
    Agent target;                                   //create a dummie agent for the target
    target.position = pathGetPosition(pathOne, targetParam);    //set that agents position by passing path and target parameters to getPathPosition
    linear = getSteeringSeek(target).linear;        //set linear vector to equal seek steering result.linear
    angular = getSteeringSeek(target).angular;      //set angular value to equal seek steering result.angular
    return getSteeringSeek(target);                 //return the entire agent -- not sure if this is for real doing anything but my function cannot be
}                                                           //void as needs to pass to dynamic update as a "steering behavior"

/**************************
DYNAMIC UPDATE FOR ALL AGENTS
**************************/
void Agent::DynamicUpdate(Agent Steering, float deltaTime){
    Agent target;
    target.SetValues(1);
    //float halfTimeSq = 0.5 *deltaTime*deltaTime;  hs physics

    //position[0] = position[0] + (velocity[0]*deltaTime) + (Steering.linear[0] *halfTimeSq);   //hs physics
    //position[1] = position[1] + (velocity[1]*deltaTime) + (Steering.linear[1] *halfTimeSq);
    position[0] = position[0] + (velocity[0]*deltaTime);
    position[1] = position[1] + (velocity[1]*deltaTime);

   //orientation = orientation + (rotation * deltaTime) + (Steering.angular * halfTimeSq);  //hs physics
    orientation = orientation + (rotation * deltaTime);

    velocity[0] = velocity[0] + (Steering.linear[0] * deltaTime);
    velocity[1] = velocity[1] + (Steering.linear[1] * deltaTime);

    rotation =  rotation + (Steering.angular * deltaTime);      //rotation is = previous rotation + (angular velocity* time change)

    if(getVectorLength(velocity) > maxSpeed){   //if length of velocity vector is more than the max speed fix it
        velocity = normalizeVector(velocity);   //normalize it

        velocity[0] = velocity[0] * maxSpeed;   //multiply by max speed
        velocity[1] = velocity[1] * maxSpeed;
    }

    if(getVectorLength(velocity) < stopSpeed){  //if length of velocity vector is less than stop speed set it to 0
        velocity = {0,0};
    }


    if(rotation < stopRotate){  //if rotation is less than stop rotation bound set to 0
        rotation = 0;
    }
}

