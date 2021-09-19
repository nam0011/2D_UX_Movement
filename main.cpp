#include "Agent.h"

using namespace std;
int main() {

/*
    Agent stop;
    Agent flee;
    Agent seek;
    Agent arrive;
*/

    Agent follow;

    path pathOne;

    pathOne.setPathValues(1);
    pathOne.pathPrep();

/*
    stop.SetValues(1);
    flee.SetValues(2);
    seek.SetValues(3);
    arrive.SetValues(4);
*/

    follow.SetValues(5);




    float time = 0;
    float stopTime = 100;
    float changeTime = 0.5;


    string fileName = "C:/Users/Nathan Moore/Desktop/CS 330, Dynamic trajectory data.csv";       //file location information
    ofstream outFile;

    outFile.open(fileName);  //opening output file
    cout<<endl;
    if (!outFile) {     //if the file doesn't open
        cout << "output file could not be opened please try again"; //prompt failure
        return 2;   //return error code quit program
    } else {    //if success
        cout << "Output file opened successfully." << endl
             << endl; //prompt success continue program
    }


    while(time <= stopTime) {

/*
        outFile << time << ", ";
        outFile << stop.id<<", ";
        outFile << stop.position[0] << ", ";
        outFile << stop.position[1] << ", ";
        outFile << stop.velocity[0] << ", ";
        outFile << stop.velocity[1] << ", ";
        outFile << stop.linear[0] << ", ";
        outFile << stop.linear[1] <<", ";
        outFile << stop.orientation << ", ";
        outFile << stop.steer<<", "<<endl;

        outFile << time << ", ";
        outFile << flee.id<<", ";
        outFile << flee.position[0] << ", ";
        outFile << flee.position[1] << ", ";
        outFile << flee.velocity[0] << ", ";
        outFile << flee.velocity[1] << ", ";
        outFile << flee.linear[0] << ", ";
        outFile << flee.linear[1] <<", ";
        outFile << flee.orientation << ", ";
        outFile << flee.steer<<", "<<endl;

        flee.DynamicUpdate(flee.getSteeringFlee(stop), changeTime);

        outFile << time << ", ";
        outFile << seek.id<<", ";
        outFile << seek.position[0] << ", ";
        outFile << seek.position[1] << ", ";
        outFile << seek.velocity[0] << ", ";
        outFile << seek.velocity[1] << ", ";
        outFile << seek.linear[0] << ", ";
        outFile << seek.linear[1] <<", ";
        outFile << seek.orientation << ", ";
        outFile << seek.steer<<", "<<endl;

        seek.DynamicUpdate(seek.getSteeringSeek(stop), changeTime);

        outFile << time << ", ";
        outFile << arrive.id<<", ";
        outFile << arrive.position[0] << ", ";
        outFile << arrive.position[1] << ", ";
        outFile << arrive.velocity[0] << ", ";
        outFile << arrive.velocity[1] << ", ";
        outFile << arrive.linear[0] << ", ";
        outFile << arrive.linear[1] <<", ";
        outFile << arrive.orientation << ", ";
        outFile << arrive.steer<<", "<<endl;

        arrive.DynamicUpdate(arrive.getSteeringArrive(stop), changeTime);

        */
               follow.DynamicUpdate(follow.getSteeringFollowPath(pathOne), changeTime);
                if (follow.align) {
                    follow.orientation = atan2(follow.velocity[1], follow.velocity[0]);
                }

               outFile << time << ", ";
               outFile << follow.id<<", ";
               outFile << follow.position[0] << ", ";
               outFile << follow.position[1] << ", ";
               outFile << follow.velocity[0] << ", ";
               outFile << follow.velocity[1] << ", ";
               outFile << follow.linear[0] << ", ";
               outFile << follow.linear[1] << ", " << follow.orientation << ", ";
               outFile << follow.steer<<", "<<endl;

        time+=0.5;
    }
    return 0;
}