#include "AATC.h"

int main(){

    cout << "Gui Interface Called!" << endl;

    // File for output
    ofstream mission_output;
    mission_output.open("../Missions/current_mission_output.txt");

    // load current_mission (missions/current_mission.txt)
    AATC gui_mission("../Missions/current_mission.txt");

    // formulate mission
    gui_mission.formulateMission();
    cout << "Formulate Mission" << endl;

    // solve mission
    gui_mission.solveCentralized();
    cout << "Solved Mission" << endl;

    // export to MATLAB for plotting
    gui_mission.printMissionOutput(mission_output);
    cout << "Exported Mission Output" << endl;

    // close file
    mission_output.close();

}