#include "AATC.h"


int main(){

    // // test reading from file

    // // File with mission input
    // ifstream inFile;
    // string line;
    // string line1 = " ";
    // string::size_type sz;   // alias of size_t

    // int i = 0;
    // int nDrones;
    // int nGoals;
    // float Horizon;
    // float h;
    // int nObs;
    // float T;
    // float minSep;
    // vector<vector<double>> p0;
    // vector<vector<double>> obs;
    // vector<vector<double>> goal;
    // vector<vector<double>> droneGoal;

    // // open mission file
    // inFile.open("missions/mission.txt");

    // // cout << string::npos << endl;
    // while (!inFile.eof()){
    //     getline(inFile, line);

    //     if ((line.rfind("***")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("#")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("---")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("Details")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("Goals")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("Obstacles")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("Goal Intervals")!=string::npos)){
    //         continue;
    //     }

    //     if ((line.rfind("Number of Drones")!=string::npos)){
    //         i = line.find(":");
    //         nDrones = stoi(line.substr(i+1));
    //         continue;
    //     }

    //     if ((line.rfind("Mission Horizon")!=string::npos)){
    //         i = line.find(":");
    //         Horizon = stof(line.substr(i+1));
    //         continue;
    //     }

    //     if ((line.rfind("Waypoint Interval")!=string::npos)){
    //         i = line.find(":");
    //         T = stof(line.substr(i+1));
    //         continue;
    //     }

    //     if ((line.rfind("Time step")!=string::npos)){
    //         i = line.find(":");
    //         h = stof(line.substr(i+1));
    //         continue;
    //     }

    //     if ((line.rfind("Minimum Separation")!=string::npos)){
    //         i = line.find(":");
    //         minSep = stof(line.substr(i+1));
    //         continue;
    //     }

    //     if ((line.rfind("Drone")!=string::npos)){
    //         vector<double> p0_;
    //         string line_ = line.substr(1+line.find("["));
    //         std::string::size_type sz;

    //         for (int j = 0; j < 3; j++){
    //             p0_.push_back(stof(line_, &sz));
    //             line = line.substr(sz+1);
    //         }

    //         p0.push_back(p0_);
    //         cout << p0_ << endl;
    //         continue;
    //     }

    //     if ((line.rfind("Spec")!=string::npos)){
    //         vector<double> droneGoal_;
    //         string line_ = line.substr(1+line.find("["));
    //         std::string::size_type sz;

    //         for (int j = 0; j < 4; j++){
    //             droneGoal_.push_back(stof(line_, &sz));
    //             line = line.substr(sz+1);
    //         }

    //         droneGoal.push_back(droneGoal_);
    //         cout << droneGoal_ << endl;
    //         continue;
    //     }

    //     if ((line.rfind("Goal")!=string::npos)){
    //         vector<double> goal_;
    //         string line_ = line.substr(1+line.find("["));
    //         std::string::size_type sz;

    //         for (int j = 0; j < 6; j++){
    //             goal_.push_back(stof(line_, &sz));
    //             line = line.substr(sz+1);
    //         }

    //         goal.push_back(goal_);
    //         cout << goal_ << endl;
    //         continue;
    //     }

    //     if ((line.rfind("Obstacle")!=string::npos)){
    //         vector<double> obs_;
    //         string line_ = line.substr(1+line.find("["));
    //         std::string::size_type sz;

    //         for (int j = 0; j < 6; j++){
    //             obs_.push_back(stof(line_, &sz));
    //             line = line.substr(sz+1);
    //         }

    //         obs.push_back(obs_);
    //         cout << obs_ << endl;
    //         continue;
    //     }

        

    //     cout << line << endl;
    // }

    // cout << "nDrones : " << nDrones << endl;
    // cout << "initial pos : " << p0 << endl;
    // cout << "obs : " << obs << endl;
    // cout << "goals : " << goal << endl;




    // ***************************************************************************************************
    // //  set mode
    // // centralize (0), distributed (1), both (2)
    // int mode = 0;

    // // File for output
    // ofstream log_centr, log_distr;

    // instantiate new AATC with default mission
    AATC my_aatc("missions/mission.txt");
    
    // if (mode == 0 || mode == 2){
    //     log_centr.open("runtime_logs/Centralized_stats.txt");
    //     log_centr << "Timing Specs/Scaling for centralized AATC" << endl;
    //     log_centr << endl;
    //     log_centr << "# of Drones | Robustness | Time Taken | # of iter" << endl;
    // }

    // if (mode == 1 || mode == 2){
    //     log_distr.open("runtime_logs/Distributed_stats.txt");
    //     log_distr << "Timing Specs/Scaling for distributed AATC" << endl;
    //     log_distr << endl;

    //     log_distr << "alpha    : " << my_aatc.get_alpha() << endl;
    //     log_distr << "gamma    : " << my_aatc.get_gamma() << endl;
    //     log_distr << "runs     : " << my_aatc.get_runs() << endl;
    //     log_distr << "min_rob  : " << my_aatc.get_min_rob() << endl;
    //     log_distr << endl;

    //     log_distr << "# of Drones | Robustness | Time Taken | # of iter" << endl;
    // }

    // for (int i = 1; i <= 8 ; i++){
    //     // set number of drones
    //     my_aatc.set_nDrones(i);

    //     // formulate mission
    //     my_aatc.formulateMission();

    //     // solve centralized
    //     if (mode == 0 || mode == 2){
    //         my_aatc.solveCentralized();

    //         // print to file
    //         log_centr << "      " << i << "        " << my_aatc.get_robustness() << "     " << my_aatc.get_time_centralized() << "     " << my_aatc.get_num_iter() << endl;
    //     }

    //     // solve distributed
    //     if (mode == 1 || mode == 2){
    //         my_aatc.solveDistributed();

    //         // print to file
    //         log_centr << "      " << i << "        " << my_aatc.get_robustness() << "     " << my_aatc.get_time_distributed() << "     " << my_aatc.get_num_iter() << endl;
    //     }
    // }


    // if (mode == 0 || mode ==2){
    //     my_aatc.printMission(log_centr);
    //     log_centr.close();
    // }

    // if (mode == 1 || mode ==2){
    //     my_aatc.printMission(log_distr);
    //     log_distr.close();
    // }

}
