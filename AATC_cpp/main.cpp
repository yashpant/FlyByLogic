#include "AATC.h"


int main(){

    // ***************************************************************************************************
    //  set mode
    // centralize (0), distributed (1), both (2)
    int mode = 0;

    // File for output
    ofstream log_centr, log_distr;

    // instantiate new AATC with default mission ("missions/mission.txt")
    AATC my_aatc;
    
    if (mode == 0 || mode == 2){
        log_centr.open("runtime_logs/Centralized_stats.txt");
        log_centr << "Timing Specs/Scaling for centralized AATC" << endl;
        log_centr << endl;
        log_centr << "# of Drones | Robustness | Time Taken | # of iter" << endl;
    }

    if (mode == 1 || mode == 2){
        log_distr.open("runtime_logs/Distributed_stats.txt");
        log_distr << "Timing Specs/Scaling for distributed AATC" << endl;
        log_distr << endl;

        log_distr << "alpha    : " << my_aatc.get_alpha() << endl;
        log_distr << "gamma    : " << my_aatc.get_gamma() << endl;
        log_distr << "runs     : " << my_aatc.get_runs() << endl;
        log_distr << "min_rob  : " << my_aatc.get_min_rob() << endl;
        log_distr << endl;

        log_distr << "# of Drones | Robustness | Time Taken | # of iter" << endl;
    }

    for (int i = 1; i <= 8 ; i++){
        // set number of drones
        my_aatc.set_nDrones(i);

        // formulate mission
        my_aatc.formulateMission();

        // solve centralized
        if (mode == 0 || mode == 2){
            my_aatc.solveCentralized();

            // print to file
            log_centr << "      " << i << "        " << my_aatc.get_robustness() << "     " << my_aatc.get_time_centralized() << "     " << my_aatc.get_num_iter() << endl;
        }

        // solve distributed
        if (mode == 1 || mode == 2){
            my_aatc.solveDistributed();

            // print to file
            log_centr << "      " << i << "        " << my_aatc.get_robustness() << "     " << my_aatc.get_time_distributed() << "     " << my_aatc.get_num_iter() << endl;
        }
    }


    if (mode == 0 || mode ==2){
        my_aatc.printMission(log_centr);
        log_centr.close();
    }

    if (mode == 1 || mode ==2){
        my_aatc.printMission(log_distr);
        log_distr.close();
    }

}
