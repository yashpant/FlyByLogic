#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Eigen>
#include <time.h>

#define C 50.0

using namespace casadi;
using namespace std;
using namespace Eigen;

// struct to define mission parameters
struct missionStruct {
    int nDrones;                        // number of drones in mission
    float Horizon;                      // length of mission in seconds
    float h;                            // sampling time
    float minSep;                       // time step intervals
    vector<vector<double>> obs;         // obstacles
    vector<vector<double>> goal;        // goal
    vector<vector<double>> droneGoal;   // goal intervals
    float k1t;                          // spline constant 1
    float k2t;                          // spline constant 2
    int npt;                            // waypoints per time interval
    float T;                            // time interval between major waypoints
    vector<vector<double>> p0;          // initial positions of drones
    vector<vector<double>> v0;          // initial velocities of drones
    vector<vector<double>> v_bounds;    // bounds on velocities and accelerations of drones
    vector<double> map_boundary;

    DM M;                               // min jerk trajectory spline charateristic matrix
    DM A;                               // for distributed implementation (experimental)
    DM Hmat;                            // for distributed implementation (experimental)
    ArrayXXi sched;                     // for distributed implementation (experimental)
};

struct mission {
    MX f;     // objective function
    MX x;     // decision variables    
    MX g;     // system constraints
    DM x0;    // intial solution
    DM lbx;   // lower bounds on x
    DM ubx;   // upper bounds on x
    DM lbg;   // lower bounds on g
    DM ubg;   // upper bounds on g
};

// output type of missionRob
template <typename T>
struct FuncOut {
    T rob;
    T sepRob;
    T goalRob;
    T unsafeRob;
    T traj_length;
};

class AATC {
    public:

        // constructors
        AATC();

        AATC(string filename);

        AATC(int n_drones, float horizon, float dt, float DT, float min_sep, vector<vector<double>> goal, vector<vector<double>> goal_intervals, vector<vector<double>> obs);

        ~AATC(){
            cout << "Destroyed AATC object" << endl;
        }
        
        ArrayXXi getPairCombos(int N);

        int getIndex(float t);

        template <typename T>
        T smoothMinVec(T x, double c=C);

        template <typename T>
        T smoothMin(T x, double c=C);

        template <typename T>
        T smoothMax(T x, double c=C);

        template <typename T>
        T inSet(T xx, T yy, T zz, vector<double> set);
        
        template <typename T>
        T always_in(T xx, T yy, T zz, vector<double> set);

        template <typename T>
        T always_not_in(T xx, T yy, T zz, vector<double> set);

        template <typename T>
        T eventually_in(T xx, T yy, T zz, vector<double> set);

        template <typename T>
        T always_eventually(T xx, T yy, T zz, vector<double> set, float a, float b, float c, float d);

        template <typename T>
        T eventually_always(T xx, T yy, T zz, vector<double> set, float a, float b, float c, float d);

        template <typename T>
        T unsafeRob(T xx, T yy, T zz, vector<vector<double>> obs);

        template <typename T>
        T goalRob(T xx, T yy, T zz, vector<vector<double>> goal, vector<vector<double>> droneGoal);

        template <typename T>
        T sepRob(T xx, T yy, T zz, double minSep);

        template <typename in_type>
        void missionRob(in_type var);

        void formulateMission();

        void solveCentralized();

        void solveDistributed();

        void printMission(ostream& out);

        void printMissionOutput(ostream& out);

        void set_nDrones(int n);

        double get_robustness();

        double get_time_centralized();

        double get_time_distributed();

        float get_alpha();

        float get_gamma();

        int get_runs();

        int get_num_iter();

        int get_max_iter();

        float get_min_rob();

        void set_max_iter(int n=200);

    private:
        float lambda;
        bool boolean_mode;
        float max_rob = inf;
        float min_rob = 0.05;
        float time_centralized;
        float time_distributed;
        int num_iter = 0;
        clock_t t, w, q;

        missionStruct optParams;
        mission myMission;
        FuncOut<MX> mxOut;
        FuncOut<DM> dmOut;

        // distributes AATC variables
        //SpDict qp;
        Dict qpoasesOpts;
        DMDict res;

        vector<DM> lbx_vec;
        vector<DM> ubx_vec;

        vector<DM> lbx_pos;
        vector<DM> ubx_pos;

        vector<DM> lbx_vel;
        vector<DM> ubx_vel;

        vector<DM> lba;
        vector<DM> uba;
        vector<DM> grad_vec, grad_pos, grad_vel, x0_vec, x0_pos, x0_vel;

        DM x0Cur, lbxCur, ubxCur, lbaCur, ubaCur, gradCur;
        //Function S = conic("S", "qpoases", qp, qpoasesOpts);
        Function f;
        Function fp;

        //DMDict args_qp;
        //DMDict sol;
        //DMDict jacob;

        vector<DM> tempSol;
        vector<DM> tempSol_vec;
        
        // distributed AATC params
        float alpha = 0.001;
        float gam = 0.95;
        int max_iter = 500;
        int runs = 4;

        bool loadMission(string filename);

};