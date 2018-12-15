/*
 *    This file implements all the robustness cost function
 *    
 */

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Eigen>
#include <time.h>

#define C 50.0
#define BOOLEAN 0
#define MIN_ROB 0.5
#define MAX_ROB 100

clock_t t;
clock_t q;
clock_t w;
float lambda = 0.05;

float alpha = 0.075;
float gam = 0.975;
int max_iter = 500;

using namespace casadi;
using namespace std;
using namespace Eigen;

vector<DM> lbx_vec;
vector<DM> ubx_vec;

vector<DM> lbx_pos;
vector<DM> ubx_pos;

vector<DM> lbx_vel;
vector<DM> ubx_vel;

vector<DM> lba;
vector<DM> uba;

// struct to define mission parameters
struct missionStruct {
  int nDrones;         // number of drones in mission
  float Horizon;       // length of mission in seconds
  float h;             // sampling time
  float minSep;        // time step intervals
  ArrayXXf obs;        // obstacles
  ArrayXXf goal;       // goal
  ArrayXXf droneGoal;
  float k1t;
  float k2t;
  int npt;
  float T;
  vector<vector<double>> p0;
  vector<vector<double>> v0;

  DM M;
  DM A;
  DM Hmat;
  ArrayXXi sched;
} optParams;

struct mission {
  MX f;     // objective function
  MX x;     // decision variables    
  MX g;     // system constraints
  DM x0;    // intial solution
  DM lbx;   // lower bounds on x
  DM ubx;   // upper bounds on x
  DM lbg;   // lower bounds on g
  DM ubg;   // upper bounds on g
} myMission;

// output type of missionRob
template <typename T>
struct FuncOut {
  T rob;
  T sepRob;
  T goalRob;
  T unsafeRob;
  T traj_length;
};

FuncOut<MX> mxOut;
FuncOut<DM> dmOut;

ArrayXXi getPairCombos(int N){
  // return the list of all combinations of pairs given N items
  int M = N*(N-1)/2;

  int a = 1;
  int b = 2;

  ArrayXXi temp(M,2);
  for(int i = 0; i < M; i++){
    temp.block(i,0,1,2) << a,b;

    if (b == N){
      a++;
      b = a + 1;
    }
    else{
      b++;
    }
  }

  // adjust for c++ 0 indexing
  return temp - 1;
}

int getIndex(float t){
  //cout << "t : " << t << endl;
  //cout << "npt : " << optParams.npt << endl;
  if (t >= optParams.Horizon){
    return (int) floor(optParams.Horizon * optParams.npt - 1);
  }
  // cout << "hey" << endl;
  return (int) (t*optParams.npt);
}

template <typename T>
T smoothMin(T x, double c=C){
  // return the smooth min of a vector
  int n = x.size1();
  int m = x.size2();

  // return log(exp(x).transpose()*MX::ones(size))
  return -log(dot(mtimes(exp(-c*x),T::ones(m,1)), T::ones(n,1)))/c;
}


template <typename T>
T smoothMax(T x, double c=C){
  // return the smooth max of a vector
  int n = x.size1();
  int m = x.size2();

  return log(dot(mtimes(exp(c*x),T::ones(m,1)), T::ones(n,1)))/c;
}

template <typename T>
T smoothMinMax(T x, double c=C){
  // return the smooth max of a vector
  int n = x.size1();
  int m = x.size2();
  
  T minVec = -log(mtimes(exp(-c*x),T::ones(m,1)))/c;
  return smoothMax(minVec);
}


template <typename T>
T smoothMinMin(T x, double c=C){
  // return the smooth max of a vector
  int n = x.size1();
  int m = x.size2();
  
  T minVec = -log(mtimes(exp(-c*x),T::ones(m,1)))/c;
  return smoothMin(-minVec);
}


template <typename T>
T always_in(T xx, T yy, T zz, ArrayXXf set){
  // return the robustness of a path always in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // obs is of size 1 x 6

  vector<T> temp;

  // compute distance of drone from set at each point
  temp.push_back(xx-set(0,0));
  temp.push_back(yy-set(0,1));
  temp.push_back(zz-set(0,2));
  temp.push_back(set(0,3)-xx);
  temp.push_back(set(0,4)-yy);
  temp.push_back(set(0,5)-zz);

  return smoothMin(horzcat(temp));
}

template <typename T>
T always_not_in(T xx, T yy, T zz, ArrayXXf set){
  // return the robustness of a path always in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // obs is of size 1 x 6

  vector<T> temp;

  // compute distance of drone from set at each point
  temp.push_back(xx-set(0,0));
  temp.push_back(yy-set(0,1));
  temp.push_back(zz-set(0,2));
  temp.push_back(set(0,3)-xx);
  temp.push_back(set(0,4)-yy);
  temp.push_back(set(0,5)-zz);

  return smoothMinMin(horzcat(temp));
}

template <typename T>
T eventually_in(T xx, T yy, T zz, ArrayXXf set){
  // return the robustness of a path always in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // obs is of size 1 x 6

  vector<T> temp;

  // compute distance of drone from set at each point
  temp.push_back(xx-set(0,0));
  temp.push_back(yy-set(0,1));
  temp.push_back(zz-set(0,2));
  temp.push_back(set(0,3)-xx);
  temp.push_back(set(0,4)-yy);
  temp.push_back(set(0,5)-zz);

  return smoothMinMax(horzcat(temp));
}

template <typename T>
T always_eventually(T xx, T yy, T zz, ArrayXXf set, float a, float b, float c, float d){
  // STL formula
  // ALWAYS (a, b) EVENTUALLY (c, b) BE_in_SET

  // return the robustness of a path always eventually in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // obs is of size 1 x 6

  vector<T> temp;
  T x, y, z;

  float t = 0,dt = optParams.h;

  // start and end indices
  int iS, iE;
  for (t = a; t <= b; t += dt){
    iS = getIndex(t+c);
    iE = getIndex(t+d);

    // extract trajectory path
    x = xx(Slice(iS, iE));
    y = yy(Slice(iS, iE));
    z = zz(Slice(iS, iE));
    
    temp.push_back(eventually_in(x, y, z, set));
  }

  return smoothMin(vertcat(temp));
}

template <typename T>
T eventually_always(T xx, T yy, T zz, ArrayXXf set, float a, float b, float c, float d){
  // STL formula
  // ALWAYS (a, b) EVENTUALLY (c, b) BE_in_SET

  // return the robustness of a path always eventually in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // obs is of size 1 x 6

  vector<T> temp;
  T x, y, z;

  float dt = optParams.h;

  // start and end indices
  int iS, iE;
  for (float t = a; t <= b; t += dt){
    iS = getIndex(t+c);
    iE = getIndex(t+d);

    // extract trajectory path
    x = xx(Slice(iS, iE));
    y = yy(Slice(iS, iE));
    z = zz(Slice(iS, iE));
    
    temp.push_back(always_in(x, y, z, set));
  }

  return smoothMax(vertcat(temp));
}


template <typename T>
T unsafeRob(T xx, T yy, T zz, ArrayXXf obs){
  // return the robustness of all paths with respect to all obstacles
  // xx is of size T x N
  // yy is of size T x N
  // zz is of size T x N
  // obs is of size M x 6
  
  // Algorithm
  
  // get number of obstacles
  int M = obs.rows();
  // cout << "Number of Obs: " << M << endl;

  vector<T> out;

  // variable to store current drone 
  T x, y, z;

  // for each obstacle
  for(int j = 0; j < M; j++)
  {
    // for each drone
    for(int k = 0; k < optParams.nDrones; k++){

      // extract kth drone path
      x = xx(Slice(),k);
      y = yy(Slice(),k);
      z = zz(Slice(),k);

      out.push_back(always_not_in(x, y, z, obs.row(j)));
    }
  }

  return smoothMin(vertcat(out));
}

template <typename T>
T goalRob(T xx, T yy, T zz, ArrayXXf goal, ArrayXXf droneGoal){
  // return the robustness of a path with respect to a desired goal
  // Limited implementation
  // no specific intervals
  // Algorithm
  
  // how many drones
  int N = xx.size2();
  int npt = optParams.npt;
  float Time = optParams.T;

  // container to store all drone-goal distances
  vector<T> out;

  // hold the path coordinates of drone
  T x, y, z;

  // set a counter to 1st drone
  int j = 0;

  // keep track of current goal
  int k = 0;
  int iS, iE;

  // for each drone
  for(int i = 0; i < N; i++){
  
    // iterate through all goals ith drone visits
    while(i == droneGoal(j,0)){
      
      // cout << droneGoal.row(i) << endl;
      //extract start and end intervals for current goal
      k = int(droneGoal(j, 1));
      iS = int(droneGoal(j, 2)*npt/Time);
      iE = int(droneGoal(j, 3)*npt/Time);
      
      // extract relevant interval of ith drone path
      x = xx(Slice(iS, iE),i);
      y = yy(Slice(iS, iE),i);
      z = zz(Slice(iS, iE),i);

      //cout << goal(k) << endl;
      out.push_back(eventually_in(x, y, z, goal));

      if(j < droneGoal.rows()-1){
        j++;
      }
      else{
        break;
      }
  
    }
  }

  return smoothMin(vertcat(out));
}

template <typename T>
T sepRob(T xx, T yy, T zz, double minSep){
  // return the robustess
  // xx, yy, zz are all of size Traj_len x N
  // Traj_len is the number of time steps
  // N is the number of drones
  
  int N = xx.size2();
  ArrayXXi pairs = getPairCombos(N);
  int p1, p2;
  T pathA, pathB, dPath, mu;
  vector<T> out;

  for (int i = 0; i < pairs.rows(); i++){
    p1 = pairs(i,0); p2 = pairs(i,1);

    pathA = horzcat(xx(Slice(),p1), yy(Slice(),p1), zz(Slice(),p1));
    pathB = horzcat(xx(Slice(),p2), yy(Slice(),p2), zz(Slice(),p2));
    dPath = pathA-pathB;

    // compute rowwise norm
    mu = sqrt(sum2(pow(dPath,2))) - minSep;
    out.push_back(smoothMin(mu));
  }

  return smoothMin(vertcat(out));
}

template <typename in_type>
FuncOut<in_type> missionRob(in_type var){
  // extract some mission details
  int H = optParams.Horizon;
  int npt = optParams.npt;
  int nDrones = optParams.nDrones;
  DM M = optParams.M;
  int N = var.size1();
  float T = optParams.T;
  double dv = 0, da = 0;

  //cout << endl << "N : " << N << endl;
  //cout << "nDrones : " << nDrones << endl;

  vector<double> dT_;
  double step = T/npt;

  // generate mx matrix of time step
  for(double dt = step; dt < T+step; dt+=step){
    dT_.push_back(dt);
  }

  DM dT = casadi::Matrix<double>(dT_).T();
  DM B = vertcat(pow(dT,5), 5*pow(dT,4), 20*pow(dT,3))/120;

  // split var into vector<MX> again
  vector<in_type> var_vec = vertsplit(var,N/2);
  vector<in_type> w_vec = vertsplit(var_vec[0],3);
  vector<in_type> v_vec = vertsplit(var_vec[1],3);

  // temp structure for construction of function
  vector<in_type> xx_, yy_, zz_, txyz_;
  in_type pCur, pPrev, vCur, vPrev, dp, foo, A, txyz, xx, yy, zz;
  in_type rho_sep, rho_goal, rho_unsafe;
  
  int i = 0;
  
  for(int d = 0; d < nDrones; d++){
    // initial positions
    pPrev = w_vec[i];
    vPrev = v_vec[i];

    xx_.push_back(pPrev(0));
    yy_.push_back(pPrev(1));
    zz_.push_back(pPrev(2));

    i++;

    for(int k = 0; k < H; k++){
      pCur = w_vec[i];
      vCur = v_vec[i];

      dp = pCur - pPrev - T*vPrev;
      foo = horzcat(dp,dv*DM::ones(3,1),da*DM::ones(3,1)).T();
      A = mtimes(M, foo).T();
  
      // separate output into xyz component vectors
      txyz = (mtimes(A,B) + pPrev + mtimes(vPrev,dT)).T();
      txyz_ = horzsplit(txyz);

      xx_.push_back(txyz_[0]);
      yy_.push_back(txyz_[1]);
      zz_.push_back(txyz_[2]);
      
      pPrev = pCur; vPrev = vCur;
      i++;
    }
  }
  
  vector<in_type> xxPerDrone = vertsplit(vertcat(xx_),vertcat(xx_).size1()/nDrones);
  vector<in_type> yyPerDrone = vertsplit(vertcat(yy_),vertcat(xx_).size1()/nDrones);
  vector<in_type> zzPerDrone = vertsplit(vertcat(zz_),vertcat(xx_).size1()/nDrones);

  xx = horzcat(xxPerDrone);
  yy = horzcat(yyPerDrone);
  zz = horzcat(zzPerDrone);

  rho_sep = sepRob(xx, yy, zz, optParams.minSep);
  rho_goal = goalRob(xx, yy, zz, optParams.goal, optParams.droneGoal);
  rho_unsafe = unsafeRob(xx, yy, zz, optParams.obs);

  // compute total length of trajectory
  in_type traj_length = 0;
  in_type xyz, xyz1, xyz2;

  for (int i = 0; i < nDrones; i++){
    xyz = horzcat(xxPerDrone[i], yyPerDrone[i], zzPerDrone[i]);
    xyz1 = xyz(Slice(0,-2),Slice());
    xyz2 = xyz(Slice(1,-1),Slice());

    traj_length = traj_length + sum1(sum2(pow((xyz1 - xyz2),2)));
  }

  in_type rob =  smoothMin(vertcat(rho_unsafe, rho_goal, rho_sep));
  
  FuncOut<in_type> out;

  out.rob = rob;
  out.sepRob = rho_sep;
  out.goalRob = rho_goal;
  out.unsafeRob = rho_unsafe;
  out.traj_length = traj_length;

  return out;
}

mission formulateMission(){
  // Formulation begins here (Needs all the inputs above)
  int nDrones = optParams.nDrones;
  float H = optParams.Horizon;
  float h = optParams.h;
  float T = optParams.T;
  float minSep = optParams.minSep; 
  DM M = optParams.M;
  vector<vector<double>> p0 = optParams.p0;
  vector<vector<double>> v0 = optParams.v0;

  // build the structure of the problem
  vector<MX> w, v, g;
  vector<double> lbw, ubw, lbv, ubv, lbg, ubg, lbq, ubq;

  // define map bounds -- INPUT
  double lmapX = -2, lmapY = -2, lmapZ = -0, umapX = 2, umapY = 2, umapZ = 2;

  MX pCur, pPrev, vCur, vPrev, dp, foo, mCons, vf;
  double dv = 0, da = 0, maxVel = 20, maxAcc = 20; 

  double k1t = optParams.k1t;
  double k2t = optParams.k2t;

  maxAcc = 1;
  maxVel = 0.751;

  vector<double> lbg_ = {-maxVel,-maxVel,-maxVel,-maxAcc,-maxAcc,-maxAcc,0,0,0};
  vector<double> ubg_ = {+maxVel,+maxVel,+maxVel,+maxAcc,+maxAcc,+maxAcc,0,0,0};
  vector<double> lbw_ = {lmapX, lmapY, lmapZ};
  vector<double> ubw_ = {umapX, umapY, umapZ};
  vector<double> lbv_ = {-maxVel, -maxVel, -maxVel};
  vector<double> ubv_ = {+maxVel, +maxVel, +maxVel};
  vector<double> goall = {1.75, 1.75, 0.75};
  vector<double> goalu = {2, 2, 1};
  vector<double> zil_ = {0, 0, 0};

  // construct T vector
  vector<double> Tvec_ = {pow(T,4)/24, pow(T,3)/6, pow(T,2)/2};
  MX Tvec = MX(Tvec_);
 
  stringstream pname, vname;

  for(int d = 0; d < nDrones; d++){
    lbw.insert(lbw.end(), p0[d].begin(), p0[d].end());
    ubw.insert(ubw.end(), p0[d].begin(), p0[d].end());

    lbv.insert(lbv.end(), v0[d].begin(), v0[d].end());
    ubv.insert(ubv.end(), v0[d].begin(), v0[d].end());

    pname << "p" << d << "0";
    vname << "v" << d << "0";

    pPrev = MX::sym(pname.str(),3,1);
    vPrev = MX::sym(pname.str(),3,1);

    w.push_back(pPrev);
    v.push_back(vPrev);

    pname.str(""); vname.str("");

    for(int k = 0; k < H; k++){
      pname << "p" << d << "" << k+1;
      vname << "v" << d << "" << k+1;

      pCur = MX::sym(pname.str(),3,1);
      vCur = MX::sym(vname.str(),3,1);

      w.push_back(pCur);
      lbw.insert(lbw.end(), lbw_.begin(), lbw_.end());
      ubw.insert(ubw.end(), ubw_.begin(), ubw_.end());
      
      v.push_back(vCur);
      if(k < H-1){
        lbv.insert(lbv.end(), lbv_.begin(), lbv_.end());
        ubv.insert(ubv.end(), ubv_.begin(), ubv_.end());
      }
      else{
        lbv.insert(lbv.end(), zil_.begin(), zil_.end());
        ubv.insert(ubv.end(), zil_.begin(), zil_.end());
      }

      pname.str(""); vname.str("");

      // dp for all axes
      dp = pCur - pPrev - T*vPrev;
      foo = horzcat(dp,dv*MX::ones(3,1),da*MX::ones(3,1)).T();
      mCons = mtimes(M, foo);

      vf = mtimes(mCons.T(),Tvec) + vPrev;

      g.push_back(k1t*dp + vPrev);  // vel constraints on dp
      g.push_back(k2t*dp);      // acc constraints
      g.push_back(vCur - vf);

      lbg.insert(lbg.end(), lbg_.begin(), lbg_.end());
      ubg.insert(ubg.end(), ubg_.begin(), ubg_.end());

      pPrev = pCur;
      vPrev = vCur;
    }
  }
  
  // join v to end of w
  w.insert(w.end(), v.begin(), v.end());
  lbw.insert(lbw.end(), lbv.begin(), lbv.end());
  ubw.insert(ubw.end(), ubv.begin(), ubv.end());

  MX var = vertcat(w);    // this is x
  mxOut = missionRob(var);
  MX f;  
  // get the waypoints
  vector<MX> var_ = vertsplit(var, var.size1()/2);
  
  // define QP
  MXDict qp;
  qp["x"] = var;
  qp["f"] = sum1(sum2(pow(var_[0] - repmat(DM(goall),nDrones*(H+1),1), 2)));
  qp["g"] = vertcat(g);

  DMDict qp_args;
  qp_args["lbg"] = lbg;
  qp_args["ubg"] = ubg;
  qp_args["lbx"] = lbw;
  qp_args["ubx"] = ubw;

  Dict qpoasesOpts;
  qpoasesOpts["printLevel"] = "none";

  Function init_solver =  qpsol("init", "qpoases", qp, qpoasesOpts);
  cout << "HELLO THERE" << endl;
  DMDict init_res = init_solver(qp_args);
  
  cout << endl;
  cout << "----------------------------------------------------------------------------" << endl;
  cout << "x0 = " << init_res.at("x") << ";" << endl;
  cout << "----------------------------------------------------------------------------" << endl;


  if (BOOLEAN){
    f = 0*mxOut.traj_length;
    g.push_back(mxOut.rob);
    lbg.push_back(MIN_ROB);
    ubg.push_back(MAX_ROB);
  }
  else{
    f = lambda*mxOut.traj_length - mxOut.rob;
  }

  mission out;

  out.f = f;
  out.x = var;
  out.g = vertcat(g);
  out.x0 = init_res.at("x");
  out.lbx = lbw;
  out.ubx = ubw;
  out.lbg = lbg;
  out.ubg = ubg;
  
  int nVar = var.size1();
  int nG = DM(lbg).size1();

  lbx_vec = vertsplit(DM(lbw), nVar/2);
  ubx_vec = vertsplit(DM(ubw), nVar/2);

  lbx_pos = vertsplit(lbx_vec[0], 3*(H+1));
  ubx_pos = vertsplit(ubx_vec[0], 3*(H+1));

  lbx_vel = vertsplit(lbx_vec[1], 3*(H+1));
  ubx_vel = vertsplit(ubx_vec[1], 3*(H+1));

  lba = vertsplit(DM(lbg), nG/nDrones);
  uba = vertsplit(DM(ubg), nG/nDrones);

  return out;
}

void solveCentalized(){
  
  MXDict prob;
  prob["f"] = myMission.f;
  prob["x"] = myMission.x;
  prob["g"] = myMission.g;

  DMDict args;
  args["x0"] = myMission.x0; //init_res.at("x");
  args["lbx"] = myMission.lbx;
  args["ubx"] = myMission.ubx;
  args["lbg"] = myMission.lbg;
  args["ubg"] = myMission.ubg;


  Dict Opts, ipoptOpts;
  ipoptOpts["linear_solver"] = "mumps";
  ipoptOpts["print_level"] = 0;
  ipoptOpts["acceptable_tol"] = 1e-6;
  ipoptOpts["tol"] = 1e-6;
  ipoptOpts["max_iter"] = 5000;
  ipoptOpts["hessian_approximation"] = "limited-memory";
  Opts["ipopt"] = ipoptOpts;

  Function solver = nlpsol("solver", "ipopt", prob, Opts);  

  t = clock();
  DMDict res = solver(args);
  t = clock() - t;

  // get the trajectories and individual robustness
  dmOut = missionRob(res.at("x"));
  cout << endl;
  cout << "Optimal cost:                     " << double(res.at("f")) << endl;
  cout << "smooth robustness :               " << dmOut.rob << endl;
  cout << "separation_robustness :           " << dmOut.sepRob << endl;
  cout << "goal_robustness :                 " << dmOut.goalRob << endl;
  cout << "unsafe_robustness :               " << dmOut.unsafeRob << endl;
  cout << "total trajectory length :         " << dmOut.traj_length << endl;


  float time_taken = ((float)t)/CLOCKS_PER_SEC;
  cout << endl;
  cout << "----------------------------------------------------------------------------" << endl;
  cout << "cpp_out = " << res.at("x") << ";" << endl;
  cout << "----------------------------------------------------------------------------" << endl;
  cout << "Solving took "<< time_taken <<" second(s)."<< endl;
}

void solveDistributed(){

  DM x =  myMission.x0;
  int dn = 0;
  int nVar = x.size1();
  int nG = myMission.lbg.size1();

  int H = optParams.Horizon;
  int nDrones = optParams.nDrones;

  // Start  -----------------------------------------------------------------------

  /*
    int K --> iteration step (previous k state variables held fixed)
    x = [x_pos, x_vel]
    x_pos = [d1_pos_0 ... d1_pos_K ... d1_pos_H,  d2_pos_0 ... d2_pos_K.. d2_pos_H, ... , dn_pos_0 ... dn_pos_K.. dn_pos_H]
    x_vel = [d1_vel_0 ... d1_vel_K ... d1_vel_H,  d2_vel_0 ... d2_vel_K.. d2_vel_H, ... , dn_vel_0 ... dn_vel_K.. dn_vel_H]
  */

  // Initialize qp problem
  SpDict qp;
  DMDict args_qp;
  Dict qpoasesOpts;
  qp["h"] = optParams.Hmat.sparsity();
  qp["a"] = optParams.A.sparsity();
  args_qp["h"] = optParams.Hmat*alpha/2;
  args_qp["a"] = optParams.A;
  qpoasesOpts["printLevel"] = "none";

  Function S = conic("S", "qpoases", qp, qpoasesOpts);

  DMDict sol;
  DMDict input;
  DMDict jacob;

  ArrayXXi sched = optParams.sched;
  Function fp = Function("f",{myMission.x}, {myMission.f}, {"x"}, {"q"}).jacobian();
  vector<DM> tempSol;
  vector<DM> tempSol_vec;

  t = clock();

  int nWayPoints = 1;//(H+1);
  vector<DM> grad_vec, grad_pos, grad_vel, x0_vec, x0_pos, x0_vel;
  DM x0Cur, lbxCur, ubxCur, lbaCur, ubaCur, gradCur;
  dmOut = missionRob(x);
  // online implementation wrapper
  for (int k =0; k < nWayPoints; k++)
  {
    w = clock();
    // PSCA Algorithm loop
    for (int r = 0; r < max_iter; r++){
      // serial implementation of BCD
      for (int i = 0; i < sched.rows(); i++){
        // select drones for update
        dn = sched(i,r % sched.cols()) - 1;

        // cout << "HERE" << endl;
        // compute gradient of system
        input["x"] = x;
        jacob = fp(input);
        // cout << "HERE" << endl;
        // section out x0
        x0_vec = vertsplit(x, nVar/2);
        x0_pos = vertsplit(x0_vec[0], 3*(H+1));
        x0_vel = vertsplit(x0_vec[1], 3*(H+1));

        // section out gradient
        grad_vec = vertsplit(jacob["jac"].T(), nVar/2);
        grad_pos = vertsplit(grad_vec[0], 3*(H+1));
        grad_vel = vertsplit(grad_vec[1], 3*(H+1));

        // select variables and constraints corresponding to drone dn
        x0Cur = vertcat(x0_pos[dn], x0_vel[dn]);
        lbxCur = vertcat(lbx_pos[dn], lbx_vel[dn]);
        ubxCur = vertcat(ubx_pos[dn], ubx_vel[dn]);
        gradCur = vertcat(grad_pos[dn], grad_vel[dn]) - x0Cur*alpha;
        lbaCur = lba[dn];
        ubaCur = uba[dn];
        
        args_qp["g"] = gradCur;
        args_qp["lba"] = lbaCur;
        args_qp["uba"] = ubaCur;
        args_qp["lbx"] = lbxCur;
        args_qp["ubx"] = ubxCur;
        args_qp["x0"] = x0Cur;

        q = clock();
        sol = S(args_qp);
        q = clock() - q;

        // store solutions of current block
        tempSol.push_back(sol["x"]);
      }
      
      // update x0 vector
      for (int i = 0; i < sched.rows(); i++){
        tempSol_vec = vertsplit(tempSol[i], tempSol[i].size1()/2);
        dn = sched(i,r % sched.cols()) - 1;
        x0_pos[dn] = x0_pos[dn] + pow(gam,r)*(tempSol_vec[0] - x0_pos[dn]);
        x0_vel[dn] = x0_vel[dn] + pow(gam,r)*(tempSol_vec[1] - x0_vel[dn]);
      }
      
      // stitch the trajectory back
      x = vertcat(vertcat(x0_pos), vertcat(x0_vel));

      // clear the temp container
      tempSol.erase(tempSol.begin(), tempSol.end());

      // check the robustness value every 20 iterations
      if (r % 20 == 0){
        //dmOut = missionRob(x);
        cout << "rob : " << dmOut.rob <<  "  iter : " << r << " Sol_time : " << ((float)q)/CLOCKS_PER_SEC << endl;
      }

      // exit after minimum robustnes acheived
      if (0*double(dmOut.rob) > 0.15 || r > 40){
        break;
        // dmOut.rob = 0.0;
      }
    }

    // publish kth waypoint
    // cout << endl << "Publishing waypoint " << k << endl;

    // fix the kth waypoint for all drones
    // for (int d = 0; d < nDrones; d++)
    // {
    //   lbx_pos[d](Slice(3*k,3*k+3)) = x0_pos[d](Slice(3*k,3*k+3)) - 0.0000100;
    //   ubx_pos[d](Slice(3*k,3*k+3)) = x0_pos[d](Slice(3*k,3*k+3)) + 0.0000100;
    //   lbx_vel[d](Slice(3*k,3*k+3)) = x0_vel[d](Slice(3*k,3*k+3)) - 0.0000100;
    //   ubx_vel[d](Slice(3*k,3*k+3)) = x0_vel[d](Slice(3*k,3*k+3)) + 0.0000100;
    // }

    // wait for time
    w = clock() - w;
    cout << "Time Spent : " << ((float)w)/CLOCKS_PER_SEC << endl;

    // check out first drone
    /*
    cout << endl;
    cout << "----------------------------------------------------------------------------" << endl;
    cout << "x :" << x0_pos[0] << endl;
    cout << "lb :" << lbx_pos[0].size() << endl;
    cout << "ub :" << ubx_pos[0].size() << endl;
    */
  }

  // End ------------------------------------------------------------------------------

  t = clock() - t;

  // compute robustness
  dmOut = missionRob(x);
  cout << "rob : " << dmOut.rob << endl;

  float time_taken = ((float)t)/CLOCKS_PER_SEC;
  cout << endl;
  cout << "----------------------------------------------------------------------------" << endl;
  cout << "cpp_out = " << x << ";" << endl;
  cout << "----------------------------------------------------------------------------" << endl;
  cout << "Solving took "<< time_taken <<" second(s)."<< endl;
}

/*

*/

int main(){
  
  // Main Function 

  // Formulate Reach Avoid Mission
  // -----------------------------------------------------------------------------------

  // Horizon -- INPUT
  int H = 9;                // mission horizon (s)
  optParams.Horizon = H;

  // Spline Sampling -- INPUT
  float h = 1.0/20.0;           // sampling rate (s)
  optParams.h = h;

  // Big Sampling (Pre Spline) -- INPUT
  float T = 1.0;
  optParams.T = T;

  // # of waypoints per T -- INPUT
  optParams.npt = T/h;

  // define number of drones -- INPUT
  int nDrones = 1;
  optParams.nDrones = nDrones;

  // define drone separation -- INPUT
  float minSep = 0.1;
  optParams.minSep = minSep;
  
  // define initial position and velocity of Drones -- INPUT
 // vector<vector<double>> p0 = {{-1.25,-1.25,1.75},{1.25,-1.25,1.75},{1.25,1.25,1.75},{0,1.25,1.75},{1.25,0,1.75},{0,-1.25,1.75}};
  vector<vector<double>> p0 = {{-1.25,-1.25,0.0},{1.25,-1.25,0.0},{1.25,1.25,1.75},{0,1.25,1.75},{1.25,0,1.75},{0,-1.25,1.75}};
  vector<vector<double>> v0 = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  optParams.p0 = p0;
  optParams.v0 = v0;

  // define obstacle -- INPUT
  ArrayXXf obs(1,6);
  obs << -1.0, -1.0, 0.0, 1.0, 1.0, 1.50;
  optParams.obs = obs;

  // define goals -- INPUT
  ArrayXXf goal(1,6);
  goal << 1.25, 1.25, 0.25,  2.25, 2.25, 1.25;
  optParams.goal = goal;

  // define drone-goal intervals -- INPUT
  ArrayXXf droneGoal(2,4);
  droneGoal << 0, 0, 0, 9,
               1, 0, 0, 9;
  optParams.droneGoal = droneGoal;
  
  // dynamics matrix
  vector<vector<double>> M_ = {{         90, 0, -15*pow(T,2)},
                               {      -90*T, 0,  15*pow(T,3)},
                               {30*pow(T,2), 0,  -3*pow(T,4)}};
  DM M = 0.5*DM(M_)/pow(T,5);
  optParams.M = M;

  // spline constant 1
  double k1t =90.0/(48*T) - 90.0/(12*T) + 30.0/(4*T);
  
  // spline constant 2 setup
  double aa = 90.0/(4*pow(T,5)), bb = -90.0/(2*pow(T,4)), cc = 30.0/(2*pow(T,3));

  double tp1 = (-bb+sqrt(pow(bb,2) - 4*aa*cc))/(2*aa);
  double tp2 = (-bb-sqrt(pow(bb,2) - 4*aa*cc))/(2*aa);
  double tp = tp1;

  if (tp <= 0 || tp > T){
    tp = tp2;
  }

  // splin constant 2
  double k2t = (90.0*pow(tp,3))/(12*pow(T,5)) - (90.0*pow(tp,2))/(4*pow(T,4)) + (30.0*tp)/(2*pow(T,3));

  optParams.k1t = k1t;
  optParams.k2t = k2t;
  
  // distributed stuff

  //---------------------------------------------------------------------------------------------------//
  /*
        Distributed Stuff Begins Here
  */

  DM A = DM::zeros(9*H, 6*(H+1));
  DM Hmat = DM::eye(6*(H+1));

  DM M11 = M(0,0), M21 = M(1,0), M31 = M(2,0);
  float c1 =pow(T,4)/24, c2 = pow(T,3)/6, c3 = pow(T,2)/2;
  float K1 = optParams.k1t, K2 = optParams.k2t;
  int vstart = 3*(H+1);

  // populate A matrix
  for (int k = 1; k <= H; k++) {
    for (int i = 1; i <= 3; i++) {

      // for vel bounds k
      A(i+(k-1)*9-1,i+(k-1)*3-1) = -K1;
      A(i+(k-1)*9-1,i+k*3-1) = +K1;
      A(i+(k-1)*9-1,vstart+i+(k-1)*3-1) = 1-T*K1;
    
      // for accl bounds k 
      A(3+i+(k-1)*9-1,i+(k-1)*3-1) = -K2;
      A(3+i+(k-1)*9-1,i+k*3-1) = +K2; 
      A(3+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = (-T)*K2;
                                          
      // for vel dynamics, correct them! corrected?
      A(6+i+(k-1)*9-1,i+(k-1)*3-1) = -M11*c1-M21*c2-M31*c3;
      A(6+i+(k-1)*9-1,i+k*3-1) = +M11*c1+M21*c2+M31*c3;
      A(6+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = -T*(M11*c1+M21*c2+M31*c3) + 1;
      A(6+i+(k-1)*9-1,vstart+i+(k)*3-1) = -1;
    }
  }
  
  optParams.A = A;
  optParams.Hmat = Hmat;

  ArrayXXi sched_multiple(1,6);
  sched_multiple << 1, 1, 1, 1;
  optParams.sched = sched_multiple;

  // formulate the mission (uses optParams)
  myMission = formulateMission();
  //solveCentalized();
  
  cout << endl << endl;
  cout << "******************************************************************************" << endl;
  solveDistributed();

  return 0;
}
