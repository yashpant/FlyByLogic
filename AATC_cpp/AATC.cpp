#include "AATC.h"

using namespace casadi;

bool AATC::loadMission(string filename){
  cout << "Loading Mission from " << filename << endl;
  // File with mission input
  ifstream inFile;
  string line;
  string::size_type sz;   // alias of size_t

  int i = 0;
  int count = 0;
  vector<vector<double>> p0;
  vector<vector<double>> v0;
  vector<vector<double>> v_bounds;
  vector<vector<double>> obs;
  vector<vector<double>> goal;
  vector<vector<double>> droneGoal;
  vector<double> map_boundary;

  // open mission file 
  inFile.open(filename);

  // check if valid mission file
  getline(inFile, line);

  if ((line.rfind("Mission Details")==string::npos)){
    cout << "Not a valid Mission File! Exiting!!!" << endl;
    return false;
  }

  // cout << string::npos << endl;
  while (!inFile.eof()){
    // count++;
    // cout << count << endl;

    if (count > 55){
      break;
    }

    getline(inFile, line);

    // cout << line << endl;

    if ((line.rfind("***")!=string::npos)){
      continue;
    }

    if ((line.rfind("#")!=string::npos)){
      continue;
    }

    if ((line.rfind("---")!=string::npos)){
      continue;
    }

    if ((line.rfind("Goals")!=string::npos)){
      continue;
    }

    if ((line.rfind("Obstacles")!=string::npos)){
      continue;
    }

    if ((line.rfind("Goal Intervals")!=string::npos)){
      continue;
    }

    if ((line.rfind("Number of Drones")!=string::npos)){
      i = line.find(":");
      optParams.nDrones = stoi(line.substr(i+1));
      continue;
    }

    if ((line.rfind("Mission Horizon")!=string::npos)){
      i = line.find(":");
      optParams.Horizon = stof(line.substr(i+1));
      continue;
    }

    if ((line.rfind("Waypoint Interval")!=string::npos)){
      i = line.find(":");
      optParams.T = stof(line.substr(i+1));
      continue;
    }

    if ((line.rfind("Map boundary")!=string::npos)){
      line = line.substr(1+line.find("["));
      cout << "i:"<< i << endl;
      std::string::size_type sz;
      for (int j = 0; j < 6; j++){
        map_boundary.push_back(stof(line, &sz));
        line = line.substr(sz+1);  
      }
      continue;
    }

    if ((line.rfind("Time step")!=string::npos)){
      i = line.find(":");
      optParams.h = stof(line.substr(i+1));
      continue;
    }

    if ((line.rfind("Minimum Separation")!=string::npos)){
      i = line.find(":");
      optParams.minSep = stof(line.substr(i+1));
      continue;
    }

    if ((line.rfind("Drone")!=string::npos)){
      vector<double> p0_, v0_, v_bounds_;
      line = line.substr(1+line.find("["));
      std::string::size_type sz;

      for (int j = 0; j < 5; j++){
        if (j < 3){
          v0_.push_back(0);
          p0_.push_back(stof(line, &sz));
        }
        else{
          v_bounds_.push_back(stof(line, &sz));
        }
        line = line.substr(sz+1);
      }
      v0.push_back(v0_);
      p0.push_back(p0_);
      v_bounds.push_back(v_bounds_);
      continue;
    }

    if ((line.rfind("Spec")!=string::npos)){
      vector<double> droneGoal_;
      line = line.substr(1+line.find("["));
      std::string::size_type sz;

      for (int j = 0; j < 4; j++){
        droneGoal_.push_back(stof(line, &sz));
        line = line.substr(sz+1);
      }

      droneGoal.push_back(droneGoal_);
      continue;
    }

    if ((line.rfind("Goal")!=string::npos)){
      vector<double> goal_;
      line = line.substr(1+line.find("["));
      std::string::size_type sz;

      for (int j = 0; j < 6; j++){
        goal_.push_back(stof(line, &sz));
        line = line.substr(sz+1);
      }

      goal.push_back(goal_);
      continue;
    }

    if ((line.rfind("Obstacle")!=string::npos)){
      vector<double> obs_;
      line = line.substr(1+line.find("["));
      std::string::size_type sz;

      for (int j = 0; j < 6; j++){
        obs_.push_back(stof(line, &sz));
        line = line.substr(sz+1);
      }

      obs.push_back(obs_);
      continue;
    }
  }

  optParams.npt = optParams.T/optParams.h;
  optParams.goal = goal;
  optParams.obs = obs;
  optParams.map_boundary = map_boundary;
  optParams.p0 = p0;
  optParams.v0 = v0;
  optParams.v_bounds =v_bounds;
  optParams.droneGoal = droneGoal;

  // cout << "nDrones : " << optParams.nDrones << endl;
  // cout << "initial pos : " << p0 << endl;
  // cout << "obs : " << obs << endl;
  // cout << "goals : " << goal << endl;
  // cout << "map_boundary : " << optParams.map_boundary << endl;

  inFile.close();

  return true;
}

void AATC::set_nDrones(int n){
  optParams.nDrones = n;

  int N = (int)ceil(optParams.nDrones/2.0);
  //cout << " N : " << N << endl;
  ArrayXXi sched_multiple(N,2*runs);
  sched_multiple = 0;
  for (int i = 0; i < optParams.nDrones; i++){
    if (i < N){
      sched_multiple.block(i,0,1,runs) = i+1;
    }
    else {
      sched_multiple.block(i-N,runs,1,runs) = i+1;
    }
  }
  optParams.sched = sched_multiple;
  // cout << optParams.sched << endl;
  // printMission();
}

void AATC::set_max_iter(int n){
  max_iter = n;
}

int AATC::get_max_iter(){
  return max_iter;
}

float AATC::get_alpha(){
  return alpha;
}

float AATC::get_gamma(){
  return gam;
}

int AATC::get_runs(){
  return runs;
}

int AATC::get_num_iter(){
  return num_iter;
}

float AATC::get_min_rob(){
  return min_rob;
}

double AATC::get_robustness(){
  return double(dmOut.rob);
}

double AATC::get_time_centralized(){
  return time_centralized;
}

double AATC::get_time_distributed(){
  return time_distributed;
}

void AATC::printMission(ostream& out){
  out << endl << "Printing Mission Details" << endl;
  out << "*******************************" << endl << endl;
  out << "Number of Drones     :  " << optParams.nDrones << endl;
  out << "Number of Obstacles  :  " << optParams.obs.size() << endl;
  out << "Number of Goals      :  " << optParams.goal.size() << endl;
  out << "Waypoint Interval    :  " << optParams.T << " (s) " << endl;
  out << "Time step            :  " << optParams.h << " (s) " << endl;
  out << "Mission Horizon      :  " << optParams.Horizon << " (s) " << endl;
  out << "Minimum Separation   :  " << optParams.minSep << " (m) " << endl;
  
  out << endl << "Initial Positions" << endl;
  out << "-----------------------" << endl;
  out << "Drone#  : [x0, y0, z0, v_max, a_max]" << endl;
  for (int i = 0; i < optParams.p0.size() + 2; i++){
    out << "Drone" << i << "  : " << optParams.p0[i] << endl;
  }
  out << endl;

  out << "Obstacles" << endl;
  out << "-----------------------" << endl;
  out << "Obstacle#  : [lbx, lby, lbz, ubx, uby, ubz]" << endl;
  for (int i = 0; i < optParams.obs.size() ; i++){
    out << "Obstacle" << i << "  : " << optParams.obs[i] << endl;
  }
  out << endl;

  out << "Goals" << endl;
  out << "-----------------------" << endl;
  out << "Goal#  : [lbx, lby, lbz, ubx, uby, ubz]" << endl;
  for (int i = 0; i < optParams.goal.size() ; i++){
    out << "Goal" << i << "  : " << optParams.goal[i] << endl;
  }
  out << endl;

  out << "Goal Intervals" << endl;
  out << "-----------------------" << endl;
  out << "Spec#  : [Drone#, Goal#, iStart, iEnd]" << endl;
  for (int i = 0; i < optParams.droneGoal.size() ; i++){
    out << "Spec" << i+1 << "  : " << optParams.droneGoal[i] << endl;
  }
  out << endl;
}

void AATC::printMissionOutput(ostream& out){
  
  // print details to output file
  //***********************************************************
  out << "Mission Results" << endl;
  out << "*********************************" << endl;

  out << endl;
  out << "smooth robustness :               " << dmOut.rob << endl;
  out << "separation_robustness :           " << dmOut.sepRob << endl;
  out << "goal_robustness :                 " << dmOut.goalRob << endl;
  out << "unsafe_robustness :               " << dmOut.unsafeRob << endl;
  out << "total trajectory length :         " << dmOut.traj_length << endl;


  time_centralized = ((float)t)/CLOCKS_PER_SEC;
  out << endl;
  out << "----------------------------------------------------------------------------" << endl;
  out << "w_opt = ";
  out << fixed;
  for (int k = 0; k < res["x"].size1()-1; k++){
    out << setprecision(1) << res["x"](k) << " ";
  }
  out << setprecision(1) << res["x"](res["x"].size1()-1) << endl;
  out << "----------------------------------------------------------------------------" << endl;
  out << "Time Taken : "<< time_centralized <<" second(s)"<< endl;
  //***********************************************************
}

AATC::AATC(){
  cout << "Initializing AATC with default mission" << endl;
  // default settings
  optParams.nDrones = 1;

  float T = 1.0;
  optParams.T = T;

  float h = 1.0/20.0;
  optParams.h = h;
  optParams.npt = T/h;

  float H = 6.0;
  optParams.Horizon = H;

  optParams.minSep = 0.1;

  // define initial position and velocity of Drones -- INPUT
  // vector<vector<double>> p0 = {{-1.25,-1.25,0.0},{1.25,-1.25,0.0},{-1.25,1.25,0.0},{1.25,1.25,0.0},{-1.45,-1.45,0.0},{1.45,-1.45,0.0},{-1.45,1.45,0.0},{1.45,1.45,0.0}};
  vector<vector<double>> p0 = {{-1.25,-1.25,1.75},{1.25,-1.25,1.75},{-1.25,1.25,1.75},{1.25,1.25,1.75},{-1.45,-1.45,1.75},{1.45,-1.45,1.75},{-1.45,1.45,1.75},{1.45,1.45,1.75}};
  vector<vector<double>> v0 = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  optParams.p0 = p0;
  optParams.v0 = v0;

  // define obstacle -- INPUT
  vector<vector<double>> obs = {{-1.0, -1.0, 0.0, 1.0, 1.0, 1.50}};
  optParams.obs = obs;

  // define goals -- INPUT
  vector<vector<double>>goal = {{1.25, 1.25, 0.25,  2.25, 2.25, 1.25}};
  optParams.goal = goal;

  // define drone-goal intervals -- INPUT
  vector<vector<double>> droneGoal = {{0, 0, 0, H},{1, 0, 0, H},{2, 0, 0, H},{3, 0, 0, H},{4, 0, 0, H},{5, 0, 0, H},{6, 0, 0, H},{7, 0, 0, H}};
  optParams.droneGoal = droneGoal;
  
  // dynamics matrix
  vector<vector<double>> M_ = {{        90, 0, -15*pow(T,2)},
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

  // select right tp
  if (tp <= 0 || tp > T){
      tp = tp2;
  }

  // spline constant 2
  double k2t = (90.0*pow(tp,3))/(12*pow(T,5)) - (90.0*pow(tp,2))/(4*pow(T,4)) + (30.0*tp)/(2*pow(T,3));
  
  optParams.k1t = k1t;
  optParams.k2t = k2t;

  mxOut.rob = -inf;
  lambda = 0.00;

  // boolean mode params
  boolean_mode = false;

  // Distribute params
  DM A = DM::zeros(9*H, 6*(H+1));
  DM Hmat = DM::eye(6*(H+1));

  DM M11 = M(0,0), M21 = M(1,0), M31 = M(2,0);
  float c1 =pow(T,4)/24, c2 = pow(T,3)/6, c3 = pow(T,2)/2;
  int vstart = 3*(H+1);

  // populate A matrix
  for (int k = 1; k <= H; k++) {
      for (int i = 1; i <= 3; i++) {

      // for vel bounds k
      A(i+(k-1)*9-1,i+(k-1)*3-1) = -k1t;
      A(i+(k-1)*9-1,i+k*3-1) = +k1t;
      A(i+(k-1)*9-1,vstart+i+(k-1)*3-1) = 1-T*k1t;
      
      // for accl bounds k 
      A(3+i+(k-1)*9-1,i+(k-1)*3-1) = -k2t;
      A(3+i+(k-1)*9-1,i+k*3-1) = +k2t; 
      A(3+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = (-T)*k2t;
                                          
      // for vel dynamics, correct them! corrected?
      A(6+i+(k-1)*9-1,i+(k-1)*3-1) = -M11*c1-M21*c2-M31*c3;
      A(6+i+(k-1)*9-1,i+k*3-1) = +M11*c1+M21*c2+M31*c3;
      A(6+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = -T*(M11*c1+M21*c2+M31*c3) + 1;
      A(6+i+(k-1)*9-1,vstart+i+(k)*3-1) = -1;
      }
  }

  optParams.A = A;
  optParams.Hmat = Hmat;

  int N = (int)ceil(optParams.nDrones/2.0);
  //cout << " N : " << N << endl;
  ArrayXXi sched_multiple(N,2*runs);
  sched_multiple = 0;
  for (int i = 0; i < optParams.nDrones; i++){
    if (i < N){
      sched_multiple.block(i,0,1,runs) = i+1;
    }
    else {
      sched_multiple.block(i-N,runs,1,runs) = i+1;
    }
  }
  optParams.sched = sched_multiple;

  // cout << optParams.sched << endl;

  //printMission();
}

AATC::AATC(string filename){
  
  if (loadMission(filename)){

  float T = optParams.T;
  float H = optParams.Horizon;
  
  // dynamics matrix
  vector<vector<double>> M_ = {{        90, 0, -15*pow(T,2)},
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

  // select right tp
  if (tp <= 0 || tp > T){
      tp = tp2;
  }

  // spline constant 2
  double k2t = (90.0*pow(tp,3))/(12*pow(T,5)) - (90.0*pow(tp,2))/(4*pow(T,4)) + (30.0*tp)/(2*pow(T,3));
  
  optParams.k1t = k1t;
  optParams.k2t = k2t;

  mxOut.rob = -inf;
  lambda = 0.00;

  // boolean mode params
  boolean_mode = false;

  // Distribute params
  DM A = DM::zeros(9*H, 6*(H+1));
  DM Hmat = DM::eye(6*(H+1));

  DM M11 = M(0,0), M21 = M(1,0), M31 = M(2,0);
  float c1 =pow(T,4)/24, c2 = pow(T,3)/6, c3 = pow(T,2)/2;
  int vstart = 3*(H+1);

  // populate A matrix
  for (int k = 1; k <= H; k++) {
      for (int i = 1; i <= 3; i++) {

      // for vel bounds k
      A(i+(k-1)*9-1,i+(k-1)*3-1) = -k1t;
      A(i+(k-1)*9-1,i+k*3-1) = +k1t;
      A(i+(k-1)*9-1,vstart+i+(k-1)*3-1) = 1-T*k1t;
      
      // for accl bounds k 
      A(3+i+(k-1)*9-1,i+(k-1)*3-1) = -k2t;
      A(3+i+(k-1)*9-1,i+k*3-1) = +k2t; 
      A(3+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = (-T)*k2t;
                                          
      // for vel dynamics, correct them! corrected?
      A(6+i+(k-1)*9-1,i+(k-1)*3-1) = -M11*c1-M21*c2-M31*c3;
      A(6+i+(k-1)*9-1,i+k*3-1) = +M11*c1+M21*c2+M31*c3;
      A(6+i+(k-1)*9-1,vstart+i+(k-1)*3-1) = -T*(M11*c1+M21*c2+M31*c3) + 1;
      A(6+i+(k-1)*9-1,vstart+i+(k)*3-1) = -1;
      }
  }

  optParams.A = A;
  optParams.Hmat = Hmat;

  int N = (int)ceil(optParams.nDrones/2.0);
  //cout << " N : " << N << endl;
  ArrayXXi sched_multiple(N,2*runs);
  sched_multiple = 0;
  for (int i = 0; i < optParams.nDrones; i++){
    if (i < N){
      sched_multiple.block(i,0,1,runs) = i+1;
    }
    else {
      sched_multiple.block(i-N,runs,1,runs) = i+1;
    }
  }
  optParams.sched = sched_multiple;
  }
  else{
    exit(1);
  }

  // printMission(cout);
}

AATC::AATC(int n_drones, float horizon, float dt, float DT, float min_sep, vector<vector<double>> goal, vector<vector<double>> goal_intervals, vector<vector<double>> obs){

    cout << "Initializing AATC with default mission" << endl;
    optParams.nDrones = n_drones;
    optParams.Horizon = horizon;
    optParams.h = dt;
    optParams.T = DT;
    optParams.minSep = min_sep;
}


ArrayXXi AATC::getPairCombos(int N){
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

int AATC::getIndex(float t){
  //cout << "t : " << t << endl;
  //cout << "npt : " << optParams.npt << endl;
  if (t >= optParams.Horizon){
    return (int) floor(optParams.Horizon * optParams.npt - 1);
  }
  // cout << "hey" << endl;
  return (int) (t*optParams.npt);
}

template <typename T>
T AATC::smoothMinVec(T x, double c){
  // return the vector of smooth min of a vector
  int m = x.size2();
  return -log(mtimes(exp(-c*x),T::ones(m,1)))/c;
}

template <typename T>
T AATC::smoothMin(T x, double c){
  //cout << "C : " << c << endl;
  // return the smooth min of a vector
  int n = x.size1();
  int m = x.size2();

  // return log(exp(x).transpose()*MX::ones(size))
  return -log(dot(mtimes(exp(-c*x),T::ones(m,1)), T::ones(n,1)))/c;
}

template <typename T>
T AATC::smoothMax(T x, double c){
  // return the smooth max of a vector
  int n = x.size1();
  int m = x.size2();

  return log(dot(mtimes(exp(c*x),T::ones(m,1)), T::ones(n,1)))/c;
}

template <typename T>
T AATC::inSet(T xx, T yy, T zz, vector<double> set){
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // set is of size 1 x 6
  // compute distance of drone from set at each point
  vector<T> temp;
  temp.push_back(xx-set[0]);
  temp.push_back(yy-set[1]);
  temp.push_back(zz-set[2]);
  temp.push_back(set[3]-xx);
  temp.push_back(set[4]-yy);
  temp.push_back(set[5]-zz);
  //perform min over coordinates
  return smoothMinVec(horzcat(temp));
}

template <typename T>
T AATC::always_in(T xx, T yy, T zz, vector<double> set){
  // return the robustness of a path always in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // set is of size 1 x 6
  T temp = inSet(xx, yy, zz, set);
  return smoothMin(temp);
}

template <typename T>
T AATC::always_not_in(T xx, T yy, T zz, vector<double> set){
  // return the robustness of a path always not in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // set is of size 1 x 6
  T temp = inSet(xx, yy, zz, set);
  return smoothMin(-temp);
}

template <typename T>
T AATC::eventually_in(T xx, T yy, T zz, vector<double> set){
  // return the robustness of a path eventually in a set
  // xx is of size T x 1
  // yy is of size T x 1
  // zz is of size T x 1
  // set is of size 1 x 6
  T temp = inSet(xx, yy, zz, set);
  return smoothMax(temp);
}

template <typename T>
T AATC::always_eventually(T xx, T yy, T zz, vector<double> set, float a, float b, float c, float d){
  // STL formula: ALWAYS (a, b) EVENTUALLY (c, b) BE_in_SET
  // Rhudi's version
  vector<T> temp;
  T x, y, z;
  // start and end indices
  int iS, iE;
  float t, dt = optParams.h;
  for (t = a; t <= b+(dt/10); t += dt){
    iS = getIndex(t+c);
    iE = getIndex(t+d);
    // extract trajectory path
    x = xx(Slice(iS, iE+1));
    y = yy(Slice(iS, iE+1));
    z = zz(Slice(iS, iE+1));
    temp.push_back(eventually_in(x, y, z, set));
  }
  return smoothMin(vertcat(temp));
}

template <typename T>
T AATC::always_eventually_alena(T xx, T yy, T zz, vector<double> set, float a, float b, float c, float d){
  // STL formula: ALWAYS (a, b) EVENTUALLY (c, b) BE_in_SET
  // Alena's version
  vector<T> temp;
  T x, y, z;
  float dt = optParams.h;
  T ap, apInJ;
  int lenI = getIndex(b)-getIndex(a)+1;
  int lenJ = getIndex(d)-getIndex(c)+1;
  int indL = getIndex(a+c);
  int indU = getIndex(b+d);

  x = xx(Slice(indL, indU+1));
  y = yy(Slice(indL, indU+1));
  z = zz(Slice(indL, indU+1));
  ap = inSet(x, y, z, set);
  //cout<<"ap size "<<ap.size1()<<endl;

  for (int i=0; i<lenI; i++){  
    apInJ = ap(Slice(i, i+lenJ));
    // perform EVENTUALLY
    temp.push_back(smoothMax(apInJ));
  }
  return smoothMin(vertcat(temp));
}

template <typename T>
T AATC::eventually_always(T xx, T yy, T zz, vector<double> set, float a, float b, float c, float d){
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
T AATC::unsafeRob(T xx, T yy, T zz, vector<vector<double>> obs){
  // return the robustness of all paths with respect to all obstacles
  // xx is of size T x N
  // yy is of size T x N
  // zz is of size T x N
  // obs is of size M x 6
  
  // Algorithm
  
  // get number of obstacles
  int M = obs.size();
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

      out.push_back(always_not_in(x, y, z, obs[j]));
    }
  }

  return smoothMin(vertcat(out));
}

template <typename T>
T AATC::goalRob(T xx, T yy, T zz, vector<vector<double>> goal, vector<vector<double>> droneGoal){
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
    while(i == droneGoal[j][0]){
      
      // cout << droneGoal.row(i) << endl;
      //extract start and end intervals for current goal
      k = int(droneGoal[j][1]);
      iS = int(droneGoal[j][2]*npt/Time);
      iE = int(droneGoal[j][3]*npt/Time);
      
      // extract relevant interval of ith drone path
      x = xx(Slice(iS, iE),i);
      y = yy(Slice(iS, iE),i);
      z = zz(Slice(iS, iE),i);

      //cout << goal(k) << endl;
      // OPTION 1: EVENTUALLY
      //out.push_back(eventually_in(x, y, z, goal[k]));
      
      // OPTION 2: ALWAYS EVENTUALLY (constantly cycle loops around the goal)
      float a = 1.0;
      float b = 8.0;
      float c = 0.0;
      float d = 1.95;
      out.push_back(always_eventually(x, y, z, goal[k], a, b, c, d));
      //out.push_back(always_eventually_alena(x, y, z, goal[k], a, b, c, d));
  
      // cout << "j :" << j << endl;
      if(j < droneGoal.size()-1){
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
T AATC::sepRob(T xx, T yy, T zz, double minSep){
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
void AATC::missionRob(in_type var){
  cout << "In Mission Rob" << endl;
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
  cout << "Got 1st Params" << endl;
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

  // cout << "Computing Robusteness....." << endl;
  rho_sep = sepRob(xx, yy, zz, optParams.minSep);
  // cout << "Got Sep Robustness" << endl;
  rho_goal = goalRob(xx, yy, zz, optParams.goal, optParams.droneGoal);
  // cout << "Got Goal Robustness" << endl;
  rho_unsafe = unsafeRob(xx, yy, zz, optParams.obs);
  // cout << "Got Unsafe Robustness" << endl;

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
  cout<<"Final rob = "<< rob<<endl;
  
  if (typeid(rob).name() == typeid(MX).name()){
    mxOut.rob = (MX) rob;
    mxOut.sepRob = (MX) rho_sep;
    mxOut.goalRob = (MX) rho_goal;
    mxOut.unsafeRob = (MX) rho_unsafe;
    mxOut.traj_length = (MX) traj_length;
  }

  if (typeid(rob).name() == typeid(DM).name()){
    dmOut.rob = (DM) rob;
    dmOut.sepRob = (DM) rho_sep;
    dmOut.goalRob = (DM) rho_goal;
    dmOut.unsafeRob = (DM) rho_unsafe;
    dmOut.traj_length = (DM) traj_length;
  }
  
}

void AATC::formulateMission(){
  cout << "Formulating Mission" << endl;
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
  double lmapX = optParams.map_boundary[0];
  double lmapY = optParams.map_boundary[1];
  double lmapZ = optParams.map_boundary[2];

  double umapX = optParams.map_boundary[3];
  double umapY = optParams.map_boundary[4];
  double umapZ = optParams.map_boundary[5];  
  // sainty check
  //cout<< "optParams.map_boundary low: ["<< lmapX<<", "<<lmapY<<", "<<lmapZ<<"]"<<endl;
  //cout<< "optParams.map_boundary up : ["<< umapX<<", "<<umapY<<", "<<umapZ<<"]"<<endl;

  MX pCur, pPrev, vCur, vPrev, dp, foo, mCons, vf;
  double dv = 0, da = 0, maxVel = 20, maxAcc = 20; 

  double k1t = optParams.k1t;
  double k2t = optParams.k2t;

  maxAcc = 2;
  maxVel = 1.751;
  vector<vector<double>> lbg_, ubg_, lbw_, ubw_, lbv_, ubv_;
  vector<double> goall = {1.75, 1.75, 0.75};
  vector<double> goalu = {2, 2, 1};
  vector<double> zil_ = {0, 0, 0};
  
  for (int k = 0; k < optParams.nDrones; k++){
    if (optParams.v_bounds.size()){
      maxVel = optParams.v_bounds[k][0];
      maxAcc = optParams.v_bounds[k][1];
    }
    lbg_.push_back({-maxVel,-maxVel,-maxVel,-maxAcc,-maxAcc,-maxAcc,0,0,0});
    ubg_.push_back({+maxVel,+maxVel,+maxVel,+maxAcc,+maxAcc,+maxAcc,0,0,0});
    lbw_.push_back({lmapX, lmapY, lmapZ});
    ubw_.push_back({umapX, umapY, umapZ});
    lbv_.push_back({-maxVel, -maxVel, -maxVel});
    ubv_.push_back({+maxVel, +maxVel, +maxVel});
  }
  
  // construct T vector
  vector<double> Tvec_ = {pow(T,4)/24, pow(T,3)/6, pow(T,2)/2};
  MX Tvec = MX(Tvec_);
  
  stringstream pname, vname;

  for(int d = 0; d < nDrones; d++){
    // cout << d << endl;
    // set position bounds for initial state
    lbw.insert(lbw.end(), p0[d].begin(), p0[d].end());
    ubw.insert(ubw.end(), p0[d].begin(), p0[d].end());

    // set velocity bounds for initial state
    lbv.insert(lbv.end(), v0[d].begin(), v0[d].end());
    ubv.insert(ubv.end(), v0[d].begin(), v0[d].end());

    // construct variable names for initial state of drone d
    pname << "p" << d << "0";
    vname << "v" << d << "0";
    
    // initialize casadi smybolic variable for initial state of drone d
    pPrev = MX::sym(pname.str(),3,1);
    vPrev = MX::sym(pname.str(),3,1);

    // push symbolic variable to state variable array
    w.push_back(pPrev);
    v.push_back(vPrev);

    // for all big time steps from 0 to H (steps of T - waypoint interval)
    for(int k = 0; k < H; k++){

      // clear out variable names (to empty strings)
      pname.str(""); vname.str("");

      // construct variable name for k+1(th) state of drone d
      pname << "p" << d << "" << k+1;
      vname << "v" << d << "" << k+1;

      // initialize casadi symbolic variable for k+1(th) state of drone d
      pCur = MX::sym(pname.str(),3,1);
      vCur = MX::sym(vname.str(),3,1);

      // push varible into state variable array
      w.push_back(pCur);
      
      // set position bounds for k+1(th) state of drone d
      lbw.insert(lbw.end(), lbw_[d].begin(), lbw_[d].end());
      ubw.insert(ubw.end(), ubw_[d].begin(), ubw_[d].end());
      
      // push variable into state variable array
      v.push_back(vCur);

      // set velocity bounds for k+1(th) state of drone d
      // if last waypoint, ensure stopping criteria 
      if(k < H-1){
        lbv.insert(lbv.end(), lbv_[d].begin(), lbv_[d].end());
        ubv.insert(ubv.end(), ubv_[d].begin(), ubv_[d].end());
      }
      else{
        lbv.insert(lbv.end(), zil_.begin(), zil_.end());
        ubv.insert(ubv.end(), zil_.begin(), zil_.end());
      }

      // Handle dynamic constaints
      // dp for all axes
      dp = pCur - pPrev - T*vPrev;
      foo = horzcat(dp,dv*MX::ones(3,1),da*MX::ones(3,1)).T();
      mCons = mtimes(M, foo);

      vf = mtimes(mCons.T(),Tvec) + vPrev;

      g.push_back(k1t*dp + vPrev);  // vel constraints on dp
      g.push_back(k2t*dp);      // acc constraints
      g.push_back(vCur - vf);

      lbg.insert(lbg.end(), lbg_[d].begin(), lbg_[d].end());
      ubg.insert(ubg.end(), ubg_[d].begin(), ubg_[d].end());

      pPrev = pCur;
      vPrev = vCur;
    }
  }
  
  // concatenate the velocity state to the position state
  w.insert(w.end(), v.begin(), v.end());
  lbw.insert(lbw.end(), lbv.begin(), lbv.end());
  ubw.insert(ubw.end(), ubv.begin(), ubv.end());

  // convert state array from vector<MX> to column MX vector (this is how casadi wants it to be) 
  MX var = vertcat(w);    // this is x

  // Generate initial waypoints using a simple QP problem
  //***************************************************************************

  // Extract waypoints from state array
  vector<MX> var_ = vertsplit(var, var.size1()/2);

  // Define QP
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

  cout << "Getting Initial Solution" << endl;
  // cout << "Init solve" << endl;
  Function init_solver =  qpsol("init", "qpoases", qp, qpoasesOpts);
  DMDict init_res = init_solver(qp_args);

  cout << "Got Initial Solution" << endl;
  
  //cout << endl;
  //cout << "-----------------------------------------------------" << endl;
  //cout << "x0 = " << init_res.at("x") << ";" << endl;
  //cout << "-----------------------------------------------------" << endl;
  //*******************************************************************************

  // Setup [elements] of (cost function) --> (a*[robustness] + b*[trajectory length])
  
  missionRob(var);
  MX f;

  // choose between boolean mode and normal mode
  if (boolean_mode){
    f = 0*mxOut.traj_length;
    g.push_back(mxOut.rob);
    lbg.push_back(min_rob);
    ubg.push_back(max_rob);
  }
  else{
    f = lambda*mxOut.traj_length - mxOut.rob;
  }

  cout << "Constructed Cost Function" << endl;

  // populate mission information (for later use in optimazation problem)
  myMission.f = f;                      // cost function
  myMission.x = var;                    // state variables
  myMission.g = vertcat(g);             // contriants
  myMission.x0 = init_res.at("x");      // initial guess
  myMission.lbx = lbw;                  // lower bounds on state
  myMission.ubx = ubw;                  // upper bounds on state
  myMission.lbg = lbg;                  // lower bounds on constraints
  myMission.ubg = ubg;                  // upper bounds on constraints
  
  // prepare some structure for distributed approach
  //***********************************************************
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
  //***********************************************************
}

void AATC::solveCentralized(){
  
  // Define optimization problem
  MXDict prob;

  prob["f"] = myMission.f;      // cost function
  prob["x"] = myMission.x;      // state variables
  prob["g"] = myMission.g;      // constraints

  DMDict args;
  args["x0"] = myMission.x0;    // initial guess
  args["lbx"] = myMission.lbx;  // lower bounds on state
  args["ubx"] = myMission.ubx;  // upper bounds on state
  args["lbg"] = myMission.lbg;  // lower bounds on constraints
  args["ubg"] = myMission.ubg;  // upper bounds on constraints

  // set options for nlp optimization solver
  //***********************************************************
  Dict Opts, ipoptOpts;
  //ipoptOpts["linear_solver"] = "ma27";    // using HSL routines
  ipoptOpts["print_level"] = 5;
  ipoptOpts["acceptable_tol"] = 1e-6;
  ipoptOpts["tol"] = 1e-6;
  ipoptOpts["max_iter"] = 5000;
  ipoptOpts["hessian_approximation"] = "limited-memory";
  Opts["ipopt"] = ipoptOpts;
  //***********************************************************

  // initialize solver (ipopt used)
  Function solver = nlpsol("solver", "ipopt", prob, Opts);  

  // solve optimization problem with set arguments, measure time taken to solve
  t = clock();
  res = solver(args);
  t = clock() - t;

  // get the trajectories and individual robustness
  missionRob(res.at("x"));
}

// Experimental
void AATC::solveDistributed(){

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
  qpoasesOpts["terminationTolerance"] = 1e-6;

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

  // cout << "Hey" << endl;
  // online implementation wrapper
  for (int k =0; k < nWayPoints; k++)
  {
    w = clock();
    // PSCA Algorithm loop
    for (int r = 0; r < max_iter; r++){

      // cout << "PSCA solve" << endl;

      num_iter = r+1;
      // serial implementation of BCD
      for (int i = 0; i < sched.rows(); i++){
        // select drones for update
        dn = sched(i,r % sched.cols()) - 1;

        // cout << "Hey" << endl;
        if (dn <= -1){
          continue;
        }

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

        if (dn <= -1){
          continue;
        }

        x0_pos[dn] = x0_pos[dn] + pow(gam,r)*(tempSol_vec[0] - x0_pos[dn]);
        x0_vel[dn] = x0_vel[dn] + pow(gam,r)*(tempSol_vec[1] - x0_vel[dn]);
      }
      
      // stitch the trajectory back
      x = vertcat(vertcat(x0_pos), vertcat(x0_vel));

      // clear the temp container
      tempSol.erase(tempSol.begin(), tempSol.end());

      // check the robustness value every 20 iterations
      if (r % 20 == 0){
        missionRob(x);
        // cout << "rob : " << dmOut.rob <<  "  iter : " << r << " Sol_time : " << ((float)q)/CLOCKS_PER_SEC << endl;
      }

      // exit after minimum robustnes acheived
      if (double(dmOut.rob) > 0.05){
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
    time_distributed = ((float)w)/CLOCKS_PER_SEC;
    // cout << "Time Spent : " << ((float)w)/CLOCKS_PER_SEC << endl;

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
  missionRob(x);
  // cout << "rob : " << dmOut.rob << endl;

  // float time_taken = ((float)t)/CLOCKS_PER_SEC;
  // cout << endl;
  // cout << "----------------------------------------------------------------------------" << endl;
  // cout << "cpp_out = " << x << ";" << endl;
  // cout << "----------------------------------------------------------------------------" << endl;
  // cout << "Solving took "<< time_distributed <<" second(s)."<< endl;
}
