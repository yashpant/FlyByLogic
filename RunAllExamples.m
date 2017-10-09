addpath(genpath('./'));
% change solver to ma27 from mumps in each file if the hsl routines are
% present
% Note: if the optimization returns errors for larger number of drones, change
% the constant C in the robustness functions to a smaller value
%% Reach-avoid (Boolean, stop go)
reach_avoid_stopgo_Ndrones_boolean;

%% Reach-avoid (Robust, stop go)
reach_avoid_stopgo_Ndrones;

%% Reach-avoid (Boolean, free end pt vel)
reach_avoid_Ndrones_boolean_varvel;

%% Reach-avoid (Robust, free end pt vel)
reach_avoid_Ndrones_varvel;

%% Multi-mission (Boolean)
CaseStudyMain_boolean_iccps;

%% Multi-mission (Robust)
CaseStudyMain_iccps;
