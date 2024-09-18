import context
import time
import numpy as np
import random
import cbss_d_mcpfd, cbss_d_old_mcpfd, cbss_tpg_mcpfd
import common as cm
import os
import pickle

SAVE_FLAG = True        
DEBUG_CASE = False

### obtain current path
cur_path = os.getcwd()

def run_CBSS_MCPFD_Example(sim = False, flag1=True, flag2=True, flag3=True):
  """
  include assignment constraints.
  N: 5~20
  M: 10~50
  """
  case_dict = dict()
  ny = 4
  nx = 4
  grids = np.zeros((ny,nx))
  # obstacles
  grids[0:2, 0] = 1
  grids[3,0] = 1
  grids[1,3] = 1
  grids[3,3] = 1

  starts = [8,1,2]
  targets = [6,9,10]
  dests = [11,5,14]
  ac_dict = dict()
  ag_dict = dict()
  ac_dict = {6: {2: 5}, 9: {0: 5}, 10: {0: 4, 2: 5},
             11: {0: 1}, 5: {1: 1}, 14: {2: 1}}
  ag_dict = {0: {9: 5, 10: 4, 11: 1},
             1: {5: 1},
             2: {6: 5, 10: 5, 14: 1}}
  print("Assignment constraints : ", ac_dict)
  case_dict["starts"] = starts
  case_dict["goals"] = targets
  case_dict["finals"] = dests
  case_dict["ac_dict"] = ac_dict
  case_dict["ag_dict"] = ag_dict

  configs = dict()
  configs["problem_str"] = "mcpfd"
  # configs["mtsp_fea_check"] = 1
  # configs["mtsp_atLeastOnce"] = 1
    # this determines whether the k-best TSP step will visit each node for at least once or exact once.
  configs["tsp_exe"] = "./pytspbridge/tsp_solver/LKH-2.0.10/LKH"
  configs["time_limit"] = 60
  configs["eps"] = 0.00

  print("------run_CBXS_MCPFD_Benchmark------")
  cbxs_baseline_dict = {}
  cbxs_old_res_dict = {}
  cbxs_res_dict = {}
  if flag1: 
    cbxs_baseline_dict = cbss_tpg_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                      case_dict["ac_dict"], case_dict["ag_dict"], configs)
    print(cbxs_baseline_dict)
  if flag2: 
    cbxs_old_res_dict = cbss_d_old_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                      case_dict["ac_dict"], case_dict["ag_dict"], configs)
    print(cbxs_old_res_dict)
  if flag3: 
    cbxs_res_dict = cbss_d_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                      case_dict["ac_dict"], case_dict["ag_dict"], configs)
    print(cbxs_res_dict)

  ### simulate the path-sets of CBXS
  if sim:
    # print("### finals:", case_dict["finals"])
    if flag1: cm.SimulatePathSet(grids, cbxs_baseline_dict['path_set'], case_dict["goals"], finals=case_dict['finals'], 
                                 ac_dict=case_dict["ac_dict"], target_timeline=cbxs_baseline_dict["target_timeline"])
    if flag2: cm.SimulatePathSet(grids, cbxs_old_res_dict['path_set'], case_dict["goals"], finals=case_dict['finals'], 
                                 ac_dict=case_dict["ac_dict"], target_timeline=cbxs_old_res_dict["target_timeline"])
    if flag3: cm.SimulatePathSet(grids, cbxs_res_dict['path_set'], case_dict["goals"], finals=case_dict['finals'], 
                                 ac_dict=case_dict["ac_dict"], target_timeline=cbxs_res_dict["target_timeline"])
  
  ### save result data
  if SAVE_FLAG:
    result = dict()
    if flag1:
      result['cbxs_baseline_dict'] = cbxs_baseline_dict
    if flag2:
      result['cbxs_old_res_dict'] = cbxs_old_res_dict
      result['n_conf_old'] = cbxs_old_res_dict["n_conf"]
      result['n_sp_conf_old'] = cbxs_old_res_dict["n_sp_conf"]
    if flag3:
      result['cbxs_res_dict'] = cbxs_res_dict
      result['n_conf'] = cbxs_res_dict["n_conf"]
      result['n_sp_conf'] = cbxs_res_dict["n_sp_conf"]

  return cbxs_baseline_dict, cbxs_old_res_dict, cbxs_res_dict

if __name__ == '__main__':
  print("begin of main")
  # EXPERIMENT:
  run_CBSS_MCPFD_Example(sim=True, flag1=True, flag2=False, flag3=True)
  print("end of main")