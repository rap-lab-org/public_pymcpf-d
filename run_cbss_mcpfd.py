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

def run_CBSS_MSMP(case_name, map_file, scen_file, N = 5, M = 10, idx = 1, duration=5, exp_name='', sim=False, flag1=True, flag2=True, flag3=True):
  """
  fully anonymous case, no assignment constraints.
  N: 5~20
  M: 10~50
  """
  case_dict = cm.LoadTestCaseDao(map_file, scen_file)

  ### create output directory
  output_dir = os.path.join(cur_path, 'output/', exp_name, case_name, 'N'+str(N)+'M'+str(M)+'D'+str(duration))
  cm.mkdir(output_dir)
  print(output_dir)
  case_path = os.path.join(output_dir, 'case'+str(idx)+'.pickle')
  result_path = os.path.join(output_dir, 'result'+str(idx)+'.pickle')
  if os.path.exists(result_path) and not DEBUG_CASE: return {}, {}, {}
  ### processing benchmark data
  grids = case_dict["grids"]
  starts_all = case_dict["starts"]
  targets_all = case_dict["goals"]
  dests_all = case_dict["finals"]
  sf_index = list(range(len(starts_all)))
  sf_index = random.sample(sf_index, N)
  starts = [starts_all[x] for x in sf_index]
  dests = [dests_all[y] for y in sf_index]
  targets = random.sample(targets_all, M)
  ac_dict = dict()
  ag_dict = dict()
  for k in targets:
    ac_dict[k] = {ri % len(starts): duration for ri in range(N)}
    for ri in range(N):
      if ri % len(starts) not in ag_dict: ag_dict[ri % len(starts)] = dict()
      ag_dict[ri % len(starts)].update({k: duration})
  for k in dests:
    ac_dict[k] = {ri: 1 for ri in range(N)}
    for ri in range(N):
      if ri not in ag_dict: ag_dict[ri] = dict()
      ag_dict[ri].update({k: 1})

  case_dict["starts"] = starts
  case_dict["goals"] = targets
  case_dict["finals"] = dests
  case_dict["ac_dict"] = ac_dict
  case_dict["ag_dict"] = ag_dict

  ### debug case load
  if DEBUG_CASE:
    case_pickle = open(case_path,'rb')
    case_dict = pickle.load(case_pickle)

  ### save case data
  if SAVE_FLAG:
    with open(case_path, 'wb') as f:
      pickle.dump(case_dict, f)

  configs = dict()
  configs["problem_str"] = "msmp"
  # configs["mtsp_fea_check"] = 1
  # configs["mtsp_atLeastOnce"] = 1
    # this determines whether the k-best TSP step will visit each node for at least once or exact once.
  configs["tsp_exe"] = "./pytspbridge/tsp_solver/LKH-2.0.10/LKH"
  configs["time_limit"] = 60
  configs["eps"] = 0.00

  print("------run_CBSS_MCPF_Benchmark------")
  cbxs_baseline_dict = {}
  cbxs_old_res_dict = {}
  cbxs_res_dict = {}
  if flag1: cbxs_baseline_dict = cbss_tpg_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                             case_dict["ac_dict"], configs)
  if flag2: cbxs_old_res_dict = cbss_d_old_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                              case_dict["ac_dict"], configs)
  if flag3: cbxs_res_dict = cbss_d_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], 
                                                      case_dict["ac_dict"], configs)
  print(cbxs_baseline_dict)
  print(cbxs_old_res_dict)
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
      
    with open(result_path, 'wb') as f:
      pickle.dump(result, f)

  return cbxs_baseline_dict, cbxs_old_res_dict, cbxs_res_dict

def run_CBSS_MCPFD(case_name, map_file, scen_file, N = 5, M = 10, idx = 1, duration=5, exp_name='', sim = False, flag1=True, flag2=True, flag3=True):
  """
  fully anonymous case, no assignment constraints.
  N: 5~20
  M: 10~50
  """
  case_dict = cm.LoadTestCaseDao(map_file, scen_file)

  ### create output directory
  output_dir = os.path.join(cur_path, 'output/', exp_name, case_name, 'N'+str(N)+'M'+str(M)+'D'+str(duration))
  cm.mkdir(output_dir)
  print(output_dir)
  case_path = os.path.join(output_dir, 'case'+str(idx)+'.pickle')
  result_path = os.path.join(output_dir, 'result'+str(idx)+'.pickle')
  if os.path.exists(result_path) and not DEBUG_CASE: return {}, {}, {}
  ### processing benchmark data
  grids = case_dict["grids"]
  starts_all = case_dict["starts"]
  targets_all = case_dict["goals"]
  dests_all = case_dict["finals"]
  sf_index = list(range(len(starts_all)))
  sf_index = random.sample(sf_index, N)
  print(sf_index)
  # return
  starts = [starts_all[x] for x in sf_index]
  dests = [dests_all[y] for y in sf_index]
  targets = random.sample(targets_all, M)
  ac_dict = dict()
  ag_dict = dict()
  ri = 0
  for k in targets:
    ac_dict[k] = {ri % len(starts): duration,(ri+1)%len(starts): duration}
    if ri % len(starts) not in ag_dict: ag_dict[ri % len(starts)] = dict()
    if (ri+1)%len(starts) not in ag_dict: ag_dict[(ri+1)%len(starts)] = dict()
    ag_dict[ri % len(starts)].update({k: duration})
    ag_dict[(ri+1)%len(starts)].update({k: duration})
    ri += 1
  ri = 0
  for k in dests:
    ac_dict[k] = {ri: 1}
    if ri not in ag_dict: ag_dict[ri] = dict()
    ag_dict[ri].update({k: 1})
    ri += 1

  case_dict["starts"] = starts
  case_dict["goals"] = targets
  case_dict["finals"] = dests
  case_dict["ac_dict"] = ac_dict
  case_dict["ag_dict"] = ag_dict

  ### debug case load
  if DEBUG_CASE:
    case_pickle = open(case_path,'rb')
    case_dict = pickle.load(case_pickle)

  ### save case data
  if SAVE_FLAG:
    with open(case_path, 'wb') as f:
      pickle.dump(case_dict, f)

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
  if flag1: cbxs_baseline_dict = cbss_tpg_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], case_dict["ac_dict"], case_dict["ag_dict"], configs)
  if flag2: cbxs_old_res_dict = cbss_d_old_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], case_dict["ac_dict"], case_dict["ag_dict"], configs)
  if flag3: cbxs_res_dict = cbss_d_mcpfd.RunCbssMCPFD(grids, case_dict["starts"], case_dict["goals"], case_dict["finals"], case_dict["ac_dict"], case_dict["ag_dict"], configs)
  print(cbxs_baseline_dict)
  print(cbxs_old_res_dict)
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
    with open(result_path, 'wb') as f:
      pickle.dump(result, f)

  return cbxs_baseline_dict, cbxs_old_res_dict, cbxs_res_dict

def RunExp(exp_name, case, map, problem, N_list, M_list, duration_list, sim, flag1=True, flag2=True, flag3=True):
  #   Run large amounts of scenes
  exp = exp_name+'/'+problem
  for n in N_list:
    for m in M_list:
      for dur in duration_list:
        i = 1
        while(i<=25):
          scen_name = "scen-random/"+case+"-random-"+str(i)+".scen"
          if problem == 'msmp':
            cbxs_baseline_dict, cbxs_old_res_dict, cbxs_res_dict = run_CBSS_MSMP(case,
                            os.path.join(cur_path, 'data', case, map),
                            os.path.join(cur_path, 'data', case, scen_name),
                            N = n, M = m, idx = i, duration = dur, exp_name = exp, sim=sim, flag1=flag1, flag2=flag2, flag3=flag3)
          else:
            cbxs_baseline_dict, cbxs_old_res_dict, cbxs_res_dict = run_CBSS_MCPFD(case,
                            os.path.join(cur_path, 'data', case, map),
                            os.path.join(cur_path, 'data', case, scen_name),
                            N = n, M = m, idx = i, duration = dur, exp_name = exp, sim=sim, flag1=flag1, flag2=flag2, flag3=flag3)
          i += 1

if __name__ == '__main__':
  print("begin of main")
  # EXPERIMENT:
  exp_name = "exp"
  case1_name = "random-32-32-20"
  case2_name = "maze-32-32-2"
  map1_name = case1_name +".map"
  map2_name = case2_name +".map"
  problem1 = 'msmp' 
  problem2 = 'mcpfd'
  sim = True
  # N_list = [5,10,20]
  # M_list = [10,20,30,40,50]
  # duration_list = [2,5,10,20]
  N_list = [5]
  M_list = [10]
  duration_list = [2]
  RunExp(exp_name, case1_name, map1_name, problem1, N_list, M_list, duration_list, sim, True, True, True)
  # RunExp(exp_name, case2_name, map2_name, problem1, N_list, M_list, duration_list, sim, True, True, True)
  # RunExp(exp_name, case1_name, map1_name, problem2, N_list, M_list, duration_list, sim, True, True, True)
  # RunExp(exp_name, case2_name, map2_name, problem2, N_list, M_list, duration_list, sim, True, True, True)

  print("end of main")