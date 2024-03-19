"""
Author: Zhongqiang (Richard) Ren
Version@2021-07
All Rights Reserved
ABOUT: this file constains CBXS-mcpfd-AC, which is derived from CBSS (framework) and aim to solve mcpfd-AC problems.
"""

import libmcpfd.cbss_tpg.cbss_tpg as cbss_tpg
import libmcpfd.seq_msmp as seq_msmp
import libmcpfd.seq_mcpf as seq_mcpf
import time

class CbssTPGMCPFD(cbss_tpg.CbssTPGFramework) :
  """
  """
  def __init__(self, grids, starts, goals, dests, ac_dict, configs):
    """
    """
    ac_dict_mcpf = dict()
    ac_dict_mcpfd = dict()
    for tar in ac_dict:
      ac_dict_mcpf[tar] = list()
      ac_dict_mcpfd[tar] = dict()
      for ag in ac_dict[tar]:
        ac_dict_mcpf[tar].append(ag)
        ac_dict_mcpfd[tar][ag] = 1
    if configs["problem_str"]=='msmp':
      mcpf_solver = seq_msmp.SeqMSMP(grids, starts, goals, dests, ac_dict_mcpf, configs)
    else: mcpf_solver = seq_mcpf.SeqMCPF(grids, starts, goals, dests, ac_dict_mcpf, configs) # NOTE that ac_dict is only used in mcpfd_solver, not in CBSS itself.
    super(CbssTPGMCPFD, self).__init__(mcpf_solver, grids, starts, goals, dests, ac_dict_mcpfd, configs)
    return

def RunCbssMCPFD(grids, starts, targets, dests, ac_dict, configs):
  """
  starts, targets and dests are all node ID.
  heu_weight and prune_delta are not in use. @2021-05-26
  """
  cbxs_planner = CbssTPGMCPFD(grids, starts, targets, dests, ac_dict, configs)
  path_set, search_res, cstr_set, sp_conf = cbxs_planner.Search()
  res_dict = dict()
  t0 = time.perf_counter()
  path_set, search_res = cbxs_planner.PostSTNProcess(ac_dict, search_res)
  res_dict["postprocess_time"] = time.perf_counter() - t0
  # print(path_set)
  # print(res_dict)
  res_dict["path_set"] = path_set
  res_dict["cstr_set"] = cstr_set
  res_dict["sp_conf"] = sp_conf
  res_dict["round"] = search_res[0] # = num of high level nodes closed.
  res_dict["best_g_value"] = search_res[1]
  res_dict["open_list_size"] = search_res[3]
  res_dict["num_low_level_expanded"] = search_res[4]
  res_dict["search_success"] = search_res[5]
  res_dict["search_time"] = search_res[6]
  res_dict["n_tsp_call"] = search_res[7]
  res_dict["n_tsp_time"] = search_res[8]
  res_dict["n_roots"] = search_res[9]
  res_dict["n_conf"] = search_res[10]
  res_dict["n_sp_conf"] = search_res[11]
  res_dict["init_graph_time"] = search_res[12]
  res_dict["target_timeline"] = search_res[13]

  return res_dict
