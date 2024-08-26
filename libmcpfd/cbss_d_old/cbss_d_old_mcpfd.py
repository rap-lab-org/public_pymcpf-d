"""
Author: Zhongqiang (Richard) Ren
Version@2021-07
All Rights Reserved
ABOUT: this file constains CBSS-D-MCPFD, which is derived from CBSS (framework) and aim to solve mcpfd problems.
"""

import libmcpfd.cbss_d_old.cbss_d_old as cbss_d
import libmcpfd.seq_msmp as seq_msmp
import libmcpfd.seq_mcpfd as seq_mcpfd

class CbssDMCPFD(cbss_d.CbssDFramework) :
  """
  """
  def __init__(self, grids, starts, goals, dests, ac_dict, ag_dict, configs):
    """
    """
    if configs["problem_str"]=='msmp':
      mtsp_solver = seq_msmp.SeqMSMP(grids, starts, goals, dests, ac_dict, configs)
    else: mtsp_solver = seq_mcpfd.SeqMCPFD(grids, starts, goals, dests, ac_dict, configs) # NOTE that ac_dict is only used in mtsp_solver, not in CBSS itself.
    super(CbssDMCPFD, self).__init__(mtsp_solver, grids, starts, goals, dests, ac_dict, ag_dict, configs)
    return

def RunCbssMCPFD(grids, starts, targets, dests, ac_dict, ag_dict, configs):
  """
  starts, targets and dests are all node ID.
  heu_weight and prune_delta are not in use. @2021-05-26
  """
  cbss_d_planner = CbssDMCPFD(grids, starts, targets, dests, ac_dict, ag_dict, configs)
  path_set, search_res, cstr_set, sp_conf = cbss_d_planner.Search()
  # print(path_set)
  # print(res_dict)
  res_dict = dict()
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
