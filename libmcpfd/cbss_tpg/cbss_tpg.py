"""
Author: Zhongqiang (Richard) Ren
All Rights Reserved.
ABOUT: this file contains CBXS framework (abstract).
Oeffentlich fuer: RSS22
"""

import stn_mcpf

import kbtsp as kb
import libmcpfd.cbss_lowlevel as sipp
import common as cm
import copy
import time
import numpy as np
import sys
import os
cur_path = os.getcwd()

DEBUG_CBXS = 0
DEBUG_SAVE = 0

class CbsConstraint:
  """
  borrowed from my previous code.
  """
  def __init__(self, i, va,vb, ta,tb, j=-1, flag=-1):
    """
    create a CCbsConstraint, if a single point, then va=vb
    """
    self.i = i # i<0, iff not valid
    self.va = va
    self.vb = vb
    self.ta = ta
    self.tb = tb
    self.j = j
    self.flag = flag # flag = 1, vertex conflict, flag = 2 swap conflict

  def __str__(self):
    return "{i:"+str(self.i)+",va:"+str(self.va)+",vb:"+str(self.vb)+\
      ",ta:"+str(self.ta)+",tb:"+str(self.tb)+",j:"+str(self.j)+",flag:"+str(self.flag)+"}"

def FindPathCost(p):
  """
  find the cost of a path
  """
  last_idx = -2
  for idx in range(len(p[0])): # find last loc_id that reach goal
    i1 = len(p[0]) - 1 - idx # kth loc id
    i2 = i1-1 # (k-1)th loc id
    if i2 < 0:
      break
    if p[0][i2] == p[0][i1]:
      last_idx = i2
    else:
      break
  # print("## single path: ", p)
  # print("## last_idx: ", last_idx)
  return p[1][last_idx] - p[1][0] # why does the path-cost only depends on time?

class CbsSol:
  """
  The solution in CBS high level node. A dict of paths for all robots.
  """
  def __init__(self):
    self.paths = dict()
    return

  def __str__(self):
    return str(self.paths)

  def AddPath(self, i, lv, lt):
    """
    lv is a list of loc id
    lt is a list of time
    """
    nlv = lv
    nlt = lt
    # add a final infinity interval
    nlv.append(lv[-1])
    nlt.append(np.inf)
    self.paths[i] = [nlv,nlt]
    return 

  def DelPath(self, i):
    self.paths.pop(i)
    return

  def GetPath(self, i):
    return self.paths[i]

  def CheckConflict(self, i, j, ac_dict, target_timeline):
    """
    return the first constraint found along path i and j.
    If no conflict, return empty list.
    """
    ix = 0
    sp_flag = 0
    while ix < len(self.paths[i][1])-1:
      for jx in range(len(self.paths[j][1])-1):
        jtb = self.paths[j][1][jx+1]
        jta = self.paths[j][1][jx]
        itb = self.paths[i][1][ix+1]
        ita = self.paths[i][1][ix]
        iva = self.paths[i][0][ix] 
        ivb = self.paths[i][0][ix+1]
        jva = self.paths[j][0][jx]
        jvb = self.paths[j][0][jx+1]
        overlaps, t_lb, t_ub = cm.ItvOverlap(ita,itb,jta,jtb)
        if not overlaps:
          continue
        if ivb == jvb: # vertex conflict at ivb (=jvb)
          if DEBUG_CBXS:
            print("i:%d, j:%d, ivb:%d, t_lb:%d" % (i, j, ivb, t_lb), "\n ac_dict: ", ac_dict, "\n timeline: ", target_timeline)
          if(ivb not in ac_dict or (i in ac_dict[ivb] and ac_dict[ivb][i]==1) 
             or (j in ac_dict[ivb] and ac_dict[ivb][j]==1)): # normal vertex conflict, because ivb(jvb) is not even a target
            pass
          elif(i in ac_dict[ivb] and i == target_timeline[ivb][0] and t_lb+1 >= target_timeline[ivb][1]): # if i is executing tasks in target ivb then j must be not
            sp_flag = 1
          elif(j in ac_dict[ivb] and j == target_timeline[ivb][0] and t_lb+1 >= target_timeline[ivb][1]): # if j is executing tasks in target ivb then i must be not
            sp_flag = 2
          else: # normal vertex conflict, though ivb(jvb) is a target
            pass
          return {'type':1, 'i':i, 'j':j, 'vb':ivb, 't_lb':t_lb},[[CbsConstraint(i, ivb, ivb, t_lb+1, t_lb+1, j, 1)], [CbsConstraint(j, jvb, jvb, t_lb+1, t_lb+1, i, 1)]], sp_flag # t_ub might be inf?
          # use min(itb,jtb) to avoid infinity
        elif (ivb == jva) and (iva == jvb): # swap location
          return {'type':2, 'i':i, 'j':j, 'iva':iva, 'jva':jva, 't_lb':t_lb},[[CbsConstraint(i, iva, ivb, t_lb, t_lb+1, j, 2)], [CbsConstraint(j, jva, jvb, t_lb, t_lb+1, i, 2)]], sp_flag 
      ix = ix + 1
    return {}, [], False

  def ComputeCost(self, flag=DEBUG_CBXS):
    """
    """
    sic = 0
    for k in self.paths:
      sic = sic + FindPathCost(self.paths[k])
      if DEBUG_CBXS:
        print("self.paths[%d] is:" % k, self.paths[k])
        print("FindPathCost(self.paths[%d]) is: %f" % (k,FindPathCost(self.paths[k])))
    return sic

class CbssTPGNode:
  """
  CBSS-TPG
  High level search tree node
  """
  def __init__(self, id0, sol=CbsSol(), cstr=[CbsConstraint(-1,-1,-1,-1,-1,-1)], c=0, parent=-1):
    """
    id = id of this high level CT node
    sol = an object of type CCbsSol.
    cstr = a list of CCbsConstraint, either empty or of length 2.
      newly added constraint in this node, to get all constraints, 
      need to backtrack from this node down to the root node.
    parent = id of the parent node of this node.
    """
    self.id = id0
    self.sol = sol
    self.cstr = cstr
    self.cost = c
    self.parent = -1 # root node
    self.root_id = -1
    self.root_num = -1
    return

  def __str__(self):
    str1 = "{id:"+str(self.id)+",c:"+str(self.cost)+",par:"+str(self.parent)
    return str1+",cstr:"+str(self.cstr)+",sol:"+str(self.sol)+"}"

  def CheckConflict(self, ac_dict, target_timeline):
    """
    check for conflicts along paths of all pairs of robots.
    record the first one conflict.
    Note that one conflict should be splited to 2 constraints.
    """
    done_set = set()
    for k1 in self.sol.paths:
      for k2 in self.sol.paths:
        if k2 in done_set or k2 == k1:
          continue
        # check for collision
        conflict, res, sp_flag = self.sol.CheckConflict(k1,k2, ac_dict, target_timeline)
        if len(res) > 0:
          # self.cstr = res # update member
          return conflict, res, sp_flag
      # end for k2
      done_set.add(k1)
    # end for k1
    return {}, [],False # no conflict

  def ComputeCost(self, flag=DEBUG_CBXS):
    """
    compute sic cost, update member, also return cost value
    """
    self.cost = self.sol.ComputeCost(flag)
    return self.cost

class CbssTPGFramework:
  """
  """
  def __init__(self, mtsp_solver, grids, starts, goals, dests, ac_dict, configs):
    """
    grids is 2d static grids.
    """
    self.tstart = time.perf_counter() # must be re-initialized in Search()
    self.grids = grids
    (self.yd, self.xd) = self.grids.shape
    self.starts = starts
    self.goals = goals
    self.dests = dests
    self.total_num = len(starts) + len(dests) + len(goals)
    # print("### init ac_dict:", ac_dict)
    self.ac_dict = ac_dict
    self.num_robots = len(starts)
    self.eps = configs["eps"]
    self.configs = configs
    self.nodes = dict() # high level nodes
    self.open_list = cm.PrioritySet()
    self.closed_set = set()
    self.num_closed_low_level_states = 0
    self.total_low_level_time = 0
    self.num_low_level_calls = 0
    self.node_id_gen = 1
    self.root_set = set() # a set of all root IDs.
    self.root_seq_dict = dict() # map a root ID to its joint sequence
    self.mtsp = mtsp_solver
    self.kbtsp = kb.KBestMTSP(self.mtsp)
    self.next_seq = None
    self.eps_cost = np.inf
    self.conflict_set = list()
    self.err_cstr = list()
    self.target_timeline = {1:[]}
    self.num_roots = -1
    self.sp_conflict = list()
    self.path_set = dict()
    self.reached_goal_id = -1
    self.agent_timeline = {1:[]}
    # self.STN_graph = STNgraph(0)
    self.start_idx = dict()
    self.end_idx = dict()
    self.start_idx[0] = 0
    return

  def BacktrackCstrs(self, nid, ri = -1, t0 = 0):
    """
    given a node, trace back to the root, find all constraints relavant.
    """
    node_cs = list()
    swap_cs = list()
    cid = nid
    if ri < 0:
      ri = self.nodes[nid].cstr[0].i
    # if ri < 0, then find constraints related to robot ri.
    while cid != -1:
      # print("cid = ",cid)
      if self.nodes[cid].cstr[0].i == ri: # not a valid constraint
        # init call of mocbs will not enter this.
        cstr = self.nodes[cid].cstr # here cstr can be a CbsConstraint() or a list of CbsConstraint
        if len(cstr) == 0:
          pass
        elif len(cstr) == 1:
          if self.nodes[cid].cstr[0].flag == 1: # vertex constraint
            if(cstr[0].tb >= t0): node_cs.append( (cstr[0].vb, cstr[0].tb) )
          elif self.nodes[cid].cstr[0].flag == 2: # swap constraint
            if(cstr[0].ta >= t0): swap_cs.append( (cstr[0].va, cstr[0].vb, cstr[0].ta) )
            if(cstr[0].tb >= t0): node_cs.append( (cstr[0].va, cstr[0].tb) ) # since another robot is coming to v=va at t=tb
        else: # must be special vertex constraint
          # print("!!!!!!!!!!!!!!!!!!!! Backtrack mutiple constraints: ")
          for k in range(len(cstr)):
            if(cstr[0].tb >= t0): 
              node_cs.append( (cstr[k].vb, cstr[k].tb) )
              # print("cstr[%d] is: " % k, cstr[k])
      cid = self.nodes[cid].parent
    return node_cs, swap_cs
  
  def _IfNewRoot(self, curr_node):
    """
    """
    if DEBUG_CBXS:
      print("### curr_nid:%d, curr_cost:%f" % (curr_node.id, curr_node.cost))
      print("### eps_cost:", self.eps_cost)
    cval = curr_node.cost
    if cval > self.eps_cost:
      if not self.next_seq: # next_seq not computed yet, compute next seq
        tlimit = self.time_limit - (time.perf_counter() - self.tstart)
        flag = self.kbtsp.ComputeNextBest(tlimit, self.total_num)
        if not flag: # no joint sequence any more.
          self.next_seq = None
        else:
          self.next_seq = self.kbtsp.GetKthBestSol() # will be used to check if new root needs to be generated.
    else:
      return False

    ### if reach here, must be the case: cval > (1+eps)*curr_root_cost.
    if DEBUG_CBXS:
      print("### CBXS _IfNewRoot 2nd phase, input cost = ", cval, " eps_cost = ", self.eps_cost, " next_cost = ", (1+self.eps)*self.next_seq.cost)
    if (self.next_seq is None):
      self.eps_cost = np.inf # no next root!
      return False
    else:
      if (cval > self.next_seq.cost):
        return True
      else: # now what?
        return False

  def _GenCbxsNode(self, nid):
    return CbssTPGNode(nid)

  def _UpdateEpsCost(self, c):
    self.eps_cost = (1+self.eps)*c # update eps cost.
    # print(" _UpdateEpsCost input ", c, " eps_cost = ", self.eps_cost)
    return

  def _GenNewRoot(self):
    """
    called at the beginning of the search. 
    generate first High level node.
    compute individual optimal path for each robot.
    """

    ### Generate the first HL node, a root node ###
    nid = self.node_id_gen
    self.nodes[nid] = self._GenCbxsNode(nid)
    self.node_id_gen = self.node_id_gen + 1
    self.num_roots += 1
    self.root_set.add(nid)
    self.nodes[nid].root_id = nid
    self.nodes[nid].root_num = self.num_roots
    if DEBUG_CBXS:
      print("### nid is:", nid)
    ### Init sequencing related ###
    if not self.next_seq:
      if (nid == 1): # init
        tlimit = self.time_limit - (time.perf_counter() - self.tstart)
        flag = self.kbtsp.ComputeNextBest(tlimit, self.total_num)
        if not flag:
          print("[ERROR] CBXS: No feasible joint sequence or time out at init!")
          sys.exit("[ERROR]")
        self.root_seq_dict[nid] = self.kbtsp.GetKthBestSol() # assign seq data to root node.
      else:
        return False # no new root to be generated.
    else: # during search
      self.root_seq_dict[nid] = self.next_seq
      self.next_seq = None # next_seq has been used, make it empty.
    ### init target/agent_timeline
    self.target_timeline[nid]={}
    self.agent_timeline[nid]={}
    ### plan path based on goal sequence for all agents ###
    if DEBUG_CBXS:
      print("### self.root_seq_dict[nid] is:", self.root_seq_dict[nid])
      print("### cstr")
      for cstr in self.nodes[nid].cstr:
        print(cstr)
    for ri in range(self.num_robots): # loop over agents, plan their paths
      lv, lt, stats = self.Lsearch(nid, ri)
      if len(lv) == 0: # fail to init, time out or sth.
        return False
      self.nodes[nid].sol.AddPath(ri,lv,lt)
    if DEBUG_CBXS and DEBUG_SAVE:
      if(nid==10):
        print("### Test id")
        cm.savePath(self.nodes[nid].sol, os.path.join(cur_path, 'test', 'cbxs.pickle'))
    ### update cost and insert into OPEN ###
    c = self.nodes[nid].ComputeCost() # update node cost and return cost value
    if DEBUG_CBXS:
      print("new cost is:", c)
    self.open_list.add(c,nid)
    self._UpdateEpsCost(c)
    return True

  def Lsearch(self, nid, ri):
    """
    input a high level node, ri is optional(why optional?).
    """
    if DEBUG_CBXS:
      print("Lsearch, nid:",nid)
    nd = self.nodes[nid]
    root_num = self.nodes[nid].root_num
    tlimit = self.time_limit - (time.perf_counter() - self.tstart)

    # plan from start to assigned goals and to dest as specified in goal sequence
    gseq = self.root_seq_dict[self.nodes[nid].root_id].sol[ri]
    ss = gseq[0]
    kth = 1
    t0 = 0
    all_lv = []
    all_lv.append(self.starts[ri])
    all_lt = []
    all_lt.append(0)
    success = True
    for kth in range(1, len(gseq)):
      # TODO, this can be optimized, 
      # no need to plan path between every pair of waypoints each time! Impl detail.
      # TODO: for CBXS, low-level search need to be modified on the special targets(repeat)
      gg = gseq[kth]
      ignore_goal_cstr = True
      if kth == len(gseq)-1: # last goal
        ignore_goal_cstr = False
      lv, lt, sipp_stats = self.LsearchP2P(nid, ri, ss, gg, t0, ignore_goal_cstr, self.ac_dict[gg][ri])
      if DEBUG_CBXS:
        print("---LsearchP2P--- for agent ", ri, ", ignore_goal_cstr = ", ignore_goal_cstr, ", lv = ", lv, ", lt = ", lt, ", t0 = ", t0)
      if len(lv) == 0: # failed
        print("-----failed for Lsearch")
        success = False
        break
      else: # success
        self.UpdateStats(sipp_stats)
        # check for the need to modify the path for the target with tasks
        if(gg in self.ac_dict and ri in self.ac_dict[gg] and self.ac_dict[gg][ri]>=1):
          if(nid not in self.target_timeline):
            self.target_timeline[nid] = copy.deepcopy(self.target_timeline[self.nodes[nid].parent])
            self.agent_timeline[nid] = copy.deepcopy(self.agent_timeline[self.nodes[nid].parent])
            self.agent_timeline[nid][ri] = {gg: [lt[-1], lt[-1] + self.ac_dict[gg][ri]-1]} # 'ri': {targeti: [start, end], ...} 
          else:
            if(ri not in self.agent_timeline[nid]):
              self.agent_timeline[nid][ri] = {gg: [lt[-1], lt[-1] + self.ac_dict[gg][ri]-1]} # 'ri': {targeti: [start, end], ...}
            else:
              self.agent_timeline[nid][ri].update({gg: [lt[-1], lt[-1] + self.ac_dict[gg][ri]-1]}) # 'ri': {targeti: [start, end], ...}
          self.target_timeline[nid][gg] = [ri, lt[-1], lt[-1] + self.ac_dict[gg][ri]-1] # 'target': [start, end]
          for _ in range(self.ac_dict[gg][ri]-1):
            lv.append(lv[-1])
            lt.append(lt[-1]+1)
        all_lv, all_lt = self.ConcatePath(all_lv, all_lt, lv, lt)
      ss = gg # update start for the next call
      t0 = lt[-1]
    # end for kth
    if not success:
      return [], [], success
    else:
      # print("### Lsearch path lv: ", all_lv)
      # print("### Lsearch path lt: ", all_lt)
      return all_lv, all_lt, success

  def LsearchP2P(self, nid, ri, ss, gg, t0, ignore_goal_cstr, task_duration):
    """
    Do low level search for agent-i from vertex ss with starting time step t0
      to vertex gg subject to constraints in HL node nid.
    """
    nd = self.nodes[nid]
    if ri < 0: # to support init search.
      ri = nd.cstr.i
    tlimit = self.time_limit - (time.perf_counter() - self.tstart)
    ncs, ecs = self.BacktrackCstrs(nid, ri, t0)
    # print("### ncs:", ncs)
    # print("### ecs:", ecs)
    # plan from start to assigned goals and to dest as specified in goal sequence
    ssy = int(np.floor(ss/self.xd)) # start y
    ssx = int(ss%self.xd) # start x
    ggy = int(np.floor(gg/self.xd)) # goal y
    ggx = int(gg%self.xd) # goal x
    res_path, sipp_stats = sipp.RunSipp(self.grids, ssx, ssy, \
      ggx, ggy, t0, ignore_goal_cstr, 1.0, 0.0, tlimit, ncs, ecs, task_duration) # note the t0 here!
    if len(res_path)==0:
      return [],[],sipp_stats
    else:
      return res_path[0], res_path[1], sipp_stats
  
  def ConcatePath(self, all_lv, all_lt, lv, lt):
    """
    remove(why remove?) the first node in lv,lt and then concate with all_xx.
    """
    if (len(all_lt) > 0) and (lt[0] != all_lt[-1]):
      print("[ERROR] ConcatePath lv = ", lv, " lt = ", lt, " all_lv = ", all_lv, " all_lt = ", all_lt)
      sys.exit("[ERROR] ConcatePath, time step mismatch !")
    return all_lv + lv[1:], all_lt + lt[1:]

  def FirstConflict(self, nd):
    return nd.CheckConflict(self.ac_dict, self.target_timeline[nd.id])

  def UpdateStats(self, stats):
    """
    """
    if DEBUG_CBXS:
      print("UpdateStats, ", stats)
    self.num_closed_low_level_states = self.num_closed_low_level_states + stats[0]
    self.total_low_level_time = self.total_low_level_time + stats[2]
    return

  def ReconstructPath(self, nid):
    """
    """
    path_set = dict()
    for i in range(self.num_robots):
      lx = list()
      ly = list()
      lv = self.nodes[nid].sol.GetPath(i)[0]
      for v in lv:
        y = int(np.floor(v / self.xd))
        x = int(v % self.xd)
        ly.append(y)
        lx.append(x)
      lt = self.nodes[nid].sol.GetPath(i)[1]
      path_set[i] = [lx,ly,lt]
    return path_set

  def _HandleRootGen(self, curr_node):
    """
    generate new root if needed
    """  
    # print(" popped node ID = ", curr_node.id)
    if self._IfNewRoot(curr_node):
      if DEBUG_CBXS:
        print(" ### CBXS _GenNewRoot...")
      self._GenNewRoot()
      self.open_list.add(curr_node.cost, curr_node.id) # re-insert into OPEN for future expansion.
      popped = self.open_list.pop() # pop_node = (f-value, high-level-node-id)
      curr_node = self.nodes[popped[1]]
    else:
      # print(" self._IfNewRoot returns false...")
      place_holder = 1
    # end of if/while _IfNewRoot
    # print("### CBXS, expand high-level node ID = ", curr_node.id)
    return curr_node

  def SortTimeline(self, tar_timeline):
    sort_timeline = list()
    for tar in tar_timeline:
      sort_timeline.append([tar, tar_timeline[tar][0], tar_timeline[tar][1], tar_timeline[tar][2]])
    sort_timeline.sort(key=lambda x: x[2])
    return sort_timeline

  ## finish the post-process based on STN
  def PostSTNProcess(self, ac_dict, search_res):
    """
    Ren: I changed this function. Simply replace the the old code with the new one.
    """

    ######
    ###### Ren, change starts, @2023-09-23
    ######

    id = self.reached_goal_id
    if id==-1:
      return {},search_res
    # print("####### Post self.target_timeline[%d] is: " % id, self.target_timeline[id])
    (stn, d_dict, next_dict) = stn_mcpf.Sol2STN(self.nodes[id].sol.paths, ac_dict, self.target_timeline[id]) # Ren, new code

    post_paths = stn_mcpf.ExtractPaths(stn, d_dict, self.starts, next_dict) # Ren, new code

    # Ren: below are copied from your old code.

    print("! BASELINE succeed !")
    print("### post-process paths: ", post_paths)
    ### tranform paths to path_set
    path_set = dict()
    for i in range(self.num_robots):
      lx = list()
      ly = list()
      lv = post_paths[i][0]
      for v in lv:
        y = int(np.floor(v / self.xd))
        x = int(v % self.xd)
        ly.append(y)
        lx.append(x)
      lt = post_paths[i][1]
      path_set[i] = [lx,ly,lt]
    self.path_set = path_set
    self.nodes[id].sol.paths = post_paths
    self.nodes[id].ComputeCost(True)
    print("baseline solution root sequence: ", self.root_seq_dict[self.nodes[id].root_id])
    print("baseline solution: ", self.nodes[id].sol.paths)
    conflict, cstrs,sp_flag = self.FirstConflict(self.nodes[id])
    if(len(cstrs)!=0):
      print("!!!!!!!!!!!!!!!conflict detected!!!!!!!!!!!!!!!")
      print("conflict is: ", conflict, sp_flag)
      print("cstrs is: ", cstrs)
      # exit(1)
    else: print("!!!!no conflict!!!!")
    ### update search_res
    search_res[1] = self.nodes[id].cost
    search_res[6] = time.perf_counter() - self.tstart
    return self.path_set, search_res

    ######
    ###### RR, change ends, @2023-09-23
    ######

  def Search(self):
    """
    = high level search
    """
    print("CBXS search begin!")
    self.time_limit = self.configs["time_limit"]
    self.tstart = time.perf_counter()

    good = self._GenNewRoot()
    if not good:
      output_res = [ int(0), float(-1), int(0), int(0), \
        int(self.num_closed_low_level_states), 0, float(time.perf_counter()-self.tstart), \
        int(self.kbtsp.GetTotalCalls()), float(self.kbtsp.GetTotalTime()), int(len(self.root_set)) ]
      return dict(), output_res
    
    tnow = time.perf_counter()
    # print("After init, tnow - self.tstart = ", tnow - self.tstart, " tlimit = ", self.time_limit)
    if (tnow - self.tstart > self.time_limit):
      print(" FAIL! timeout! ")
      search_success = False
      output_res = [ int(0), float(-1), int(0), int(0), \
        int(self.num_closed_low_level_states), 0, float(time.perf_counter()-self.tstart), \
        int(self.kbtsp.GetTotalCalls()), float(self.kbtsp.GetTotalTime()), int(len(self.root_set)) ]
      return dict(), output_res

    search_success = False
    best_g_value = -1
    reached_goal_id = -1
    while True:
      tnow = time.perf_counter()
      rd = len(self.closed_set)
      # print("tnow - self.tstart = ", tnow - self.tstart, " tlimit = ", self.time_limit)
      if (tnow - self.tstart > self.time_limit):
        print(" FAIL! timeout! ")
        search_success = False
        break
      if (self.open_list.size()) == 0:
        print(" FAIL! openlist is empty! ")
        search_success = False
        break

      popped = self.open_list.pop() # pop_node = (f-value, high-level-node-id)
      # print("### open_list size:", self.open_list.size())
      curr_node = self.nodes[popped[1]]
      curr_node = self._HandleRootGen(curr_node) # generate new root if needed
      tnow = time.perf_counter()
      # print("tnow - self.tstart = ", tnow - self.tstart, " tlimit = ", self.time_limit)

      if (tnow - self.tstart > self.time_limit):
        print(" FAIL! timeout! ")
        search_success = False
        break

      self.closed_set.add(popped[1]) # only used to count numbers

      if DEBUG_CBXS:
        print("### CBXS popped node: ", curr_node)
      
      conflict, cstrs, sp_flag = self.FirstConflict(curr_node)

      if len(cstrs) == 0: # no conflict, terminates
        print("! CBXS succeed !")
        search_success = True
        best_g_value = curr_node.cost
        reached_goal_id = curr_node.id
        self.reached_goal_id = reached_goal_id
        break

      max_child_cost = 0
      resolve_flag = False
      for cstr in cstrs:
        if DEBUG_CBXS:
          print("#### CBXS loop over cstr:",cstr)
        ### generate constraint and new HL node ###
        new_id = self.node_id_gen
        self.node_id_gen = self.node_id_gen + 1
        self.nodes[new_id] = copy.deepcopy(curr_node)
        self.nodes[new_id].id = new_id
        self.nodes[new_id].parent = curr_node.id
        self.nodes[new_id].cstr = cstr
        self.nodes[new_id].root_id = self.nodes[curr_node.id].root_id # copy root id.
        self.nodes[new_id].root_num = self.nodes[curr_node.id].root_num
        ri = cstr[0].i

        ### TODO:check special constraint
        c_ri = FindPathCost(self.nodes[new_id].sol.paths[ri])
      
        ### replan paths for the agent, subject to new constraint ###
        lv,lt,flag = self.Lsearch(new_id, ri)
        self.num_low_level_calls = self.num_low_level_calls + 1 
        if len(lv)==0:
          # this branch fails, robot ri cannot find a consistent path.
          print("**************************robot %d cannot find a consistent path" % (ri))
          continue
        elif resolve_flag==False:
          resolve_flag = True

        self.nodes[new_id].sol.DelPath(ri)
        self.nodes[new_id].sol.AddPath(ri,lv,lt)
        nn_cost = self.nodes[new_id].ComputeCost()
        if DEBUG_CBXS:
          print("### CBXS add node ", self.nodes[new_id], " into OPEN,,, nn_cost = ", nn_cost, ", nid = ", new_id)
        self.open_list.add(nn_cost, new_id)
        max_child_cost = np.max([nn_cost, max_child_cost])
      # end of for cstr
      if resolve_flag:
        self.conflict_set.append(conflict)
        if(sp_flag):
          self.sp_conflict.append(conflict)
      # print(">>>>>>>>>>>>>>>>>>>> end of an iteration")
    # end of while

    output_res = [ int(len(self.closed_set)), float(best_g_value), int(0), int(self.open_list.size()), \
      int(self.num_closed_low_level_states), int(search_success), float(time.perf_counter()-self.tstart),\
      int(self.kbtsp.GetTotalCalls()), float(self.kbtsp.GetTotalTime()), int(len(self.root_set)),\
      int(len(self.conflict_set)),int(len(self.sp_conflict)),self.kbtsp.init_graph_time, self.target_timeline[reached_goal_id]]
    # print("############ cstrs[0]: \n", self.cstr_set[0])
    if search_success:
      print("#### search path is: ", self.nodes[reached_goal_id].sol.paths)
      print("compute cost is: ")
      self.nodes[reached_goal_id].ComputeCost(True)
      self.path_set = self.ReconstructPath(reached_goal_id)
      return self.path_set, output_res, self.conflict_set, self.sp_conflict
    else:
      return self.path_set, output_res, self.conflict_set, self.sp_conflict
    