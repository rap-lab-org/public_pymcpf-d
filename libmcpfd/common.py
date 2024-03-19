"""
Author: Zhongqiang (Richard) Ren
All Rights Reserved.
ABOUT: Utility.
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq as hpq
import matplotlib.cm as cm
import json
import pickle
import os
import copy

class PrioritySet(object):
  """
  priority queue, min-heap
  """
  def __init__(self):
    """
    no duplication allowed
    """
    self.heap_ = []
    self.set_ = set()
    
  def add(self, pri, d):
    """
    will check for duplication and over-write.
    """
    if d in self.set_:
      self.remove(d)
    hpq.heappush(self.heap_, (pri, d))
    self.set_.add(d)
    return

  def pop(self):
    """
    impl detail: return the first(min) item that is in self.set_
    """
    pri, d = hpq.heappop(self.heap_)
    while d not in self.set_:
      pri, d = hpq.heappop(self.heap_)
    self.set_.remove(d)
    return pri, d
  
  def top(self):
    """
    impl detail: return the first(min) item that is in self.set_(not remove)
    """
    [(pri,d)] = hpq.nsmallest(1, self.heap_, key=lambda x: x[0])
    return pri, d
  
  def size(self):
    return len(self.set_)
  def print(self):
    print(self.heap_)
    print(self.set_)
    return
  def remove(self, d):
    """
    implementation: only remove from self.set_, not remove from self.heap_ list.
    """
    if not d in self.set_:
      return False
    self.set_.remove(d)
    return True


def gridAstar(grids, start, goal, w=1.0):
  """
  Four-connected Grid
  Return a path (in REVERSE order!)
  a path is a list of node ID (not x,y!)
  """
  output = list()
  (nyt, nxt) = grids.shape # nyt = ny total, nxt = nx total
  action_set_x = [-1,0,1,0]
  action_set_y = [0,-1,0,1]
  open_list = []
  hpq.heappush( open_list, (0, start) )
  close_set = dict()
  parent_dict = dict()
  parent_dict[start] = -1
  g_dict = dict()
  g_dict[start] = 0
  gx = goal % nxt
  gy = int(np.floor(goal/nxt))
  search_success = True
  while True:
    if len(open_list) == 0:
      search_success = False
      break
    cnode = hpq.heappop(open_list)
    cid = cnode[1]
    curr_cost = g_dict[cid]
    if cid in close_set:
      continue
    close_set[cid] = 1
    if cid == goal:
      break
    # get neighbors
    # action_idx_seq = np.random.permutation(5)
    cx = cid % nxt
    cy = int(np.floor(cid / nxt))
    for action_idx in range(len(action_set_x)):
      nx = cx + action_set_x[action_idx]
      ny = cy + action_set_y[action_idx]
      if ny < 0 or ny >= nyt or nx < 0 or nx >= nxt:
        continue
      if grids[ny,nx] > 0.5:
        continue
      nid = ny*nxt+nx
      heu = np.abs(gx-nx) + np.abs(gy-ny) # manhattan heu
      gnew = curr_cost+1
      if (nid) not in close_set:
        if (nid) not in g_dict:
          hpq.heappush(open_list, (gnew+w*heu, nid))
          g_dict[nid] = gnew
          parent_dict[nid] = cid
        else: 
          if (gnew < g_dict[nid]):
            hpq.heappush(open_list, (gnew+w*heu, nid))
            g_dict[nid] = gnew
            parent_dict[nid] = cid
  # end of while

  # reconstruct path
  if search_success:
    cid = goal
    output.append(cid)
    while parent_dict[cid] != -1 :
      cid = parent_dict[cid]
      output.append(cid)
  else:
    # do nothing
    print(" fail to plan !")
  return output

def getTargetGraph(grids,Vo,Vt,Vd):
  """
  Return a cost matrix of size |Vo|+|Vt|+|Vd| (|Vd|=|Vo|)
    to represent a fully connected graph.
  The returned spMat use node index (in list Vo+Vt+Vd) instead of node ID.
  """
  N = len(Vo)
  M = len(Vt)
  nn = N+M+N
  V = Vo + Vt + Vd
  spMat = np.zeros((nn,nn))
  for i in range(nn):
    for j in range(i+1,nn):
      spMat[i,j] = len( gridAstar(grids,V[i],V[j]) ) - 1
      spMat[j,i] = spMat[i,j]
  return spMat

def ItvOverlap(ita,itb,jta,jtb):
  """
  check if two time interval are overlapped or not.
  """
  if ita >= jtb or jta >= itb: # non-overlap
    return False, -1.0, -1.0
  # must overlap now
  tlb = jta # find the larger value among ita and jta, serve as lower bound
  if ita >= jta:
    tlb = ita
  tub = jtb # find the smaller value among itb and jtb, serve as upper bound
  if itb <= jtb:
    tub = itb
  return True, tlb, tub






#################################################################
##################### Non-public part below #####################
#################################################################


def SaveJson(data_dict, file_path):
  """
  save input dict as a json file
  """
  with open(file_path, "w") as write_file:
    json.dump(data_dict, write_file, cls=NpEncoder)
    return 1

# load from json file into a dict
def LoadJson(path):
  with open(path, "r") as read_file:
    data = json.load(read_file)
    return data
  return dict()

def IsDictSubset(dict1, dict2):
  """return if dict1 is a subset of dict2"""
  for k in dict1:
    if k not in dict2:
      return False
  return True

def GridPolicy(grids, sx, sy):
  """
  2d grids, grids[y,x] 
  generate a policy for 2d grids leading to (sx,sy), where (sx,sy) is the goal !!
  return np.array(policy), np.array(distmat)
  CAVEAT, this policy maps a state to its next state, instead of action!
  policy[y][x] = (x2,y2) !! Notice that input [y][x], output (x2,y2), not (y2,x2) !!
  distmat[y][x] = dist to goal from (x,y)
  """
  (nyt, nxt) = grids.shape # nyt = ny total, nxt = nx total
  policy = [[ (-1,-1) for x in range(nxt)] for y in range(nyt)] 
  distmat = [[ np.inf for x in range(nxt)] for y in range(nyt)] 
  # print("len policy = ", len(policy))
  action_set_x = [0,-1,0,1,0]
  action_set_y = [0,0,-1,0,1]
  open_list = []
  hpq.heappush( open_list, (0, sx,sy) )
  close_set = dict()
  parent_dict = dict()
  parent_dict[sy*nxt+sx] = -1
  policy[sy][sx] = (sx,sy)
  distmat[sy][sx] = 0
  g_dict = dict()
  g_dict[sy*nxt+sx]=0

  search_success = True
  while True:
    if len(open_list) == 0:
      search_success = False
      break
    cnode = hpq.heappop(open_list)
    cid = cnode[2]*nxt+cnode[1]
    # print(" cx = ", cnode[1], " cy = ", cnode[2], " cid = ", cid)
    if cid in close_set:
      continue
    close_set[cid] = 1
    # if cnode[1]==gx and cnode[2]==gy:
    #   break
    # get neighbors
    curr_cost = g_dict[cid]
    for action_idx in range(len(action_set_x)):
      nx = cnode[1] + action_set_x[action_idx]
      ny = cnode[2] + action_set_y[action_idx]
      if ny < 0 or ny >= nyt or nx < 0 or nx >= nxt:
        continue
      if grids[ny,nx] > 0.5:
        continue
      nid = ny*nxt+nx
      gnew = curr_cost+1
      if (nid) not in close_set:
        if (nid) not in g_dict:
          hpq.heappush(open_list, (gnew, nx, ny))
          g_dict[nid] = gnew
          distmat[ny][nx] = gnew
          parent_dict[nid] = cid
          policy[ny][nx] = (cnode[1], cnode[2])
        else: 
          if (gnew < g_dict[nid]):
            hpq.heappush(open_list, (gnew, nx, ny))
            g_dict[nid] = gnew
            distmat[ny][nx] = gnew
            parent_dict[nid] = cid
            policy[ny][nx] = (cnode[1], cnode[2])
  # end of while
  return np.array(policy), np.array(distmat)

def GenerateRandomGrids(ny = 50, nx = 50, obst_thres=0.15):
  """
  grid(ny,nx), obstacle threshold 0 = no obst, 1 = all obst.
  """
  sc_flag = True
  grids = np.floor(np.random.random((ny,nx)) + 0.0)

  while True:
    sc_flag = True # strongly connected flag
    grids = np.floor(np.random.random((ny,nx)) + 0.0) # ?? unnecessary use of numpy

    xfre,yfre = GetFreeXY(grids)
    xfree = np.array(xfre)
    yfree = np.array(yfre)
    idx_offset = int(len(xfree)*obst_thres)
    # np.random.seed()  
    idx_list = np.random.permutation(len(xfree))

    for ix in range(idx_offset):
      grids[yfree[idx_list[ix]], xfree[idx_list[ix]]] = 1
    
    # check if strongly-connected
    optm_policy, distmat = GridPolicy(grids, xfree[idx_list[idx_offset]], yfree[idx_list[idx_offset]])
    for ix in range(nx):
      for iy in range(ny):
        if (distmat[iy][ix] == np.inf) and (grids[iy,ix] == 0):
          sc_flag = False
          break
      if sc_flag == False:
        break
    if sc_flag == True:
      break
  return grids

def PlotGrids(grids):
  plt.imshow(grids, cmap='Greys',  interpolation='nearest', origin="lower")
  # plt.colorbar()
  plt.xlabel('x')
  plt.ylabel('y')
  sx,sy = grids.shape
  plt.xticks(np.arange(0, sx, 2))
  plt.yticks(np.arange(0, sy, 2))
  # plt.axis("off")
  # plt.show()

def PlotPathSet(path_dict, skip_set=dict(), colors = []):
  """
  visualize the input path_dict where path_dict[0] is a list of x coordinates and
  path_dict[1] is list of y coordinates 
  """
  for key in path_dict:
    # print(" v ", v)
    if key in skip_set:
      continue
    v = path_dict[key]
    if len(colors) < len(path_dict):
      plt.plot(v[0], v[1], alpha=0.37)
    else:
      plt.plot(v[0], v[1], "k--", alpha=0.2)
  return

#
def PlotPointSet(x_list, y_list, colors = [], mks="o", alpha0=1.0):
  """
  visualize the input x_list and y_list.
  """
  # plt.scatter(x_list,y_list, c=colors[0:len(x_list)], cmap='viridis', marker=mks, alpha=alpha0)
  # print("x_list = ", x_list, " y_list = ", y_list, " colors = ", colors)
  plt.scatter(x_list,y_list, c=colors, cmap='viridis', marker=mks, alpha=alpha0)

# TODO, replace all of this with GridAstar.
def WeightedAstar(grids, sx, sy, gx, gy, w=1.0):
  """
  Return a path (in REVERSE order!!) 
    that connects start and goal in grids in form:
    list of x coordinates, list of y coordinates = WeightedAstar()
  - 2d grids, grids[y,x], Assume four-connected grid.
  - generate a set of size n of random paths in non-obst area
  - sx,sy, gx,gy should not be on an obstacle
  """
  op_x = list()
  op_y = list()
  (nyt, nxt) = grids.shape # nyt = ny total, nxt = nx total
  # print("sizes = ", nyt, nxt)
  weight = w
  action_set_x = [-1,0,1,0]
  action_set_y = [0,-1,0,1]
  open_list = []
  hpq.heappush( open_list, (0, sx,sy) )
  close_set = dict()
  parent_dict = dict()
  parent_dict[sy*nxt+sx] = -1
  g_dict = dict()
  g_dict[sy*nxt + sx] = 0

  search_success = True
  while True:
    if len(open_list) == 0:
      search_success = False
      break
    cnode = hpq.heappop(open_list)
    cid = cnode[2]*nxt+cnode[1]
    curr_cost = g_dict[cid]
    # print(" cx = ", cnode[1], " cy = ", cnode[2], " cid = ", cid)
    if cid in close_set:
      continue
    close_set[cid] = 1
    if cnode[1]==gx and cnode[2]==gy:
      break
    # get neighbors
    # action_idx_seq = np.random.permutation(5)
    for action_idx in range(len(action_set_x)):
      nx = cnode[1] + action_set_x[action_idx]
      ny = cnode[2] + action_set_y[action_idx]
      if ny < 0 or ny >= nyt or nx < 0 or nx >= nxt:
        continue
      if grids[ny,nx] > 0.5:
        continue
      nid = ny*nxt+nx
      heu = np.abs(gx-nx) + np.abs(gy-ny) # manhattan heu
      gnew = curr_cost+1
      if (nid) not in close_set:
        if (nid) not in g_dict:
          hpq.heappush(open_list, (gnew+weight*heu, nx, ny))
          g_dict[nid] = gnew
          parent_dict[nid] = cid
        else: 
          if (gnew < g_dict[nid]):
            hpq.heappush(open_list, (gnew+weight*heu, nx, ny))
            g_dict[nid] = gnew
            parent_dict[nid] = cid
  # end of while

  # reconstruct path
  if search_success:
    cid = gy*nxt+gx
    op_x.append(gx)
    op_y.append(gy)
    while parent_dict[cid] != -1 :
      cid = parent_dict[cid]
      op_y.append(int(np.floor(cid/nxt)))
      op_x.append(cid%nxt)
  else:
    # do nothing
    print(" fail to plan !")
  return (op_x, op_y)

def GetFreeXY(grids):
  """
  xfree,yfree = GetFreeXY()
  return non-obstacle locations in input grids in form of lists.
  """
  op_x = list()
  op_y = list()
  (nyt, nxt) = grids.shape # nyt = ny total, nxt = nx total
  for ix in range(nxt):
    for iy in range(nyt):
      if grids[iy,ix] == 0:
        op_x.append(ix)
        op_y.append(iy)
  return op_x, op_y

def JointPathToPathSet(grids, jpath):
  """
  jpath here is a list of js, [js1,js2,...,jsK], js = joint configuration
  path_set is a dict, key is robot id, e.g. 1,2,3,...,N,
  path_set[robot_id][0] = (x1,x2,x3,x4,...,xK), x locations of all K steps for robot_id
  path_set[robot_id][1] = (y1,y2,y3,y4,...,yK), y locations of all K steps for robot_id
  """
  # print(jpath)
  nyt, nxt = grids.shape
  path_set = dict()
  if (len(jpath) == 0):
    return path_set
  for ida in range(len(jpath[0])): # total num of robots
    path_set[ida] = list()
    path_set[ida].append(list()) # for x
    path_set[ida].append(list()) # for y
    path_set[ida].append(list()) # for t
  for idx in range(len(jpath)): # loop over each js in jpath
    for idy in range(len(jpath[idx])): # loop over each cfg in jc, idy = robot_id
      # loc_id = jpath[len(jpath)-idx-1][0][idy] # convert from reverse order to normal order
      loc_id = jpath[idx][idy] # convert from reverse order to normal order
      cy = int(np.floor(loc_id/nxt))
      cx = int(loc_id%nxt)
      path_set[idy][0].append(cx)
      path_set[idy][1].append(cy)
  return path_set

def PathSetToJointPath(grids, path_set):
  """
  path_set is a dict, key is robot id, e.g. 1,2,3,...,N,
  path_set[robot_id][0] = (x1,x2,x3,x4,...,xK), x locations of all K steps for robot_id
  path_set[robot_id][1] = (y1,y2,y3,y4,...,yK), y locations of all K steps for robot_id  
  jpath here is a list of js, [js1,js2,...,jsK], js = joint configuration
  """
  nyt, nxt = grids.shape
  path_nid_set = dict()
  max_len = 0
  nrobot = len(path_set)
  for ri in range(nrobot):
    path_nid_set[ri] = list()
    for kth in range(len(path_set[ri][0])): # loop over all path nodes
      path_nid_set[ri].append(path_set[ri][1][kth]*nxt+path_set[ri][0][kth])
    if len(path_nid_set[ri]) > max_len:
      max_len = len(path_nid_set[ri])
  jpath = list()
  for kth in range(max_len):
    jv = list()
    for ri in range(nrobot): # loop over all robots to build a joint vertex
      if kth >= len(path_nid_set[ri]):
        jv.append(path_nid_set[ri][-1])
      else:
        jv.append(path_nid_set[ri][kth])
    jpath.append(jv)
  return jpath

def findTargetTimelineTPG(path_dict, path_dict_baseline, ac_dict, target_timeline, target_timeline_baseline):
     print("ac_dict: ", ac_dict)
    #  print("target_timeline[89]: ", target_timeline[89])
     timeline_baseline = copy.deepcopy(target_timeline_baseline)
     for tar in timeline_baseline:
          # get the location (x,y) of target from CBSS-D path
          # print("target_timeline[%d]: " % tar, target_timeline[tar])
          agent = target_timeline[tar][0]
          start = target_timeline[tar][1]
          x_tar = path_dict[agent][0][start]
          y_tar = path_dict[agent][1][start]
          
          # get target_timeline for CBSS-TPG path
          agent_baseline = timeline_baseline[tar][0]
          dur = ac_dict[tar][agent_baseline] - 1
          x_tar_dur = [x_tar for _ in range(dur)]
          y_tar_dur = [y_tar for _ in range(dur)]
          path_baseline = path_dict_baseline[agent_baseline]
          x_list = path_baseline[0]; y_list = path_baseline[1]
        #   print(x_list)
          for idx in range(len(x_list)):
               if x_list[idx] == x_tar and y_list[idx] == y_tar:
                    if x_list[idx:idx+dur]==x_tar_dur and y_list[idx:idx+dur]==y_tar_dur:
                        timeline_baseline[tar] = [agent_baseline, idx, idx + dur]
                        break
     return timeline_baseline

def SimulatePathSet(grids, path_set, goals, jstate=[], finals=[], folder="build/mcpfd_sim/", ac_dict=dict(), target_timeline=dict(), prefix=""):
  """
  Simulate a path_set, which is a path dictionary
  path_set[robot_id][0=x_list][1=y_list]
  """
  print("ac_dict:",ac_dict)
  # path_set = JointPathToPathSet(grids, jpath)
  # np.random.seed(2)
  colors = np.arange(len(path_set))
  # colors = np.array(['r','b']) # temp for toy example
  # print(path_set)
  # print(colors)

  interp_steps = 5

  # first_id = 0
  # if first_id not in path_set: # test result loaded from json has robot id as string
  #   first_id = str(0)
  
  temp_avec = set()
  max_len = 0 # max length over each agent's paths.
  for ri in path_set:
    if len(path_set[ri][0]) > max_len:
      max_len = len(path_set[ri][0])

  print("max_len = ", max_len)
  for step in range(max_len-1): # range(len(path_set[first_id][0])): # path_set[robot_id][0=x_list]
    for subSteps in range(interp_steps):

      ### the following line is for rbt_demo
      # fig = plt.figure(figsize=(3,2))

      # print("step = ", step)
      rxlist = list()
      rylist = list()
      next_rxlist = list()
      next_rylist = list()
      for k in path_set:
        # print("k = ", k)
        if k not in path_set: # test result loaded from json has robot id as string ??? unnecessary
          nstep = step+1
          cstep = step
          csubSteps = subSteps
          if nstep >= len(path_set[str(k)][0]):
            nstep = len(path_set[str(k)][0])-1
            cstep = nstep - 1
            csubSteps = interp_steps-1
            # print(" len(path_set[k][0]) = ", len(path_set[k][0]), " nstep = ", nstep)
          percent = (csubSteps*1.0/interp_steps) 
          pos_x = (1-percent) * path_set[str(k)][0][cstep] + percent * path_set[str(k)][0][nstep]
          pos_y = (1-percent) * path_set[str(k)][1][cstep] + percent * path_set[str(k)][1][nstep]
          rxlist.append(pos_x)
          rylist.append(pos_y)

          # rxlist.append(path_set[str(k)][0][cstep])
          # rylist.append(path_set[str(k)][1][cstep])
          # next_rxlist.append(path_set[str(k)][0][nstep])
          # next_rylist.append(path_set[str(k)][1][nstep])
        else:
          nstep = step+1
          cstep = step
          csubSteps = subSteps
          # print(" len(path_set[k][0]) = ", len(path_set[k][0]), " nstep = ", nstep)
          if nstep >= len(path_set[k][0]):
            nstep = len(path_set[(k)][0])-1
            cstep = nstep - 1
            csubSteps = interp_steps-1
            # print(" len(path_set[k][0]) = ", len(path_set[k][0]), " nstep = ", nstep)
          # print("nstep,",nstep,"len:",len(path_set[k][0]))
          percent = (csubSteps*1.0/interp_steps) 
          pos_x = (1-percent) * path_set[(k)][0][cstep] + percent * path_set[(k)][0][nstep]
          pos_y = (1-percent) * path_set[(k)][1][cstep] + percent * path_set[(k)][1][nstep]
          rxlist.append(pos_x)
          rylist.append(pos_y)

          # rxlist.append(path_set[k][0][cstep])
          # rylist.append(path_set[k][1][cstep])
          # next_rxlist.append(path_set[k][0][nstep])
          # next_rylist.append(path_set[k][1][nstep])

      # percent = (csubSteps*1.0/interp_steps) 
      # rxlist = (1-percent)*np.array(rxlist) + percent*np.array(next_rxlist)
      # rylist = (1-percent)*np.array(rylist) + percent*np.array(next_rylist)
      plt.clf()
      PlotGrids(grids)
      
      PlotPathSet(path_set, dict(), colors)

      # plt.scatter(rxlist,rylist, c=colors, cmap='viridis')
      
      # PlotPointSet(rxlist, rylist, colors, "o")

      # -- plot targets/goals
      gx_list = []
      gy_list = []
      gx_list2 = []
      gy_list2 = []
      ac_gx_list = []
      ac_gy_list = []
      ac_color_list = []
      nyt,nxt = grids.shape
      for idx in range(len(goals)):
        gy = int(np.floor(goals[idx]/nxt)) # goal x
        gx = int(goals[idx]%nxt) # goal y
        if len(jstate) > 0: # jstate (avec inside) is given!
          if jstate[step].avec[idx] == -1:
            gy_list.append(gy)
            gx_list.append(gx)
          else:
            gy_list2.append(gy)
            gx_list2.append(gx)
        else: # no jstate given
          for k in path_set:
            this_step = step
            if step >= len(path_set[k][0]):
              this_step = -1 # avoid overflow.
            # print(" ac_dict[goals[idx]] = ",  ac_dict[goals[idx]], " k = ", k, " type(k)=", type(k))
            if k not in path_set: # result dict use strings
              if (gx == path_set[str(k)][0][this_step]) and (gy == path_set[str(k)][1][this_step]):
                # temp_avec.add(goals[idx]) # visited !
                if goals[idx] in ac_dict and int(k) in ac_dict[goals[idx]]:
                  temp_avec.add(goals[idx]) # visited !
                elif goals[idx] not in ac_dict:
                  temp_avec.add(goals[idx]) # visited !
            else:
              # print(" this_step = ", this_step, " len = ", len(path_set[k][0]), len(path_set[k][1]))
              if (gx == path_set[k][0][this_step]) and (gy == path_set[k][1][this_step]):
                # temp_avec.add(goals[idx]) # visited !

                ### Plot target_timeline @2023-11-25 by Yuanhang Zhang
                # if goals[idx] in ac_dict and int(k) in ac_dict[goals[idx]]:
                #   temp_avec.add(goals[idx]) # visited !
                # elif goals[idx] not in ac_dict:
                #   print("??????????????????")
                #   temp_avec.add(goals[idx]) # visited !
                # For task with duration
                if goals[idx] in target_timeline and int(k) == target_timeline[goals[idx]][0]:
                  temp_avec.add(goals[idx]) # visited !

          if goals[idx] in temp_avec: # visited
            gy_list2.append(gy)
            gx_list2.append(gx)
          else:
            gy_list.append(gy)
            gx_list.append(gx)

          ### plot ac_dict @2021-07-01
          if len(ac_dict) > 0 and goals[idx] in ac_dict:
            #TODO:ac_dict needed to be further processed
            # print("yeah!!!!", len(ac_dict[goals[idx]]))
            if len(ac_dict[goals[idx]]) == 1:
              ac_gx_list.append(gx)
              ac_gy_list.append(gy)
              for temp in ac_dict[goals[idx]]: # there is only one value.
                ac_color_list.append(colors[temp])

      # plt.scatter(gx_list,gy_list, marker='s',color=(0.,0.,0.8)) # plot terminals with squares, not visited yet
      plt.scatter(gx_list,gy_list, marker='s',color=(0.,0.8,0.)) # plot terminals with squares, not visited yet
      
      ### handle ac_dict @2021-07-01
      # print("ac_list_x = ", ac_gx_list)
      # print("ac_list_y = ", ac_gy_list)
      # print("ac_color_list = ", ac_color_list)
      PlotPointSet(ac_gx_list, ac_gy_list,ac_color_list,"s")

      ### after handle ac_dict, plot visited goals @2021-07-01
      # plt.scatter(gx_list2,gy_list2, marker='s',color=(0.,0.8,0.)) # plot terminals with squares, already visited, diff color, (green)
      plt.scatter(gx_list2,gy_list2, marker='s',color=(1,1,1)) # plot terminals with squares, already visited, diff color, (grey)
      plt.scatter(gx_list2,gy_list2, marker='s',color=(0.5,0.5,0.5),facecolors='none') # plot terminals with squares, already visited, diff color, (grey)
      # print("gx_list = ", gx_list2, gy_list2)

      PlotPointSet(rxlist, rylist, colors, "o") # plot robot at last 

      # for idx in range(len(ac_gx_list)):
      #   plt.plot( ac_gx_list[idx],ac_gy_list[idx], marker='^',color=colors[ac_color_list[idx]] )

      # -- plot finals if given
      gx_list = []
      gy_list = []
      ac_gx_list = []
      ac_gy_list = []
      ac_color_list = []
      for idx in range(len(finals)):
        gy = int(np.floor(finals[idx]/nxt)) # final x
        gx = int(finals[idx]%nxt) # final y
        gy_list.append(gy)
        gx_list.append(gx)
        ### handle ac_dict @2021-07-01
        if len(ac_dict) > 0 and finals[idx] in ac_dict:
          if len(ac_dict[finals[idx]]) == 1:
            ac_gx_list.append(gx)
            ac_gy_list.append(gy)
            for temp in ac_dict[finals[idx]]: # there is only one value.
              ac_color_list.append(colors[temp])

      plt.scatter(gx_list,gy_list, marker='*',c=colors) # plot finals with triangles

      ### handle ac_dict @2021-07-01
      PlotPointSet(ac_gx_list, ac_gy_list,ac_color_list,"*")

      plt.draw()
      plt.pause(0.01)
      # print("before save")

      fig_index = step*interp_steps + subSteps
      if folder != "":
        if prefix == "":
          # plt.savefig(folder+"mcpfd_r"+str(len(path_set))+"_j"+str(len(goals))+"_"+format(fig_index, '05d')+".png")
          plt.savefig(folder+str(fig_index)+".png")
        else:
          plt.savefig(folder+prefix+"_r"+str(len(path_set))+"_j"+str(len(goals))+"_"+format(fig_index, '05d')+".png")
  return

# @jit(nopython=True)
def Equal(v1,v2):
  for idx in range(len(v1)):
    if abs(v1[idx] - v2[idx]) > 1e-5:
      return False
  return True

# @jit(nopython=True)
def DominantLess(v1,v2):
  """
  given two vector v1,v2 (list or tuple), return if v1 dominates v2.
  """
  exist_strict_less = False
  for idx in range(len(v1)):
    if v1[idx] > v2[idx]:
      return False # v1 does not dominate v2
    else:
      if v1[idx] < v2[idx]:
        exist_strict_less = True
  if exist_strict_less:
    return True
  else:
    return False

# @jit(nopython=True)
def StrongDominantLess(v1,v2):
  """
  given two vector v1,v2 (list or tuple), return if v1 strictly dominates v2.
  """
  for idx in range(len(v1)):
    if v1[idx] >= v2[idx]:
      return False # v1 does not dominate v2
  return True

def WeakDom(v1,v2, eps=0.0):
  """
  every element in v1 is <= the corresponding lement in v2.
  If eps > 0, then this is epsilon-dominance.
  """
  if np.sum( np.array(v1) <= (1.0+eps) * np.array(v2) ) == len(v1):
    return True
  return False
  
def UFFind(dic, i):
  """
  dic = Union-find data structure, find operation. Find root of i.
  if i is not in dic, ERROR!
  """
  if i not in dic:
    print("[ERROR] UFFind, i not in dic!!!")
    return "WTF??"
  while(dic[i] != i):
    dic[i] = dic[dic[i]] # path compression
    i = dic[i]
  return i

def UFUnion(dic, i, j):
  """
  dic = Union-find data structure, union operation, 
   union the set of i and the set of j.
  """
  rooti = UFFind(dic,i)
  rootj = UFFind(dic,j)
  if rooti == rootj:
    return
  else:
    maxij = max(rooti,rootj)
    if rooti == maxij:
      dic[rootj] = rooti
    else:
      dic[rooti] = rootj

def UFDict2ListSet(dic):
  """
  Convert a dict that represents a union find data structure to a list of (disjoint) sets.
  """
  out = list()
  aux = dict() # aux is a dic that maps set id to set index in the list.
  for k in dic:
    # print("k:",k)
    root = UFFind(dic,k)
    # print("root:",root)
    if root in aux:
      out[aux[root]].add(k)
    else:
      # print("root:",root)
      out.append(set())
      aux[root] = len(out)-1
      # print("aux[root]:",aux[root])
      out[aux[root]].add(k)
  return out,aux

def AssignmentBoolDom(avec1, avec2):
    """
    Check if one assignment dominates the other.
    """
    more_done = False
    for ia in range(len(avec1)):
      if avec1[ia] == -1: # a1 not visited
        if avec2[ia] >= 0: # a2 visited
          return False
      else: # a1 visited
        if avec2[ia] == -1: # a2 not visited
          more_done = True
    if more_done:
      return True
    return False

def AssignmentBoolEqual(avec1, avec2):
  """
  Convert t
  """
  return tuple(avec1 > -1) == tuple(avec2 > -1)

def IsColDictSubset(cdic1, cdic2):
  """
  Given two col dict of format {{1:2,2:2},{5:7,6:7,7:7}}, 
   check if cdic1 is "sub-dict" of cdict2. 
  Support rM*.
  """
  for k1 in cdic1:
    for k2 in cdic1:
      if k1 == k2:
        continue
      if cm.UFFind(cdic1,k1) != cm.UFFind(cdic1,k2):
        continue
      if (k1 not in cdic2) or (k2 not in cdic2):
        return False
      # now, k1,k2 in the same set in cdic1
      if cm.UFFind(cdic2,k1) != cm.UFFind(cdic2,k2):
        return False
  # for k in cdic1:
  #   if k not in cdic2:
  #     return False
  #   if cdic2[k] != cdic1[k]:
  #     return False
  return True

def UnionColDict(cdic1, cdic2):
  """
  union cdic1 into cdic2,
  assume using index set I={0,1,..,nr-1} to represent robots.
  """
  for k in cdic1:
    if k not in cdic2:
      cdic2[k] = cm.UFFind(cdic1,k)
    if cdic1[k] not in cdic2:
      cdic2[cdic1[k]] = cm.UFFind(cdic1,cdic1[k])
    cm.UFUnion(cdic2, k, cdic1[k])
  # notice that, cdic2 is modified in place !!
  return cdic2

class NpEncoder(json.JSONEncoder):
  def default(self, obj):
    if isinstance(obj, np.integer):
      return int(obj)
    elif isinstance(obj, np.floating):
      return float(obj)
    elif isinstance(obj, np.ndarray):
      return obj.tolist()
    else:
      return super(NpEncoder, self).default(obj)

if __name__ == "__main__":
  grids = np.zeros((10,10))
  Vo = [0,5,9]
  Vt = [30,35,55,59]
  Vd = [90,95,99]
  spMat = getTargetGraph(grids,Vo,Vt,Vd)
  print(spMat)
  print(np.max(spMat))

#
#
# Note: Functions below are for loading the dataset
def LoadMapDao(map_file):
  grids = np.zeros((2,2))
  with open(map_file,'r') as f:
    lines = f.readlines()
    lidx = 0
    nx = 0
    ny = 0
    for line in lines:
      if lidx == 1:
        a = line.split(" ")
        nx = int(a[1])
      if lidx == 2:
        a = line.split(" ")
        ny = int(a[1])
      if lidx == 4:
        grids = np.zeros((nx,ny))
      if lidx >= 4: # map data begin
        x = lidx - 4
        y = 0
        a = line.split("\n")
        # print(a[0])
        # print(len(str(a[0])))
        for ia in str(a[0]):
          # print(ia)
          if ia == "." or ia == "G":
            grids[x,y] = 0
          else:
            grids[x,y] = 1
          y = y+1
      lidx = lidx + 1
  return grids
#
#
#
def LoadBenchmarkDao(map_file, scen_file):
  # load grids
  grids = LoadMapDao(map_file)
  # np.set_printoptions(threshold=sys.maxsize)
  # print(grids)

  # load scene
  with open(scen_file,'r') as f:
    lines = f.readlines()
    case_idx = -1
    test_serie = dict() # all test cases
    test_case_dict = dict() 
    sx = list()
    sy = list()
    gx = list()
    gy = list()
    d_list = list()
    for line in lines:
      a = line.split("\t")
      if len(a) != 9: # a valid line, parse it
        continue
      if case_idx != int(a[0]): # new case
        if case_idx != -1:
          # save last case
          test_case_dict = dict()
          test_case_dict["sx"] = sx
          test_case_dict["sy"] = sy
          test_case_dict["gx"] = gx
          test_case_dict["gy"] = gy
          test_case_dict["d_list"] = d_list
          test_case_dict["grids"] = grids
          test_serie[case_idx] = test_case_dict
        case_idx = int(a[0])
        sx = list()
        sy = list()
        gx = list()
        gy = list()
        d_list = list()
      else: # same as last case
        sx.append(int(a[4]))
        sy.append(int(a[5]))
        gx.append(int(a[6]))
        gy.append(int(a[7]))
        d_list.append(1) # assume homo case
  return test_serie


def LoadTestCaseDao(map_file, scen_file):
  """
  load map and scen from given files and return a test_case dict.
  """
  # load grids
  grids = LoadMapDao(map_file)
  # np.set_printoptions(threshold=sys.maxsize)
  # print(grids)
  nyt, nxt = grids.shape

  # get free locations
  xfre,yfre = GetFreeXY(grids)
  xfree = np.array(xfre)
  yfree = np.array(yfre)
  idx_list = np.random.permutation(len(xfree))
  xy_set = set()
  for idx in idx_list:
    xy_set.add( tuple([xfree[idx], yfree[idx]]) )
  # print("xy_set:", len(xy_set))
  # load scene
  starts = list()
  finals = list()
  with open(scen_file,'r') as f:
    lines = f.readlines()
    # case_idx = -1
    test_serie = dict() # all test cases
    test_case_dict = dict() 
    sx = list()
    sy = list()
    gx = list()
    gy = list()
    d_list = list()
    for line in lines:
      a = line.split("\t")
      if len(a) != 9: # an invalid line, skip it
        continue
      else: # same as last case
        sx.append(int(a[4]))
        sy.append(int(a[5]))
        gx.append(int(a[6]))
        gy.append(int(a[7]))
        starts.append( sy[-1] * nxt + sx[-1])
        finals.append( gy[-1] * nxt + gx[-1])
        # remove locs that are start or goals of agents
        xy_set.remove( tuple( [int(a[4]), int(a[5])] ) )
        xy_set.remove( tuple( [int(a[6]), int(a[7])] ) )
      if len(finals) >= 100:
        break # limit the number of finals, max. 100
  
  # generate terminals
  goals = list()
  for xy in xy_set:
    goals.append( xy[1] * nxt + xy[0] )
    if len(goals) >= 500: # limit the number of finals, max. 500
      break

  # put together a dict
  test_case_dict = dict()
  test_case_dict["sx"] = sx
  test_case_dict["sy"] = sy
  test_case_dict["fx"] = gx
  test_case_dict["fy"] = gy
  test_case_dict["grids"] = grids
  test_case_dict["starts"] = starts
  test_case_dict["finals"] = finals
  test_case_dict["goals"] = goals

  # test_serie[case_idx] = test_case_dict
  print( " -- generated a test case with #finals = ", len(finals), " #sx = ", len(sx), " #goals = ", len(goals),
        " #starts = ", len(starts),)
  # print(goals)
  return test_case_dict

def LoadTestSerieDao(map_file, scen_files, test_serie_name):
  """
  generate a test_serie from scen_files.
  """
  test_serie = dict()
  test_serie["test_serie_name"] = test_serie_name
  kk = 0
  for sf in scen_files:
    test_serie[kk] = LoadTestCaseDao(map_file, sf)
    kk = kk+1
  return test_serie

def FromDaoToPickle(map_file, scen_files, test_serie_name, folder):

  grids = LoadMapDao(map_file)
  gridy, gridx = grids.shape
  print(" grids size x=",gridx, " y=", gridy)

  test_serie = LoadTestSerieDao(map_file, scen_files, test_serie_name)
  SaveTestSerie(test_serie, folder + test_serie_name+".pickle")
  return

def loadCaseResult(case_name):
  cur_path = os.getcwd()
  output_dir = os.path.join(cur_path, 'output/',case_name)
  case_path = os.path.join(output_dir, 'case.pickle')
  result_path = os.path.join(output_dir, 'result.pickle')
  case_dict = pickle.load(open(case_path,'rb'))
  result_dict = pickle.load(open(result_path,'rb'))
  return case_dict, result_dict

def comparePath(paths1, paths2):
  for ri in paths1.paths:
    print("ri: ", ri)
    for dim in range(len(paths1.paths[ri])):
      print("paths1.paths[ri][dim] is:", paths1.paths[ri][dim])
      print("paths2.paths[ri][dim] is:", paths2.paths[ri][dim])
      if paths1.paths[ri][dim] != paths2.paths[ri][dim]:
        return False
  return True

def savePath(path_set, save_name):
  with open(save_name, 'wb') as f:
    pickle.dump(path_set, f)

def mkdir(path):
    path=path.strip()
    path=path.rstrip("\\")
    isExists=os.path.exists(path)
    if not isExists:
        # use 'utf-8' to decode
        os.makedirs(path) 
        print(path+' built successfully')
        return True
    else:
        print(path+' exists!')
        return False

def analyzeConflicts(conflict_set):
  sm_conflict = list()
  conflicts = copy.deepcopy(conflict_set)
  for conflict in conflict_set:
    num = conflicts.count(conflict)
    while(conflicts.count(conflict)>0):
      conflicts.remove(conflict)
    if(num > 1): sm_conflict.append({'conflict':conflict, 'num': num})
  return sm_conflict

def extractResults(case_name, exp_name, N, M, dur):
  cur_path = os.getcwd()
  output_dir = os.path.join(cur_path, 'output/', exp_name, case_name)
  result_list = list()
  for i in range(25):
    subpath = 'N' + str(N) + 'M' + str(M) + 'D' + str(dur) + '/result' + str(i+1) + '.pickle'
    result_path =  os.path.join(output_dir, subpath)
    result_dict = pickle.load(open(result_path,'rb'))
    result = {'i': i+1,
              'cbss+stn': {'search_success':result_dict['cbxs_baseline_dict']['search_success'],
                           'n_roots': result_dict['cbxs_baseline_dict']['n_roots'],
                           'best_g_value': result_dict['cbxs_baseline_dict']['best_g_value'],
                           'search_time': result_dict['cbxs_baseline_dict']['search_time'],
                           'num_resolved_conflict':result_dict['cbxs_baseline_dict']['n_conf'],
                           'num_target_conflict':result_dict['cbxs_baseline_dict']['n_sp_conf'],
                           'num_low_level_search':result_dict['cbxs_baseline_dict']['num_low_level_expanded']},
              'cbxs_old': {'search_success':result_dict['cbxs_old_res_dict']['search_success'],
                           'n_roots': result_dict['cbxs_old_res_dict']['n_roots'],
                           'best_g_value': result_dict['cbxs_old_res_dict']['best_g_value'],
                           'search_time': result_dict['cbxs_old_res_dict']['search_time'],
                           'num_resolved_conflict':result_dict['cbxs_old_res_dict']['n_conf'],
                           'num_target_conflict':result_dict['cbxs_old_res_dict']['n_sp_conf'],
                           'num_low_level_search':result_dict['cbxs_old_res_dict']['num_low_level_expanded']},
              'cbxs': {'search_success':result_dict['cbxs_res_dict']['search_success'],
                           'n_roots': result_dict['cbxs_res_dict']['n_roots'],
                           'best_g_value': result_dict['cbxs_res_dict']['best_g_value'],
                           'search_time': result_dict['cbxs_res_dict']['search_time'],
                           'num_resolved_conflict':result_dict['cbxs_res_dict']['n_conf'],
                           'num_target_conflict':result_dict['cbxs_res_dict']['n_sp_conf'],
                           'num_low_level_search':result_dict['cbxs_res_dict']['num_low_level_expanded']}}
    result_list.append(result)
    # if result_dict['cbxs_res_dict']['search_success'] and result_dict['cbxs_baseline_dict']['search_success'] and result['cbss+stn']['best_g_value'] < result['cbxs']['best_g_value']:
    #   print("*******************")
    #   print("N: %d, M: %d, D: %d, i: %d" % (N,M,dur,i+1))
    #   print(result)
    #   print("*******************")
  return result_list

