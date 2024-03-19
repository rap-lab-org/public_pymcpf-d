"""
Author: Zhongqiang (Richard) Ren
All Rights Reserved.
ABOUT: this file contains CBXS framework (abstract).
Oeffentlich fuer: RSS22
"""

import networkx as ntx
import copy
import sys
import numpy as np

DEBUG_STN = False

def Sol2STN(path_dict, ac_dict, target_timeline):
	"""
	path_dict[agent][0=path_id_list, 1=time_list] = kth waypoint's vertex id or time step.
	ac_dict[vertex][agent] = duration.
	target_timeline[vertex] = a list of length three: [agent, task start time, task end time]

	"""
	dg = ntx.DiGraph()
	d_dict = dict()
	next_dict = dict()

	# print(" path_dict = ", path_dict)

	# type-1 edge
	for ri in path_dict:
		t0 = 0
		v0 = path_dict[ri][0][t0]
		node0 = (ri, v0, t0)
		if (v0 in target_timeline) and (ri == target_timeline[v0][0]) and (t0 == target_timeline[v0][1]):
			d_dict[node0] = ac_dict[v0][ri]
		else:
			d_dict[node0] = 1
		node = node0
		for idx in range(1,len(path_dict[ri][0])):
			v1 = path_dict[ri][0][idx]
			if v1 == node[1]: # wait-in-place action
				# print(" ri = ", ri, " idx = ", idx, " v1 = ", v1, " node = ", node, " d_dict : ", d_dict)
				d_dict[node] += 1
				continue
			node1 = (ri, v1, idx)
			dg.add_edge(node, node1)
			if (v1 in target_timeline) and (ri == target_timeline[v1][0]) and (idx == target_timeline[v1][1]):
				d_dict[node1] = ac_dict[v1][ri]
			else:
				d_dict[node1] = 1
			next_dict[node] = node1
			node = node1

	# type-2 edge
	for ri in path_dict:
		for idx in range(len(path_dict[ri][0])):
			ti = idx
			vi = path_dict[ri][0][ti]
			nodei = (ri, vi, ti)
			if not dg.has_node(nodei):
				continue
			for rj in path_dict:
				if ri == rj:
					continue
				for idy in range(idx+1,len(path_dict[rj][0])):
					tj = idy
					vj = path_dict[rj][0][tj]
					nodej = (rj, vj, tj)
					if vj == vi and dg.has_node(nodej):
						if DEBUG_STN: print("TYPE 2 Edge: ", (nodei, nodej))
						dg.add_edge(nodei, nodej)
						break
	
	# for n in dg:
	# 	print("[INFO] Sol2STN, stn has node : ", n)
	# for e in dg.edges():
	# 	print("[INFO] Sol2STN, stn has edge : ", e)
	# print("[INFO] Sol2STN, d_dict : ", d_dict)

	return (dg, d_dict, next_dict)


def CheckForDelete(dg, nn, d_dict, next_dict, closed):
	"""
	Determine if a stn node can be deleted.
	It checks that 
	(1) the agent has d <= 0, which means the agent does not need to stay at its current vertex.
	(2) the next vertex of the agent is reachable without collision with any other agent.
	  Here, (2) involves checking for a circular case, which is allowed (i.e., collision-free) 
	  based on the conflict definition. (using the set closed for implementation.)
	"""
	# print("CheckForDelete, input nn = ", nn)
	if (d_dict[nn] > 0):
		# print("a false")
		return False
	if (nn not in next_dict): # destination
		# print("b true")
		return True
	if (nn in next_dict and dg.in_degree(next_dict[nn]) == 1):
		# print("c true")
		return True
	else:
		# in_degree of next_dict[nn] is not 1
		if nn in closed:
			# print("d true")
			return True # if a few agents form a circle, then they are allowed to rotate.
		closed.add(nn)
		preds = dg.predecessors(next_dict[nn]) # there should be only 2 predecessors.
		all_pred_flag = True
		for p in preds:
			# print("get pred", p)
			if p == nn:
				# print("same as n, continue")
				continue
			# print("recursive check")
			if not CheckForDelete(dg, p, d_dict, next_dict, closed):
				all_pred_flag = False
		if all_pred_flag:
			# print("f true")
			return True
	# print("e false")
	return False

def ExtractPaths(dg, d_dict, starts, next_dict):
	"""
	"""

	# for ri in range(len(starts)):
	# 	n = (ri, starts[ri], 0)
	# 	if n not in d_dict:
	# 		print("[ERROR] node not in STN.")
	# 		sys.exit("[ERROR]")
	# 	d_dict[n] -= 1

	out_paths = {i: [[], []] for i in range(len(starts))}

	curr_vertices = copy.deepcopy(starts)
	n_iter = 0
	while dg.number_of_nodes() > 0:
		nodes_zero_deg = []
		for n in dg:
			if dg.in_degree(n) == 0:
				nodes_zero_deg.append(n)
		if DEBUG_STN: 
			print("###### interation: ", n_iter)
			print(" nodes_zero_deg = ", nodes_zero_deg)
		to_be_delete = []
		for nn in nodes_zero_deg:
			# update robot locations
			rk = nn[0]
			vk = nn[1]
			tk = nn[2]
			curr_vertices[rk] = vk
			# decrease d-value
			if nn not in d_dict:
				print("[ERROR] node not in STN.")
				sys.exit("[ERROR]")
			if DEBUG_STN: print("nn is: ", nn, "\td_dict[nn] is: ", d_dict[nn])
			d_dict[nn] -= 1

		for nn in nodes_zero_deg:
			closed = set()
			if CheckForDelete(dg, nn, d_dict, next_dict, closed):
				to_be_delete.append(nn)

		if DEBUG_STN: 
			print(" curr_vertices = ", curr_vertices)
			print(" to_be_delete = ", to_be_delete)
		for nk in to_be_delete:
			dg.remove_node(nk)
		for ri in range(len(starts)):
			out_paths[ri][1].append(n_iter)
			out_paths[ri][0].append(curr_vertices[ri])
		n_iter += 1

	for ri in range(len(starts)):
		out_paths[ri][0].append(out_paths[ri][0][-1])
		out_paths[ri][1].append(np.inf) # follow the same convention as the old CBSS.

	# print("out_paths:", out_paths)
	return out_paths





