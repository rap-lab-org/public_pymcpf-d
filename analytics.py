import context
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import random
import common as cm
import os
import pickle
import statistics, copy

### obtain current path
cur_path = os.getcwd()
exp = "exp/mcpfd"
case_name = "random-32-32-20"
# case_name = "maze-32-32-2"
map_name = case_name +".map"

# N_list = [5,10,20]
N_list = [5]
M_list = np.array([10,20,30,40,50])
dur_list = [2,5,10,20]
X_list = [M_list-1, M_list, M_list+1, M_list+2]

def plotSR(exp_name, N, num_X, success_rate_baseline, success_rate_1,dur_list,legend=False,save=False):
    fig, ax1 = plt.subplots()
    # print(num_X)
    custom_legend = []
    for i in range(len(num_X)): 
        # 绘制success rate曲线  
        ax1.plot(num_X[i], success_rate_baseline[i], '^--', color='C'+str(i), markersize=8)
        ax1.bar(num_X[i], success_rate_1[i], color='C'+str(i), alpha=0.7)
        custom_legend.append(mpatches.Patch(color='C'+str(i), label='D='+str(dur_list[i])))
        # break
        # ax1.plot(num_X[i], success_rate_1[i], 'o--', color='C2', label='Success Rate of cbxs')
        # 添加标签
        # ax1.set_title('Performance', fontsize=15)

    # ax1.set_xlabel('Number of Targets(M)')
    # ax1.set_ylabel('Success Rate', color='C0')
    ax1.tick_params(axis='both', labelcolor='black', which='major', labelsize=18)

    # 添加标签
    # ax1.set_title('N = %d, Success Rate' % N, fontsize=15)

    # 添加图例
    # 自定义图标和标签
    if legend: ax1.legend(handles = custom_legend,loc='upper left')

    # 细节设置
    fig.tight_layout()

    if save: plt.savefig(os.path.join(cur_path, "output/" + exp_name+"-"+case_name+"-N"+str(N)+"-SR.png"))

    plt.show()

def plotCR(exp_name, N, num_X, cost_ratios,dur_list,legend=False, save=False):
    fig, ax1 = plt.subplots()
    # print(num_X)
    # print(len(num_X[0]))
    # print(len(cost_ratios[0]))
    # print(len(cost_ratios[0][0]), len(cost_ratios[0][1]), len(cost_ratios[0][2]), len(cost_ratios[0][3]), len(cost_ratios[0][4]))

    # ax1.set_xlabel('Number of Targets(M)')
    ax1.tick_params(axis='y', labelcolor='black')
    custom_legend = []
    for i in range(len(num_X)):
        # print(cost_ratios[i])
        color = 'C' + str(i)
        meann_list = []
        for j in range(5):
            if len(cost_ratios[i][j])==0:
                meann_list.append(0)
                continue
            meann = statistics.mean(cost_ratios[i][j])
            meann_list.append(meann)
            minn = min(cost_ratios[i][j])
            maxx = max(cost_ratios[i][j])
            plt.plot((num_X[i][j],num_X[i][j]), (minn,maxx), marker='_', linestyle='-', color=color, markersize=10)  # 连接两个数据点
        ax1.plot(num_X[i], meann_list, 'x', color=color, markersize=8)
        custom_legend.append(mpatches.Patch(color='C'+str(i), label='D='+str(dur_list[i])))
    
    # 添加标签
    # ax1.set_title('N = %d, Cost Ratio' % N, fontsize=15)

    # 添加图例
    if legend: ax1.legend(handles = custom_legend,loc='upper left')
    ax1.tick_params(axis='both', labelcolor='black', which='major', labelsize=18)
    # 细节设置
    fig.tight_layout()

    if save: 
        print(os.path.join(cur_path, exp_name+"-CR.png"))
        plt.savefig(os.path.join(cur_path, exp_name+"-"+case_name+"-N"+str(N)+"-CR.png"))

    plt.show()

def analyticsN(n, plot):
    num_X = []
    success_rate_baseline = []
    success_rate_0 = []
    success_rate_1 = []
    cost_ratios = []
    conflict_ratios = []
    id = -1
    cost_0 = 0
    cost_1 = 0
    cost01 =  0
    num = 0; num_success=0
    for dur in dur_list:
        id += 1
        num_X.append([])
        success_rate_baseline.append([])
        success_rate_0.append([])
        success_rate_1.append([])
        cost_ratios.append([])
        idx = -1
        for m in M_list:
            idx += 1
            result_list = cm.extractResults(case_name, exp, n, m, dur)
            num_success_baseline = 0
            num_success_0 = 0
            num_success_1 = 0
            cost_ratio_list = []
            conflict_ratio_list = []
            for res in result_list:
                num+=1
                res_base = res['cbss+stn']
                res_0 = res['cbxs_old']
                res_1 = res['cbxs']
                # if res_0['num_resolved_conflict'] > 0 or res_1['num_resolved_conflict'] > 0:
                #     # print("n_roots: ", res_0['n_roots'])
                #     # print("num_resolved_conflict: ", res_0['num_resolved_conflict'])
                #     num0 = res_0['num_resolved_conflict']
                #     num1 = res_1['num_resolved_conflict']
                #     if num0 < num1:
                #         # print("###########")
                #         # print("N: %d, M: %d, dur: %d, i: %d" % (n, m, dur,res['i']))
                #         # print("num_target_conflict: ", res_0['num_target_conflict'])
                #         # print("num_resolved_conflict of old: ", res_0['num_resolved_conflict'])
                #         # print("num_resolved_conflict of new: ", res_1['num_resolved_conflict'])
                #         cost_0+=1
                #     elif num0 > num1: cost_1+=1
                #     else: cost01 += 1
                #     # if dur == 20:
                #     #     if num0 > num1: cost_0+=1
                #     #     if num0 < num1: cost_1+=1
                if res_base['search_success'] and res_1['search_success']:
                    num_success+=1
                    if res_0['num_resolved_conflict'] > 0 or res_1['num_resolved_conflict'] > 0:
                        # print("n_roots: ", res_0['n_roots'])
                        # print("num_resolved_conflict: ", res_0['num_resolved_conflict'])
                        num0 = res_0['num_resolved_conflict']
                        num1 = res_1['num_resolved_conflict']
                        if num0 < num1:
                            # print("###########")
                            # print("N: %d, M: %d, dur: %d, i: %d" % (n, m, dur,res['i']))
                            # print("num_target_conflict: ", res_0['num_target_conflict'])
                            # print("num_resolved_conflict of old: ", res_0['num_resolved_conflict'])
                            # print("num_resolved_conflict of new: ", res_1['num_resolved_conflict'])
                            cost_0+=1
                            conflict_ratios.append((num0-num1)/num0*100)
                        elif num0 > num1:
                            print("--------------------------")
                            print(num0, num1)
                            print("search_time of old: ", res_0['search_time'])
                            print("search_time of new: ", res_1['search_time'])
                            print("N: %d, M: %d, dur: %d, i: %d" % (n, m, dur,res['i']))
                            cost_1+=1
                            conflict_ratios.append((num0-num1)/num0*100)
                        else: 
                            # print(num0, num1)
                            cost01 += 1
                        # if dur == 20:
                        #     if num0 > num1: cost_0+=1
                        #     if num0 < num1: cost_1+=1
                    # cost_ratio = (res_base['best_g_value'] - res_1['best_g_value'])/res_1['best_g_value']*100
                    cost_ratio = (res_base['best_g_value'] - res_1['best_g_value'])/res_1['best_g_value']*100
                    cost_ratio_list.append(cost_ratio)
                    if cost_ratio < 0:
                        print("cost_0: ", res_base['best_g_value'], " cost_1: ", res_1['best_g_value'])
                        print("N: %d, M: %d, dur: %d, i: %d" % (n, m, dur,res['i']))
                    if cost_ratio > 15:
                        print("found!!!!!!!!!!!!!!!!!!!!!!")
                        print("N: %d, M: %d, dur: %d, i: %d" % (n, m, dur,res['i']))
                if res_base['search_success']: num_success_baseline += 1
                if res_0['search_success']: num_success_0 += 1
                if res_1['search_success']: num_success_1 += 1
            num_X[id].append(X_list[id][idx])
            cost_ratios[id].append(cost_ratio_list)
            success_rate_baseline[id].append(num_success_baseline/len(result_list))
            success_rate_0[id].append(num_success_0/len(result_list))
            success_rate_1[id].append(num_success_1/len(result_list))
    print("#### N = %d" % n)
    print("conflicts: old < new: ", cost_0)
    print("conflicts: old > new (expected): ", cost_1)
    print("conflicts: old = new: ", cost01)
    print("num is:", num)
    print("num_success is:", num_success)
    print("conflict_ratios is:", conflict_ratios)
    if len(conflict_ratios)==0:
        print("conflict_ratios is empty")
        minn = None
        meann = None
        maxx = None
    else:
        meann = statistics.mean(conflict_ratios)
        minn = min(conflict_ratios)
        maxx = max(conflict_ratios)
    print("conflict_ratio min/mean/max:", minn, meann, maxx)
    if plot:
        plotSR(exp, n, num_X, success_rate_baseline, success_rate_1, dur_list, False, True)
        plotCR(exp, n, num_X, cost_ratios, dur_list, False, True)

if __name__ == '__main__':
    print("begin of analytics")
    # print(X_list)
    print(exp)
    print(case_name)
    for n in N_list:
        analyticsN(n, True)



