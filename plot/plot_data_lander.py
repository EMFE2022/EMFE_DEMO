import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from data_process.data_process import load_data_universe


def opted_data_process(data_dict, data_path):

    v_norm = []
    u_norm = []

    for i in range(len(data_dict['time'])):
        v_norm_i = np.sqrt(data_dict['vx'][i]**2 + data_dict['vy'][i]**2 + data_dict['vz'][i]**2)
        u_norm_i = np.sqrt(data_dict['ux'][i]**2 + data_dict['uy'][i]**2 + data_dict['uz'][i]**2)

        v_norm.append(v_norm_i)
        u_norm.append(u_norm_i)

    data_dict_add = {
        'v_norm': v_norm,
        'u_norm': u_norm,
    }

    data_dict.update(data_dict_add)

    df = pd.DataFrame(data_dict)
    df.to_csv(data_path + '/data_ad.csv', index=0)
    return data_dict


class Plot_data_lander:
    def __init__(self, data_path, file_name, picture_category):
        self.data_path = data_path
        self.file_name = file_name
        self.picture_category = picture_category
        self.color = ['royalblue', 'g', 'gray']
        self.point_shape = ['o', "^", 'D', 'P', 'v', '+', 'd', 'x']
        self.cmap = plt.get_cmap("tab10")
        self.label_list = ['OPT_result']

    def plot_2D(self, plot_nfe_flag, data):

        if 'pos' in self.picture_category:
            figure1 = plt.figure('position',figsize = (16,9))
            fig1 = figure1.add_subplot(111)
            for i in range(len(data)):
                for k in range(len(data[i])):
                    fig1.plot(data[i][k].get('time'), data[i][k].get('rx'), self.point_shape[i]+'--', label='$r_{x}$', linewidth=3, markersize=10)
                    fig1.plot(data[i][k].get('time'), data[i][k].get('ry'), self.point_shape[i]+'--', label='$r_{y}$', linewidth=3, markersize=10)
                    fig1.plot(data[i][k].get('time'), data[i][k].get('rz'), self.point_shape[i]+'--', label='$r_{z}$', linewidth=3, markersize=10)
                    # if k != len(data[i]) - 1:
                    #     plt.vlines(data[i][k].get('time_total')[-1], data[i][0].get('alt')[0], data[i][len(data[i]) - 1].get('alt')[-1], colors="k", linewidth=2, linestyles="dashed")
                    if plot_nfe_flag:
                        fig1.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('rx'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig1.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('ry'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig1.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('rz'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
            fig1.legend(prop={'family': 'Times New Roman', 'size': 35}, loc='best')
            fig1.grid(linestyle='--')
            fig1.set_title('position(m)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('t(s)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})


        if 'vel' in self.picture_category:
            figure2 = plt.figure('velocity', figsize=(16, 9))
            fig2 = figure2.add_subplot(111)
            for i in range(len(data)):
                for k in range(len(data[i])):
                    fig2.plot(data[i][k].get('time'), data[i][k].get('vx'), self.point_shape[i]+'--', label='$V_{x}$', linewidth=3, markersize=10)
                    fig2.plot(data[i][k].get('time'), data[i][k].get('vy'), self.point_shape[i]+'--', label='$V_{y}$', linewidth=3, markersize=10)
                    fig2.plot(data[i][k].get('time'), data[i][k].get('vz'), self.point_shape[i]+'--', label='$V_{z}$', linewidth=3, markersize=10)
                    fig2.plot(data[i][k].get('time'), data[i][k].get('v_norm'), self.point_shape[i]+'-.', c='k',label='$\Vert V \Vert_{2}$', linewidth=3, markersize=10)
                    if plot_nfe_flag:
                        fig2.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('vx'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig2.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('vy'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig2.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('vz'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig2.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('v_norm'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)
            fig2.legend(prop={'family': 'Times New Roman', 'size': 35}, loc='best')
            fig2.grid(linestyle='--')
            fig2.set_title('$velocity(m/s)$', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('t(s)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})


        if 'thrust' in self.picture_category:
            figure3 = plt.figure('thrust', figsize=(16, 9))
            fig3 = figure3.add_subplot(111)
            for i in range(len(data)):
                if i == 1:
                    time_total_i = data[1][0].get('time').copy()
                    engine_thrust_i = data[1][0].get('thrust').copy()
                    time_total_i.insert(31, 32.5132991147768)
                    engine_thrust_i.insert(31, 3000)
                    time_total_i.insert(68, 106.204240190564)
                    engine_thrust_i.insert(68, 15000)
                if i == 2:
                    time_total_ii = data[2][0].get('time').copy()
                    engine_thrust_ii = data[2][0].get('thrust').copy()
                    time_total_ii.insert(31, 32.4463794008115)
                    engine_thrust_ii.insert(31, 3000)
                    time_total_ii.insert(68, 106.137320476599)
                    engine_thrust_ii.insert(68, 15000)
                for k in range(len(data[i])):
                    if i == 0:
                        fig3.plot([data[i][k].get('time')[0], data[i][k].get('time')[-1]], [15000] * 2, '--', label='$T_{max}$', color='r', linewidth=3, markersize=10)
                        fig3.plot([data[i][k].get('time')[0], data[i][k].get('time')[-1]], [15000*0.2] * 2, '--', label='$T_{min}$', color='r', linewidth=3, markersize=10)
                    if i == 0:
                        fig3.plot(data[i][k].get('time'), data[i][k].get('thrust'), self.point_shape[i]+'--', color=self.cmap(i), label='$'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                    elif i == 1:
                        fig3.plot(time_total_i, engine_thrust_i, self.point_shape[i]+'--', color=self.cmap(i), label='$'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                        fig3.plot(time_total_i[31], engine_thrust_i[31], marker='s', markerfacecolor='w', color='r', markeredgewidth=3.5, markersize=7.5)
                        fig3.plot(time_total_i[68], engine_thrust_i[68], marker='s', markerfacecolor='w', color='r', markeredgewidth=3.5, markersize=7.5)
                    else:
                        fig3.plot(time_total_ii, engine_thrust_ii, self.point_shape[i]+'--', color=self.cmap(i), label='$'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                        fig3.plot(time_total_ii[31], engine_thrust_ii[31], marker='s', markerfacecolor='w', color='r', markeredgewidth=3.5, markersize=7.5)
                        fig3.plot(time_total_ii[68], engine_thrust_ii[68], marker='s', markerfacecolor='w', color='r', markeredgewidth=3.5, markersize=7.5)
                    # fig3.plot(data[i][k].get('time_total'), data[i][k].get('mass_prop_booster'), self.point_shape[i]+'--', color=self.cmap(k+1), label='$^{' + self.index[k] + '}mass_{prop}^{booster}$', linewidth=3, markersize=10)
                    if plot_nfe_flag:
                        fig3.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)], np.array(data[i][k].get('thrust'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)],  marker='s', color='r', s=10**2, zorder=2.5)

            fig3.legend(prop={'family': 'Times New Roman', 'size': 35}, loc='best')
            fig3.grid(linestyle='--')
            fig3.set_title('thrust(N)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('t(s)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            fig3.set_title('F(N)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})


        if 'u' in self.picture_category:
            figure4 = plt.figure('u', figsize=(16, 9))
            fig4 = figure4.add_subplot(111)
            for i in range(len(data)):
                for k in range(len(data[i])):
                    fig4.plot(data[i][k].get('time')[1::], data[i][k].get('ux')[1::], self.point_shape[0]+'--', color=self.cmap(i), label='$u_{x}\;'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                    fig4.plot(data[i][k].get('time')[1::], data[i][k].get('uy')[1::], self.point_shape[1]+'--', color=self.cmap(i), label='$u_{y}\;'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                    fig4.plot(data[i][k].get('time')[1::], data[i][k].get('uz')[1::], self.point_shape[2]+'--', color=self.cmap(i), label='$u_{z}\;'+ self.label_list[i] + '$', linewidth=3, markersize=10)
                    fig4.plot(data[i][k].get('time')[1::], data[i][k].get('u_norm')[1::], self.point_shape[i]+'-.', c='k', label='$\Vert u \Vert_{2}$', linewidth=3, markersize=10)
                    if plot_nfe_flag:
                        fig4.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array(data[i][k].get('ux'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig4.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array(data[i][k].get('uy'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig4.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array(data[i][k].get('uz'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::],  marker='s', color='r', s=10**2, zorder=2.5)
                        fig4.scatter(np.array(data[i][k].get('time'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array(data[i][k].get('u_norm'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::],  marker='s', color='r', s=10**2, zorder=2.5)
            fig4.legend(prop={'family': 'Times New Roman', 'size': 25}, loc='best', ncol=3)
            fig4.grid(linestyle='--')
            fig4.set_title('$u$', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('t(s)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})


        if 'tf' in self.picture_category:
            figure23 = plt.figure('tf',figsize = (16,9))
            fig23 = figure23.add_subplot(111)
            bar_containter = []
            for i in range(len(data)):
                n_nfe_total = 0
                for k in range(len(data[i])):
                    bar_containter.append(None)
                    bar_containter[-1] = fig23.bar(np.linspace(n_nfe_total, n_nfe_total+data[i][k]['nfe'][-1], data[i][k]['nfe'][-1]+1, dtype=int)[1::], \
                              np.array(data[i][k].get('tf'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array([1]*data[i][k]['nfe'][-1]), \
                              edgecolor='k')
                    fig23.bar_label(bar_containter[-1], padding=3, fmt='%.02f', fontproperties='Times New Roman', size=20, fontweight='bold')
                    n_nfe_total += data[i][k]['nfe'][-1]
            fig23.legend(prop={'family': 'Times New Roman', 'size': 35}, loc='best')
            fig23.grid(linestyle='--')
            fig23.set_title('GFEL ' + chr(963), fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('finite element index', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            fig23.set_xticks(np.linspace(1, data[i][k].get('nfe')[0], data[i][k].get('nfe')[0], dtype=int))


        if 'noncollocation_error' in self.picture_category:
            figure24 = plt.figure('noncollocation error',figsize = (16,9))
            fig24 = figure24.add_subplot(111)
            bar_containter = []
            for i in range(len(data)):
                n_nfe_total = 0
                for k in range(len(data[i])):
                    bar_containter.append(None)
                    bar_containter[-1] = fig24.bar(np.linspace(n_nfe_total, n_nfe_total+data[i][k]['nfe'][-1], data[i][k]['nfe'][-1]+1, dtype=int)[1::], \
                              1e3*np.array(data[i][k].get('noncollocation_error'))[np.linspace(0, data[i][k]['nfe'][-1]*data[i][k]['ncp'][-1], data[i][k]['nfe'][-1]+1, dtype=int)][1::], np.array([1]*data[i][k]['nfe'][-1]), \
                              edgecolor='k')
                    fig24.bar_label(bar_containter[-1], padding=3, fmt='%.02f', fontproperties='Times New Roman', size=20, fontweight='bold')
                    n_nfe_total += data[i][k]['nfe'][-1]
            fig24.legend(prop={'family': 'Times New Roman', 'size': 35}, loc='best')
            fig24.grid(linestyle='--')
            fig24.set_title(r'NCPE estimation S ($\times10^{-3}$)', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            plt.yticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xticks(fontproperties='Times New Roman', size=35, fontweight='bold')
            plt.xlabel('finite element index', fontdict={'family': 'Times New Roman', 'size': 35, 'fontweight': 'bold'})
            fig24.set_xticks(np.linspace(1, data[i][k].get('nfe')[0], data[i][k].get('nfe')[0], dtype=int))


        plt.show()


    def run_plot(self, plot_nfe_flag=False):
        data_for_plot_total = []

        for i in range(len(self.data_path)):
            data_for_plot = []
            for k in range(len(self.file_name)):
                data = load_data_universe(self.data_path[i] + self.file_name[k])
                data = opted_data_process(data, self.data_path[i])
                data_for_plot.append(data)

            data_for_plot_total.append(data_for_plot)
        self.plot_2D(plot_nfe_flag, data_for_plot_total)
