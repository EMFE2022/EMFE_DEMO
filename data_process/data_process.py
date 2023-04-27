import numpy as np
import pandas as pd
from scipy import interpolate
from pyomo.environ import *

def interpolate_datac(x, y, x_new):

    if type(x_new[0]) == type((1,1)):
        index = x.index(1)
        x = [0] + x[0:index+1]
        y_x = [0] + y[0:index+1]
        y_y = [0] + y[index+1:2*index+2]
        y_z = [0] + y[2*index+2::]

        # print([len(y), len(y_x), len(y_y), len(y_z)])

        tau_index = x_new.index((0,1))

        x_new_ed = []
        for i in range(tau_index+1):
            x_new_ed.append(x_new[i][1])

        f = interpolate.interp1d(np.array(x), np.array(y_x), kind='nearest')
        y_x_new = f(np.array(x_new_ed))
        f = interpolate.interp1d(np.array(x), np.array(y_y), kind='nearest')
        y_y_new = f(np.array(x_new_ed))
        f = interpolate.interp1d(np.array(x), np.array(y_z), kind='nearest')
        y_z_new = f(np.array(x_new_ed))
        y_new = list(y_x_new) + list(y_y_new) + list(y_z_new)

    else:
        x = [0] + x
        y = [0] + y
        f = interpolate.interp1d(np.array(x), np.array(y), kind='nearest')
        y_new = list(f(np.array(x_new)))
    return y_new


def interpolate_datav(x, y, x_new):

    if type(x_new[0]) == type((1,1)):
        index = x.index(1)
        x = x[0:index+1]
        y_x = y[0:index+1]
        y_y = y[index+1:2*index+2]
        y_z = y[2*index+2::]

        # print([len(y), len(y_x), len(y_y), len(y_z)])

        tau_index = x_new.index((0,1))

        x_new_ed = []
        for i in range(tau_index+1):
            x_new_ed.append(x_new[i][1])

        f = interpolate.interp1d(np.array(x), np.array(y_x), kind='nearest')
        y_x_new = f(np.array(x_new_ed))
        f = interpolate.interp1d(np.array(x), np.array(y_y), kind='nearest')
        y_y_new = f(np.array(x_new_ed))
        f = interpolate.interp1d(np.array(x), np.array(y_z), kind='nearest')
        y_z_new = f(np.array(x_new_ed))
        y_new = list(y_x_new) + list(y_y_new) + list(y_z_new)

    else:
        x = x
        y = y
        f = interpolate.interp1d(np.array(x), np.array(y), kind='nearest')
        y_new = list(f(np.array(x_new)))
    return y_new


def save_dual_suffix(m, data_path, file_name):
    with open(data_path+file_name, 'w+') as f:
        f.write('Duals\n')

        for c in m.component_objects(Constraint, active=True):
            f.write('Constraint ' + str(c) + '\n')
            for index in sorted(c):
                if c[index].active == True:
                    f.write(str(index)+ ' ' + str(m.dual[c[index]]) + '\n')

        f.close()


def save_ipopt_zL_out_suffix(m, data_path, file_name):
    with open(data_path+file_name, 'w+') as f:
        f.write('ipopt_zL_out\n')

        for v in m.component_objects(Var, active=True):
            f.write('Variable ' + str(v) + '\n')
            try:
                for index in sorted(v):
                    if v[index].active == True:
                        f.write(str(index) + ' ' + str(m.ipopt_zL_out[v[index]]) + '\n')
            except:
                pass
        f.close()


def save_ipopt_zU_out_suffix(m, data_path, file_name):
    with open(data_path+file_name, 'w+') as f:
        f.write('ipopt_zU_out\n')

        for v in m.component_objects(Var, active=True):
            f.write('Variable ' + str(v) + '\n')
            try:
                for index in sorted(v):
                    if v[index].active == True:
                        f.write(str(index) + ' ' + str(m.ipopt_zU_out[v[index]]) + '\n')
            except:
                pass
        f.close()


def load_dual_suffix(m, data_path, file_name):
    with open(data_path+file_name, 'r+') as f:
        lines = f.readlines()
        for c in m.component_objects(Constraint, active=True):
            tau_c = sorted(c)
            if 'Constraint '+ str(c)+'\n' in lines:
                index_list = []
                values_list = []
                tau_last_list = []
                values_new_list = []
                cons_index = lines.index('Constraint '+ str(c)+'\n')
                if cons_index < (len(lines) - 1):
                    iteration_index = cons_index + 1
                    while lines[iteration_index][0] != 'C':
                        line = lines[iteration_index]
                        if line[0] == '(':
                            seg_index_I = line.find(', ')
                            seg_index_II = line.find(') ')
                            index = (float(line[1:seg_index_I]), float(line[seg_index_I+2:seg_index_II]))
                            values = float(line[seg_index_II + 2 : -1])
                            index_list.append(index)
                            values_list.append(values)
                            tau_last_list.append(float(line[seg_index_I+2:seg_index_II]))
                        else:
                            seg_index = line.find(' ')
                            if line[0:seg_index] == 'None':
                                index = None
                            else:
                                index = float(line[0:seg_index])
                            values = float(line[seg_index + 1:-1])
                            index_list.append(index)
                            values_list.append(values)
                        iteration_index += 1
                        if iteration_index > (len(lines) - 1):
                            break
                if tau_last_list != []:
                    values_new_list = interpolate_datac(tau_last_list, values_list, tau_c)
                elif values_list != []:
                    if max(tau_c) != None:
                        if max(tau_c) > 1:
                            if len(tau_c) > len(index_list):
                                values_new_list = values_list + [1e-10]*(len(tau_c) - len(index_list))
                            else:
                                values_new_list = values_list
                        else:
                            if index_list[0] == None:
                                values_new_list = values_list * len(tau_c)
                            else:
                                values_new_list = interpolate_datac(index_list, values_list, tau_c)
                    else:
                        values_new_list = values_list

                if values_new_list != []:
                    i = 0
                    for index in tau_c:
                        m.dual[c[index]] = values_new_list[i]
                        i += 1

            # except:
            #     pass


def load_ipopt_zL_out_suffix(m, data_path, file_name):
    with open(data_path+file_name, 'r+') as f:
        lines = f.readlines()
        for v in m.component_objects(Var, active=True):
            tau_v = sorted(v)
            # print(str(v))
            if 'Variable '+ str(v)+'\n' in lines:
                index_list = []
                values_list = []
                tau_last_list = []
                values_new_list = []
                cons_index = lines.index('Variable '+ str(v)+'\n')
                if cons_index < (len(lines) - 1):
                    iteration_index = cons_index + 1
                    while lines[iteration_index][0] != 'V':
                        line = lines[iteration_index]
                        if line[0] == '(':
                            seg_index_I = line.find(', ')
                            seg_index_II = line.find(') ')
                            index = (float(line[1:seg_index_I]), float(line[seg_index_I+2:seg_index_II]))
                            values = float(line[seg_index_II + 2 : -1])
                            index_list.append(index)
                            values_list.append(values)
                            tau_last_list.append(float(line[seg_index_I+2:seg_index_II]))
                        else:
                            seg_index = line.find(' ')
                            if line[0:seg_index] == 'None':
                                index = None
                            else:
                                index = float(line[0:seg_index])
                            values = float(line[seg_index + 1:-1])
                            index_list.append(index)
                            values_list.append(values)
                        iteration_index += 1
                        if iteration_index > (len(lines) - 1):
                            break
                if tau_last_list != []:
                    values_new_list = interpolate_datav(tau_last_list, values_list, tau_v)
                elif values_list != []:
                    if max(tau_v) != None:
                        if max(tau_v) > 1:
                            if len(tau_v) > len(index_list):
                                values_new_list = values_list + [1e-10]*(len(tau_v) - len(index_list))
                            else:
                                values_new_list = values_list
                        else:
                            if index_list[0] == None:
                                values_new_list = values_list * len(tau_v)
                            else:
                                values_new_list = interpolate_datav(index_list, values_list, tau_v)
                    else:
                        values_new_list = values_list

                if values_new_list != []:
                    i = 0
                    for index in tau_v:
                        m.ipopt_zL_out[v[index]] = values_new_list[i]
                        i += 1
            # except:
            #     pass


def load_ipopt_zU_out_suffix(m, data_path, file_name):
    with open(data_path + file_name, 'r+') as f:
        lines = f.readlines()
        for v in m.component_objects(Var, active=True):
            tau_v = sorted(v)
            if 'Variable '+ str(v)+'\n' in lines:
                index_list = []
                values_list = []
                tau_last_list = []
                values_new_list= []
                cons_index = lines.index('Variable ' + str(v) + '\n')
                if cons_index < (len(lines) - 1):
                    iteration_index = cons_index + 1
                    while lines[iteration_index][0] != 'V':
                        line = lines[iteration_index]
                        if line[0] == '(':
                            seg_index_I = line.find(', ')
                            seg_index_II = line.find(') ')
                            index = (float(line[1:seg_index_I]), float(line[seg_index_I+2:seg_index_II]))
                            values = float(line[seg_index_II + 2 : -1])
                            index_list.append(index)
                            values_list.append(values)
                            tau_last_list.append(float(line[seg_index_I+2:seg_index_II]))
                        else:
                            seg_index = line.find(' ')
                            if line[0:seg_index] == 'None':
                                index = None
                            else:
                                index = float(line[0:seg_index])
                            values = float(line[seg_index + 1:-1])
                            index_list.append(index)
                            values_list.append(values)
                        iteration_index += 1
                        if iteration_index > (len(lines) - 1):
                            break
                if tau_last_list != []:
                    values_new_list = interpolate_datav(tau_last_list, values_list, tau_v)
                elif values_list != []:
                    if max(tau_v) != None:
                        if max(tau_v) > 1:
                            if len(tau_v) > len(index_list):
                                values_new_list = values_list + [1e-10]*(len(tau_v) - len(index_list))
                            else:
                                values_new_list = values_list
                        else:
                            if index_list[0] == None:
                                values_new_list = values_list * len(tau_v)
                            else:
                                values_new_list = interpolate_datav(index_list, values_list, tau_v)
                    else:
                        values_new_list = values_list

                if values_new_list != []:
                    i = 0
                    for index in tau_v:
                        m.ipopt_zU_out[v[index]] = values_new_list[i]
                        i += 1
            # except:
            #     pass


        # num_lines = len(lines)
        # for i in range(num_lines):
        #     line = lines[i]
        #     seg_index = line.find(' ')
        #     index = float(line[0:seg_index])
        #     values = float(line[seg_index+1:-1])
        #         print(index)
        #         print(values)


def load_data_universe(data_path):
    raw_data = pd.read_csv(data_path)
    columns = raw_data.columns

    data = {}

    for c in columns:
        data_k = {
            c: list(raw_data.get(c)),
        }
        data.update(data_k)
    return data

def save_data(data_path, data_dict):
    df = pd.DataFrame(data_dict)
    df.to_csv(data_path, index=0)

def load_MFE_data(data_path):
    raw_data = pd.read_csv(data_path,usecols=['tf', 'noncollocation_error'])  # rows?  skiprows = 10

    initial_MFE_data = {
        'tf': list(raw_data.tf),
        'noncollocation_error': list(raw_data.noncollocation_error),
    }
    return initial_MFE_data


def create_initial_data_for_Lander(data_path, nfe, ncp, tau):

    def interpolate_data(x, y, x_new):
        f = interpolate.interp1d(x, y, kind='cubic')
        y_new = f(x_new)
        return y_new

    initial_data = load_data_universe(data_path)
    tf = initial_data['time'][-1]


    time_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('time'),[x*tf for x in sorted(tau)])]).T

    rx_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rx'),[x*tf for x in sorted(tau)])]).T
    ry_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ry'),[x*tf for x in sorted(tau)])]).T
    rz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rz'),[x*tf for x in sorted(tau)])]).T
    vx_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vx'),[x*tf for x in sorted(tau)])]).T
    vy_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vy'),[x*tf for x in sorted(tau)])]).T
    vz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vz'),[x*tf for x in sorted(tau)])]).T
    ux_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ux'),[x*tf for x in sorted(tau)])]).T
    uy_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('uy'),[x*tf for x in sorted(tau)])]).T
    uz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('uz'),[x*tf for x in sorted(tau)])]).T
    thrust_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('thrust'),[x*tf for x in sorted(tau)])]).T
    mass_prop_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('mass_prop'),[x*tf for x in sorted(tau)])]).T


    time_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('time_dt'),[x*tf for x in sorted(tau)])]).T
    rx_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rx_dt'),[x*tf for x in sorted(tau)])]).T
    ry_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ry_dt'),[x*tf for x in sorted(tau)])]).T
    rz_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rz_dt'),[x*tf for x in sorted(tau)])]).T
    vx_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vx_dt'),[x*tf for x in sorted(tau)])]).T
    vy_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vy_dt'),[x*tf for x in sorted(tau)])]).T
    vz_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vz_dt'),[x*tf for x in sorted(tau)])]).T
    mass_prop_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('mass_prop_dt'),[x*tf for x in sorted(tau)])]).T

    initial_data_matched = {
        'time_init': time_init,
        'rx_init': rx_init,
        'ry_init': ry_init,
        'rz_init': rz_init,
        'vx_init': vx_init,
        'vy_init': vy_init,
        'vz_init': vz_init,
        'ux_init': ux_init,
        'uy_init': uy_init,
        'uz_init': uz_init,
        'thrust_init': thrust_init,
        'mass_prop_init': mass_prop_init,
        'time_dt_init': time_dt_init,
        'rx_dt_init': rx_dt_init,
        'ry_dt_init': ry_dt_init,
        'rz_dt_init': rz_dt_init,
        'vx_dt_init': vx_dt_init,
        'vy_dt_init': vy_dt_init,
        'vz_dt_init': vz_dt_init,
        'mass_prop_dt_init': mass_prop_dt_init,
    }
    return initial_data_matched


def create_MFE_initial_data_for_Lander(data_path, nfe, ncp, tau, load_MFE_data_flag):

    def interpolate_data(x, y, x_new):
        f = interpolate.interp1d(x, y, kind='cubic')
        y_new = f(x_new)
        return y_new

    initial_data = load_data_universe(data_path)
    tf = initial_data['time'][-1]


    if load_MFE_data_flag:
        tau = []
        for i in range(len(initial_data.get('time'))):
            tau += [initial_data.get('time')[i]/tf]


    time_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('time'),[x*tf for x in sorted(tau)])]).T

    rx_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rx'),[x*tf for x in sorted(tau)])]).T
    ry_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ry'),[x*tf for x in sorted(tau)])]).T
    rz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rz'),[x*tf for x in sorted(tau)])]).T
    vx_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vx'),[x*tf for x in sorted(tau)])]).T
    vy_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vy'),[x*tf for x in sorted(tau)])]).T
    vz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vz'),[x*tf for x in sorted(tau)])]).T
    ux_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ux'),[x*tf for x in sorted(tau)])]).T
    uy_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('uy'),[x*tf for x in sorted(tau)])]).T
    uz_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('uz'),[x*tf for x in sorted(tau)])]).T
    thrust_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('thrust'),[x*tf for x in sorted(tau)])]).T
    mass_prop_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('mass_prop'),[x*tf for x in sorted(tau)])]).T


    time_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('time_dt'),[x*tf for x in sorted(tau)])]).T
    rx_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rx_dt'),[x*tf for x in sorted(tau)])]).T
    ry_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('ry_dt'),[x*tf for x in sorted(tau)])]).T
    rz_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('rz_dt'),[x*tf for x in sorted(tau)])]).T
    vx_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vx_dt'),[x*tf for x in sorted(tau)])]).T
    vy_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vy_dt'),[x*tf for x in sorted(tau)])]).T
    vz_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('vz_dt'),[x*tf for x in sorted(tau)])]).T
    mass_prop_dt_init = np.array([interpolate_data(initial_data.get('time'), initial_data.get('mass_prop_dt'),[x*tf for x in sorted(tau)])]).T

    initial_data_matched = {
        'time_init': time_init,
        'rx_init': rx_init,
        'ry_init': ry_init,
        'rz_init': rz_init,
        'vx_init': vx_init,
        'vy_init': vy_init,
        'vz_init': vz_init,
        'ux_init': ux_init,
        'uy_init': uy_init,
        'uz_init': uz_init,
        'thrust_init': thrust_init,
        'mass_prop_init': mass_prop_init,
        'time_dt_init': time_dt_init,
        'rx_dt_init': rx_dt_init,
        'ry_dt_init': ry_dt_init,
        'rz_dt_init': rz_dt_init,
        'vx_dt_init': vx_dt_init,
        'vy_dt_init': vy_dt_init,
        'vz_dt_init': vz_dt_init,
        'mass_prop_dt_init': mass_prop_dt_init,
    }

    if load_MFE_data_flag:
        initial_MFE_data = load_MFE_data(data_path)
        max_error_init = max(initial_MFE_data.get('noncollocation_error'))

        initial_MFE_data_matched = {
            'tf_MFE_init': np.array(initial_MFE_data.get('tf')).reshape((len(initial_MFE_data.get('tf')),1)),
            'noncollocation_error_init': np.array(initial_MFE_data.get('noncollocation_error')).reshape((len(initial_MFE_data.get('noncollocation_error')),1)),
            'max_error_init': np.array([max_error_init] * len(initial_MFE_data.get('noncollocation_error'))).reshape((len(initial_MFE_data.get('noncollocation_error')), 1))
        }

        initial_data_matched.update(initial_MFE_data_matched)

    return initial_data_matched


def log_data_for_Lander(m, nfe, ncp, data_path):
    rx = []
    ry = []
    rz = []
    vx = []
    vy = []
    vz = []
    ux = []
    uy = []
    uz = []
    thrust = []
    mass_prop = []
    time = []
    tf = []
    nfe_list = []
    ncp_list = []

    rx_dt = [0]
    ry_dt = [0]
    rz_dt = [0]
    vx_dt = [0]
    vy_dt = [0]
    vz_dt = [0]
    mass_prop_dt = [0]
    time_dt = [0]
    for i in sorted(m.tau):
        time.append(value(m.time[i]))
        rx.append(value(m.rx[i]))
        ry.append(value(m.ry[i]))
        rz.append(value(m.rz[i]))
        vx.append(value(m.vx[i]))
        vy.append(value(m.vy[i]))
        vz.append(value(m.vz[i]))
        ux.append(value(m.ux[i]))
        uy.append(value(m.uy[i]))
        uz.append(value(m.uz[i]))
        thrust.append(value(m.thrust[i]))
        mass_prop.append(value(m.mass_prop[i]))
        tf.append(value(m.tf))
        nfe_list.append(nfe)
        ncp_list.append(ncp)
        if i == 0:
            pass
        else:
            time_dt.append(value(m.time_dt[i]))
            rx_dt.append(value(m.rx_dt[i]))
            ry_dt.append(value(m.ry_dt[i]))
            rz_dt.append(value(m.rz_dt[i]))
            vx_dt.append(value(m.vx_dt[i]))
            vy_dt.append(value(m.vy_dt[i]))
            vz_dt.append(value(m.vz_dt[i]))
            mass_prop_dt.append(value(m.mass_prop_dt[i]))
    time_dt[0] = time_dt[1]
    rx_dt[0] = rx_dt[1]
    ry_dt[0] = ry_dt[1]
    rz_dt[0] = rz_dt[1]
    vx_dt[0] = vx_dt[1]
    vy_dt[0] = vy_dt[1]
    vz_dt[0] = vz_dt[1]
    mass_prop_dt[0] = mass_prop_dt[1]

    df = pd.DataFrame({
                    'time': time,
                    'rx': rx,
                    'ry': ry,
                    'rz': rz,
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'ux': ux,
                    'uy': uy,
                    'uz': uz,
                    'thrust': thrust,
                    'mass_prop': mass_prop,
                    'time_dt': time_dt,
                    'rx_dt': rx_dt,
                    'ry_dt': ry_dt,
                    'rz_dt': rz_dt,
                    'vx_dt': vx_dt,
                    'vy_dt': vy_dt,
                    'vz_dt': vz_dt,
                    'mass_prop_dt': mass_prop_dt,
                    'tf': tf,
                    'nfe': nfe_list,
                    'ncp': ncp_list,
                       }
                      )
    df.to_csv(data_path + '/data.csv', index=0)


def log_MFE_data_for_Lander(m, nfe, ncp, data_path):
    rx = []
    ry = []
    rz = []
    vx = []
    vy = []
    vz = []
    ux = []
    uy = []
    uz = []
    thrust = []
    mass_prop = []

    time = []
    tf = []
    nfe_list = []
    ncp_list = []
    noncollocation_error = []

    rx_dt = [0]
    ry_dt = [0]
    rz_dt = [0]
    vx_dt = [0]
    vy_dt = [0]
    vz_dt = [0]
    mass_prop_dt = [0]
    time_dt = [0]
    for i in sorted(m.tau):
        time.append(value(m.time[i]))
        rx.append(value(m.rx[i]))
        ry.append(value(m.ry[i]))
        rz.append(value(m.rz[i]))
        vx.append(value(m.vx[i]))
        vy.append(value(m.vy[i]))
        vz.append(value(m.vz[i]))
        ux.append(value(m.ux[i]))
        uy.append(value(m.uy[i]))
        uz.append(value(m.uz[i]))
        thrust.append(value(m.thrust[i]))
        mass_prop.append(value(m.mass_prop[i]))
        nfe_list.append(nfe)
        ncp_list.append(ncp)
        if i == 0:
            pass
        else:
            time_dt.append(value(m.time_dt[i]))
            rx_dt.append(value(m.rx_dt[i]))
            ry_dt.append(value(m.ry_dt[i]))
            rz_dt.append(value(m.rz_dt[i]))
            vx_dt.append(value(m.vx_dt[i]))
            vy_dt.append(value(m.vy_dt[i]))
            vz_dt.append(value(m.vz_dt[i]))
            mass_prop_dt.append(value(m.mass_prop_dt[i]))

        if i == 0:
            tf.append(0)
            noncollocation_error.append(0)
        else:
            tf.append(value(m.tf[int((sorted(m.tau).index(i) - 1) / 3)]))
            noncollocation_error.append(value(m.noncollocation_error[int((sorted(m.tau).index(i) - 1) / 3)]))
    time_dt[0] = time_dt[1]
    rx_dt[0] = rx_dt[1]
    ry_dt[0] = ry_dt[1]
    rz_dt[0] = rz_dt[1]
    vx_dt[0] = vx_dt[1]
    vy_dt[0] = vy_dt[1]
    vz_dt[0] = vz_dt[1]
    mass_prop_dt[0] = mass_prop_dt[1]
    time[0] = 0

    df = pd.DataFrame({
                    'time': time,
                    'rx': rx,
                    'ry': ry,
                    'rz': rz,
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'ux': ux,
                    'uy': uy,
                    'uz': uz,
                    'thrust': thrust,
                    'mass_prop': mass_prop,
                    'time_dt': time_dt,
                    'rx_dt': rx_dt,
                    'ry_dt': ry_dt,
                    'rz_dt': rz_dt,
                    'vx_dt': vx_dt,
                    'vy_dt': vy_dt,
                    'vz_dt': vz_dt,
                    'mass_prop_dt': mass_prop_dt,
                    'tf': tf,
                    'noncollocation_error': noncollocation_error,
                    'nfe': nfe_list,
                    'ncp': ncp_list,
                       }
                      )
    df.to_csv(data_path + '/data.csv', index=0)

