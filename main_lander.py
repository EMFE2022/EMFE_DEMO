import os
import time
import shutil
from pyomo.environ import *
from pyomo.common.tempfiles import TempfileManager
from model.Lunar_lander_3DOF_model import Lunar_Lander_3dof_model
from model.Lunar_lander_3DOF_MFE_model import Lunar_Lander_3dof_EMFE_model
from data_process.data_process import save_dual_suffix, load_dual_suffix,save_ipopt_zL_out_suffix, load_ipopt_zL_out_suffix, save_ipopt_zU_out_suffix, load_ipopt_zU_out_suffix, log_data_for_Lander, log_MFE_data_for_Lander
from opt_set import project_path, ipopt_opt_lines, write_ipopt_opt, set_ipopt_opt
from plot.plot_data_lander import Plot_data_lander


# optimize the L3L problem using SC
# The result of solving this problem is to provide the initial data for EMFE
def opt():

    #### make data dir ####
    local_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    dirs_name = project_path + '/output/Lander/' + 'opt_' + local_time
    os.makedirs(dirs_name)
    #initial data in the output file
    initial_data_path = 'YOUR ADRESS'    # any folder in output/Lander
    TempfileManager.tempdir = dirs_name
    #### parameter set ####
    nfe = 15
    ncp = 3
    warm_start_flag = True

    Opt_model = Lunar_Lander_3dof_model(nfe, ncp)

    Opt_model.discretize_variable()
    Opt_model.initialize_variable(initial_data_path)

    Opt_model.add_dynamic_constraints()
    Opt_model.add_process_constraints()
    Opt_model.add_initial_final_state_constraints()
    Opt_model.add_objective()

    Opt_model.m.tf.pprint()

    #### slove ####
    start_time = time.time()

    solver = SolverFactory('ipopt', solver_io='nl')
    solver.reset()
    solver.options['halt_on_ampl_error'] = 'yes'
    solver.options['output_file'] = dirs_name + '/ipopt.txt'

    if not warm_start_flag:
        pass
    else:
        load_dual_suffix(Opt_model.m, initial_data_path, '/dual.txt')
        load_ipopt_zL_out_suffix(Opt_model.m, initial_data_path, '/ipopt_zL_out.txt')
        load_ipopt_zU_out_suffix(Opt_model.m, initial_data_path, '/ipopt_zU_out.txt')
        solver.options['warm_start_init_point'] = 'yes'
        solver.options['warm_start_bound_push'] = 1e-6
        solver.options['warm_start_mult_bound_push'] = 1e-6
        solver.options['mu_init'] = 1e-6

    solver.solve(Opt_model.m, tee=True, keepfiles=False)

    #### log data ####
    log_data_for_Lander(Opt_model.m, nfe, ncp, dirs_name)
    save_dual_suffix(Opt_model.m, dirs_name, '/dual.txt')
    save_ipopt_zL_out_suffix(Opt_model.m, dirs_name, '/ipopt_zL_out.txt')
    save_ipopt_zU_out_suffix(Opt_model.m, dirs_name, '/ipopt_zU_out.txt')

    ### check output ###
    with open(dirs_name  + '/ipopt.txt', 'r') as f:
        lines = f.readlines()
        last_line = lines[-1]
        endtime = time.time()

        if last_line == 'EXIT: Solved To Acceptable Level.\n' or last_line == 'EXIT: Optimal Solution Found.\n':
            output = 1
        else:
            shutil.rmtree(dirs_name)
            output = 0

    # ### plot ###
    if output == 1:

        plot_nfe_flag = True
        data_path = [dirs_name]#, initial_data_path]
        picture_category = ['pos', 'vel', 'thrust', 'u', 'tf']
        file_name = ['/data.csv']

        ###############
        Plot_trajectory = Plot_data_lander(data_path, file_name, picture_category)
        Plot_trajectory.run_plot(plot_nfe_flag=plot_nfe_flag)


# optimize the L3L problem using EMFE
def opt_MFE():

    #### make data dir ####
    local_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    dirs_name = project_path + '/output/Lander/' + 'opt_EMFE_' + local_time
    os.makedirs(dirs_name)
    #initial data in the output file
    initial_data_path = 'YOUR ADRESS'
    TempfileManager.tempdir = dirs_name

    #### parameter set ####
    nfe = 15
    ncp = 3
    mu_list = [1e3, 1]
    warm_start_flag = True
    load_MFE_data_flag = True
    error_upper_flag = 0

    Opt_model = Lunar_Lander_3dof_EMFE_model(nfe, ncp)

    Opt_model.discretize_variable()
    Opt_model.initialize_variable(initial_data_path, error_upper_flag, load_MFE_data_flag=load_MFE_data_flag)
    print(Opt_model.n_i)
    Opt_model.m.tau.pprint()

    Opt_model.add_dynamic_constraints()

    Opt_model.add_process_constraints()
    Opt_model.add_numerical_error_constraints()
    Opt_model.add_initial_final_state_constraints()

    Opt_model.add_objective(mu_list)

    Opt_model.m.tf.pprint()
    # Opt_model.m.noncollocation_error.pprint()

    #### slove ####
    start_time = time.time()

    solver = SolverFactory('ipopt', solver_io='nl')
    solver.reset()
    solver.options['halt_on_ampl_error'] = 'yes'
    solver.options['output_file'] = dirs_name + '/ipopt.txt'

    if not warm_start_flag:
        pass
    else:
        load_dual_suffix(Opt_model.m, initial_data_path, '/dual.txt')
        load_ipopt_zL_out_suffix(Opt_model.m, initial_data_path, '/ipopt_zL_out.txt')
        load_ipopt_zU_out_suffix(Opt_model.m, initial_data_path, '/ipopt_zU_out.txt')
        solver.options['warm_start_init_point'] = 'yes'
        solver.options['warm_start_bound_push'] = 1e-6
        solver.options['warm_start_mult_bound_push'] = 1e-6
        solver.options['mu_init'] = 1e-6

    set_ipopt_opt(['bound_push 1e-06\n'],['bound_push 1e-06\n'])
    solver.solve(Opt_model.m, tee=True, keepfiles=False)
    set_ipopt_opt(['bound_push 1e-06\n'],['bound_push 1e-06\n'])

    #### log data ####
    log_MFE_data_for_Lander(Opt_model.m, nfe, ncp, dirs_name)
    save_dual_suffix(Opt_model.m, dirs_name, '/dual.txt')
    save_ipopt_zL_out_suffix(Opt_model.m, dirs_name, '/ipopt_zL_out.txt')
    save_ipopt_zU_out_suffix(Opt_model.m, dirs_name, '/ipopt_zU_out.txt')

    ### check output ###
    with open(dirs_name  + '/ipopt.txt', 'r') as f:
        lines = f.readlines()
        last_line = lines[-1]
        endtime = time.time()

        if last_line == 'EXIT: Solved To Acceptable Level.\n' or last_line == 'EXIT: Optimal Solution Found.\n':
            output = 1
        else:
            shutil.rmtree(dirs_name)
            output = 0

    # ### plot ###
    if output == 1:

        plot_nfe_flag = True
        data_path = [dirs_name]#, initial_data_path]
        picture_category = ['pos', 'vel', 'thrust', 'u', 'tf', 'noncollocation_error']
        file_name = ['/data.csv']

        ###############
        Plot_trajectory = Plot_data_lander(data_path, file_name, picture_category)
        Plot_trajectory.run_plot(plot_nfe_flag=plot_nfe_flag)


if __name__ == '__main__':
    write_ipopt_opt(ipopt_opt_lines)
    # opt()
    opt_MFE()
