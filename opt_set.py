import os

project_path = '/home/gyskyfall/PycharmProjects/EMFE_demo'
ipopt_opt_lines = [
    'check_derivatives_for_naninf yes\n',
    'derivative_test_print_all yes\n',
    'derivative_test none\n',
    '\n',
    'max_iter 100000\n'
    '\n'
    'acceptable_tol 1e-06\n',
    'bound_push 1e-06\n',
    'mu_target 0\n',
    '\n',
    'linear_solver MA57\n',
    'ma57_automatic_scaling no\n',
    '\n',
    'bound_mult_init_method mu-based\n',
    '\n',
    'mu_strategy adaptive\n',
    'adaptive_mu_globalization obj-constr-filter\n',
    'corrector_type affine\n',
    '\n',
    'print_level 5\n',
    'print_timing_statistics yes\n',
    'print_user_options yes\n',
]

def write_ipopt_opt(lines):
    with open(project_path+'/ipopt.opt', 'w+') as ipopt_opt:
        for i in lines:
            ipopt_opt.write(i)


def set_ipopt_opt(raw_values, change_values):
    with open(project_path+'/ipopt.opt', 'r+') as ipopt_opt:
        lines = ipopt_opt.readlines()

    for i in range(len(raw_values)):
        index = lines.index(raw_values[i])
        lines[index] = change_values[i]

    with open(project_path+'/ipopt.opt', 'w+') as ipopt_opt:
        for i in lines:
            ipopt_opt.write(i)

