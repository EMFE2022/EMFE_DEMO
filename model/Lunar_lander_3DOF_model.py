
from pyomo.environ import *
from pyomo.dae import *
import numpy as np
from data_process.data_process import create_initial_data_for_Lander


class Lunar_Lander_3dof_model:
    def __init__(self, nfe, ncp):
        self.m = ConcreteModel()
        ########################################################### SET ###########################################################
        self.m.Vec_nfe = Set(initialize=list(np.linspace(0, nfe - 1, nfe, dtype=int)), ordered=True)
        self.n_i = list(np.linspace(0, 1, nfe+1))
        self.nfe = nfe
        self.ncp = ncp
        ###################################### Define Parameter ######################################
        self.m.M_dry = Param(initialize=3300)
        self.m.alpha_m_rp1 = Param(initialize=(1/2942))
        self.m.g_M = Param(initialize=-1.62)
        self.m.mountain_y = Param(initialize=361)
        self.m.mountain_z = Param(initialize=-226)
        self.m.safe_distance = Param(initialize=200)

        self.m.mass_prop_initial = Param(initialize=694)
        self.m.rx_initial = Param(initialize=10000)
        self.m.ry_initial = Param(initialize=2500)
        self.m.rz_initial = Param(initialize=-1300)
        self.m.vx_initial = Param(initialize=-15)
        self.m.vy_initial = Param(initialize=85)
        self.m.vz_initial = Param(initialize=-60)

        self.m.rx_final = Param(initialize=0)
        self.m.ry_final = Param(initialize=0)
        self.m.rz_final = Param(initialize=0)
        self.m.vx_final = Param(initialize=-1)
        self.m.vy_final = Param(initialize=0)
        self.m.vz_final = Param(initialize=0)
        ###################################### Define Variable ######################################
        self.m.tau = ContinuousSet(bounds=(0, 1), initialize=self.n_i)  # Unscaled time
        self.m.time = Var(self.m.tau, initialize=0)  # Scaled time
        self.m.tf = Var(initialize=150, bounds=(100, 500), within=NonNegativeReals)

        self.m.rx = Var(self.m.tau, bounds=(0, 11000), initialize=10000)
        self.m.ry = Var(self.m.tau, bounds=(-5000, 5000), initialize=2500)
        self.m.rz = Var(self.m.tau, bounds=(-4000, 4000), initialize=-1300)
        self.m.vx = Var(self.m.tau, bounds=(-200, 200), initialize=0)
        self.m.vy = Var(self.m.tau, bounds=(-200, 200), initialize=0)
        self.m.vz = Var(self.m.tau, bounds=(-200, 200), initialize=0)
        self.m.mass_prop = Var(self.m.tau, bounds=(0, 694), initialize=694)
        self.m.ux = Var(self.m.tau, bounds=(-1, 1), initialize=0)
        self.m.uy = Var(self.m.tau, bounds=(-1, 1), initialize=0)
        self.m.uz = Var(self.m.tau, bounds=(-1, 1), initialize=0)
        self.m.thrust = Var(self.m.tau, bounds=(0.2*15000, 15000), initialize=15000)

        self.m.rx_dt = DerivativeVar(self.m.rx)
        self.m.ry_dt = DerivativeVar(self.m.ry)
        self.m.rz_dt = DerivativeVar(self.m.rz)
        self.m.vx_dt = DerivativeVar(self.m.vx)
        self.m.vy_dt = DerivativeVar(self.m.vy)
        self.m.vz_dt = DerivativeVar(self.m.vz)

        self.m.mass_prop_dt = DerivativeVar(self.m.mass_prop)
        self.m.time_dt = DerivativeVar(self.m.time)
        ########################## Suffixes ########################################
        # Ipopt bound multipliers (obtained from solution)
        self.m.ipopt_zL_out = Suffix(direction=Suffix.IMPORT)
        self.m.ipopt_zU_out = Suffix(direction=Suffix.IMPORT)
        # Ipopt bound multipliers (sent to solver)
        self.m.ipopt_zL_in = Suffix(direction=Suffix.EXPORT)
        self.m.ipopt_zU_in = Suffix(direction=Suffix.EXPORT)
        # Obtain dual solutions from first solve and send to warm start
        self.m.dual = Suffix(direction=Suffix.IMPORT_EXPORT)

    ########################## Dynamic Constraints ########################################
    @staticmethod
    def _time_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.time_dt[i] == m.tf

    @staticmethod
    def _rx_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.rx_dt[i] == m.tf * m.vx[i]

    @staticmethod
    def _ry_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.ry_dt[i] == m.tf * m.vy[i]

    @staticmethod
    def _rz_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.rz_dt[i] == m.tf * m.vz[i]

    @staticmethod
    def _vx_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vx_dt[i] == m.tf * (m.g_M + m.thrust[i]*m.ux[i]/(m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _vy_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vy_dt[i] == m.tf * (m.thrust[i]*m.uy[i]/(m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _vz_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vz_dt[i] == m.tf * (m.thrust[i]*m.uz[i]/(m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _mass_prop_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.mass_prop_dt[i] == m.tf * (-m.alpha_m_rp1*m.thrust[i])

    ########################## Path Constraints ########################################
    # u constraint
    @staticmethod
    def _u_con(m, i):
        if i == 0:
            return Constraint.Skip
        return m.ux[i]**2 + m.uy[i]**2 + m.uz[i]**2 == 1

    @staticmethod
    def _avoidance_con(m, i):
        if i == 0:
            return Constraint.Skip
        return sqrt((m.ry[i] - m.mountain_y)**2 + (m.rz[i] - m.mountain_z)**2) >= m.safe_distance

    ############################## objective function ##########################
    @staticmethod
    def _ObjRule(m):
        return m.mass_prop[1]

    ############################## initial state ####################################
    @staticmethod
    def _initstate(m):
        yield m.time[0] == 0
        yield m.rx[0] == m.rx_initial
        yield m.ry[0] == m.ry_initial
        yield m.rz[0] == m.rz_initial
        yield m.vx[0] == m.vx_initial
        yield m.vy[0] == m.vy_initial
        yield m.vz[0] == m.vz_initial

        yield m.rx[1] == m.rx_final
        yield m.ry[1] == m.ry_final
        yield m.rz[1] == m.rz_final
        yield m.vx[1] == m.vx_final
        yield m.vy[1] == m.vy_final
        yield m.vz[1] == m.vz_final


    ############################## discretizer ###################################
    @staticmethod
    def _discretize(m, nfe, ncp):
        discretizer = TransformationFactory('dae.collocation')
        discretizer.apply_to(m, wrt=m.tau, nfe=nfe, ncp=ncp, scheme='LAGRANGE-RADAU')

    ############################# variables initialize ##########################
    @staticmethod
    def _load_initial_data(m, data_path, nfe, ncp):
        initial_data_dict = create_initial_data_for_Lander(data_path, nfe, ncp, list(m.tau))

        for i in sorted(m.tau):
            j = sorted(m.tau).index(i)
            m.rx[i] = initial_data_dict.get('rx_init')[j, 0]
            m.ry[i] = initial_data_dict.get('ry_init')[j, 0]
            m.rz[i] = initial_data_dict.get('rz_init')[j, 0]
            m.vx[i] = initial_data_dict.get('vx_init')[j, 0]
            m.vy[i] = initial_data_dict.get('vy_init')[j, 0]
            m.vz[i] = initial_data_dict.get('vz_init')[j, 0]
            m.ux[i] = initial_data_dict.get('ux_init')[j, 0]
            m.uy[i] = initial_data_dict.get('uy_init')[j, 0]
            m.uz[i] = initial_data_dict.get('uz_init')[j, 0]
            m.thrust[i] = initial_data_dict.get('thrust_init')[j, 0]
            m.mass_prop[i] = initial_data_dict.get('mass_prop_init')[j, 0]

            if i != 0:
                m.rx_dt[i] = initial_data_dict.get('rx_dt_init')[j, 0]
                m.ry_dt[i] = initial_data_dict.get('ry_dt_init')[j, 0]
                m.rz_dt[i] = initial_data_dict.get('rz_dt_init')[j, 0]
                m.vx_dt[i] = initial_data_dict.get('vx_dt_init')[j, 0]
                m.vy_dt[i] = initial_data_dict.get('vy_dt_init')[j, 0]
                m.vz_dt[i] = initial_data_dict.get('vz_dt_init')[j, 0]
                m.mass_prop_dt[i] = initial_data_dict.get('mass_prop_dt_init')[j, 0]
        m.tf = initial_data_dict.get('tf_init')


    ############################## add constraints ##########################

    def add_dynamic_constraints(self):
        self.m.time_dot = Constraint(self.m.tau, rule=self._time_dot)
        self.m.rx_dot = Constraint(self.m.tau, rule=self._rx_dot)
        self.m.ry_dot = Constraint(self.m.tau, rule=self._ry_dot)
        self.m.rz_dot = Constraint(self.m.tau, rule=self._rz_dot)
        self.m.vx_dot = Constraint(self.m.tau, rule=self._vx_dot)
        self.m.vy_dot = Constraint(self.m.tau, rule=self._vy_dot)
        self.m.vz_dot = Constraint(self.m.tau, rule=self._vz_dot)
        self.m.mass_prop_dot = Constraint(self.m.tau, rule=self._mass_prop_dot)

    def add_process_constraints(self):
        self.m.u_con = Constraint(self.m.tau, rule=self._u_con)

    def add_initial_final_state_constraints(self):
        self.m.initstate = ConstraintList(rule=self._initstate)

    ############################## add objective ##########################
    def add_objective(self):
        self.m.obj = Objective(rule=self._ObjRule, sense=maximize)

    ############################## variables initialize ##########################

    def discretize_variable(self):
        self._discretize(self.m, self.nfe, self.ncp)

    def initialize_variable(self, data_path):
        self._load_initial_data(self.m, data_path + '/data.csv', self.nfe, self.ncp)
