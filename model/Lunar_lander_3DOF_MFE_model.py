from pyomo.environ import *
from pyomo.dae import *
import numpy as np
from data_process.data_process import create_MFE_initial_data_for_Lander


class Lunar_Lander_3dof_EMFE_model:
    def __init__(self, nfe, ncp):
        self.m = ConcreteModel()
        ########################################################### SET ###########################################################
        self.m.Vec_nfe = Set(initialize=list(np.linspace(0, nfe - 1, nfe, dtype=int)), ordered=True)
        self.n_i = list(np.linspace(0, 1, nfe + 1))
        self.nfe = nfe
        self.ncp = ncp
        ###################################### Define Parameter ######################################
        self.m.M_dry = Param(initialize=3300)
        self.m.alpha_m_rp1 = Param(initialize=(1 / 2942))
        self.m.g_M = Param(initialize=-1.62)
        # self.m.omegaB_max = Param(initialize=np.pi/12)
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

        self.m.noncollocation_error_ref = Param(self.m.Vec_nfe, within=NonNegativeReals, mutable=True)
        ###################################### Define Variable ######################################
        self.m.noncollocation_error = Var(self.m.Vec_nfe, within=NonNegativeReals, bounds=(0, 1e2))
        self.m.virtual_noncollocation_error_sum = Var(within=NonNegativeReals, initialize=0, bounds=(0, 1e2))

        self.m.tau = ContinuousSet(bounds=(0, 1), initialize=self.n_i)  # Unscaled time
        self.m.time = Var(self.m.tau, initialize=0)  # Scaled time
        self.m.tf = Var(self.m.Vec_nfe, initialize=150, bounds=(100, 230), within=NonNegativeReals)

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
        self.m.thrust = Var(self.m.tau, bounds=(0.2 * 15000, 15000), initialize=15000)

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
        return m.time_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)]

    @staticmethod
    def _rx_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.rx_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * m.vx[i]

    @staticmethod
    def _ry_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.ry_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * m.vy[i]

    @staticmethod
    def _rz_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.rz_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * m.vz[i]

    @staticmethod
    def _vx_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vx_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * (m.g_M + m.thrust[i] * m.ux[i] / (m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _vy_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vy_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * (m.thrust[i] * m.uy[i] / (m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _vz_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.vz_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * (m.thrust[i] * m.uz[i] / (m.M_dry + m.mass_prop[i]))

    @staticmethod
    def _mass_prop_dot(m, i):
        if i == 0:
            return Constraint.Skip
        return m.mass_prop_dt[i] == m.tf[int((sorted(m.tau).index(i)-1)/3)] * (-m.alpha_m_rp1 * m.thrust[i])

    ########################## Path Constraints ########################################
    @staticmethod
    def _u_con(m, i):
        if i == 0:
            return Constraint.Skip
        return m.ux[i] ** 2 + m.uy[i] ** 2 + m.uz[i] ** 2 == 1

    ########################## Control Variable Constraints ########################################
    @staticmethod
    def _control_gradient_con(m, ncp):
        tau_list = sorted(m.tau)
        for j in sorted(m.Vec_nfe):
            k = ncp * j
            yield (-3.22474491303679 * m.thrust[tau_list[k + 1]] + 4.85773803496128 * m.thrust[tau_list[k + 2]] - 1.63299312192449 * m.thrust[tau_list[k + 3]]) * (-0.857738062907135 * m.thrust[tau_list[k + 1]] - 0.775255059017351 * m.thrust[tau_list[k + 2]] + 1.63299312192449 * m.thrust[tau_list[k + 3]]) >= 0
            yield (-0.857738062907135 * m.thrust[tau_list[k + 1]] - 0.775255059017351 * m.thrust[tau_list[k + 2]] + 1.63299312192449 * m.thrust[tau_list[k + 3]]) * (0.857738062907135 * m.thrust[tau_list[k + 1]] - 4.85773803496128 * m.thrust[tau_list[k + 2]] + 3.99999997205415 * m.thrust[tau_list[k + 3]]) >= 0

    ########################## Non_collocation point error Constraints ########################################
    @staticmethod
    def _noncollocation_point_error_con(m, ncp):
        tau_list = sorted(m.tau)
        for j in sorted(m.Vec_nfe):
            k = ncp*j
            ### triangle aera
            yield m.noncollocation_error[j] == 1*((0.99068111469964*sqrt((-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(m.g_M + (1.26451956243643*m.thrust[tau_list[k+1]] - 0.411152213318503*m.thrust[tau_list[k+2]] + 0.146632650882071*m.thrust[tau_list[k+3]])*(1.26451956243643*m.ux[tau_list[k+1]] - 0.411152213318503*m.ux[tau_list[k+2]] + 0.146632650882071*m.ux[tau_list[k+3]])/(m.M_dry + 0.40579463164973*m.mass_prop[tau_list[k]] + 0.632259781218216*m.mass_prop[tau_list[k+1]] - 0.0494221846433941*m.mass_prop[tau_list[k+2]] + 0.0113677717754477*m.mass_prop[tau_list[k+3]])) - m.vx[tau_list[k]] + 0.994750412688707*m.vx[tau_list[k+1]] + 0.0083851019561501*m.vx[tau_list[k+2]] - 0.00313551464485701*m.vx[tau_list[k+3]])**2 + (-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.40579463164973*m.vx[tau_list[k]] + 0.632259781218216*m.vx[tau_list[k+1]] - 0.0494221846433941*m.vx[tau_list[k+2]] + 0.0113677717754477*m.vx[tau_list[k+3]]) - m.rx[tau_list[k]] + 0.994750412688707*m.rx[tau_list[k+1]] + 0.0083851019561501*m.rx[tau_list[k+2]] - 0.00313551464485701*m.rx[tau_list[k+3]])**2 + (-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.40579463164973*m.vy[tau_list[k]] + 0.632259781218216*m.vy[tau_list[k+1]] - 0.0494221846433941*m.vy[tau_list[k+2]] + 0.0113677717754477*m.vy[tau_list[k+3]]) - m.ry[tau_list[k]] + 0.994750412688707*m.ry[tau_list[k+1]] + 0.0083851019561501*m.ry[tau_list[k+2]] - 0.00313551464485701*m.ry[tau_list[k+3]])**2 + (-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.40579463164973*m.vz[tau_list[k]] + 0.632259781218216*m.vz[tau_list[k+1]] - 0.0494221846433941*m.vz[tau_list[k+2]] + 0.0113677717754477*m.vz[tau_list[k+3]]) - m.rz[tau_list[k]] + 0.994750412688707*m.rz[tau_list[k+1]] + 0.0083851019561501*m.rz[tau_list[k+2]] - 0.00313551464485701*m.rz[tau_list[k+3]])**2 + (0.156509524305416*(tau_list[k+3]-tau_list[k])*m.alpha_m_rp1*m.tf[j]*(1.26451956243643*m.thrust[tau_list[k+1]] - 0.411152213318503*m.thrust[tau_list[k+2]] + 0.146632650882071*m.thrust[tau_list[k+3]]) - m.mass_prop[tau_list[k]] + 0.994750412688707*m.mass_prop[tau_list[k+1]] + 0.0083851019561501*m.mass_prop[tau_list[k+2]] - 0.00313551464485701*m.mass_prop[tau_list[k+3]])**2 + (-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(1.26451956243643*m.thrust[tau_list[k+1]] - 0.411152213318503*m.thrust[tau_list[k+2]] + 0.146632650882071*m.thrust[tau_list[k+3]])*(1.26451956243643*m.uy[tau_list[k+1]] - 0.411152213318503*m.uy[tau_list[k+2]] + 0.146632650882071*m.uy[tau_list[k+3]])/(m.M_dry + 0.40579463164973*m.mass_prop[tau_list[k]] + 0.632259781218216*m.mass_prop[tau_list[k+1]] - 0.0494221846433941*m.mass_prop[tau_list[k+2]] + 0.0113677717754477*m.mass_prop[tau_list[k+3]]) - m.vy[tau_list[k]] + 0.994750412688707*m.vy[tau_list[k+1]] + 0.0083851019561501*m.vy[tau_list[k+2]] - 0.00313551464485701*m.vy[tau_list[k+3]])**2 + (-0.156509524305416*(tau_list[k+3]-tau_list[k])*m.tf[j]*(1.26451956243643*m.thrust[tau_list[k+1]] - 0.411152213318503*m.thrust[tau_list[k+2]] + 0.146632650882071*m.thrust[tau_list[k+3]])*(1.26451956243643*m.uz[tau_list[k+1]] - 0.411152213318503*m.uz[tau_list[k+2]] + 0.146632650882071*m.uz[tau_list[k+3]])/(m.M_dry + 0.40579463164973*m.mass_prop[tau_list[k]] + 0.632259781218216*m.mass_prop[tau_list[k+1]] - 0.0494221846433941*m.mass_prop[tau_list[k+2]] + 0.0113677717754477*m.mass_prop[tau_list[k+3]]) - m.vz[tau_list[k]] + 0.994750412688707*m.vz[tau_list[k+1]] + 0.0083851019561501*m.vz[tau_list[k+2]] - 0.00313551464485701*m.vz[tau_list[k+3]])**2))\
                                                    + (0.728989781697479*sqrt((-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(m.g_M + (0.355051027519449*m.thrust[tau_list[k+1]] + 0.844948964096795*m.thrust[tau_list[k+2]] - 0.199999991616244*m.thrust[tau_list[k+3]])*(0.355051027519449*m.ux[tau_list[k+1]] + 0.844948964096795*m.ux[tau_list[k+2]] - 0.199999991616244*m.ux[tau_list[k+3]])/(m.M_dry - 0.359999979878985*m.mass_prop[tau_list[k]] + 0.915959158786495*m.mass_prop[tau_list[k+1]] + 0.524040817738988*m.mass_prop[tau_list[k+2]] - 0.0799999966464974*m.mass_prop[tau_list[k+3]])) + 0.20160690516595*m.vx[tau_list[k]] - m.vx[tau_list[k+1]] + 0.865595397495047*m.vx[tau_list[k+2]] - 0.0672023026609962*m.vx[tau_list[k+3]])**2 + (-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(-0.359999979878985*m.vx[tau_list[k]] + 0.915959158786495*m.vx[tau_list[k+1]] + 0.524040817738988*m.vx[tau_list[k+2]] - 0.0799999966464974*m.vx[tau_list[k+3]]) + 0.20160690516595*m.rx[tau_list[k]] - m.rx[tau_list[k+1]] + 0.865595397495047*m.rx[tau_list[k+2]] - 0.0672023026609962*m.rx[tau_list[k+3]])**2 + (-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(-0.359999979878985*m.vy[tau_list[k]] + 0.915959158786495*m.vy[tau_list[k+1]] + 0.524040817738988*m.vy[tau_list[k+2]] - 0.0799999966464974*m.vy[tau_list[k+3]]) + 0.20160690516595*m.ry[tau_list[k]] - m.ry[tau_list[k+1]] + 0.865595397495047*m.ry[tau_list[k+2]] - 0.0672023026609962*m.ry[tau_list[k+3]])**2 + (-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(-0.359999979878985*m.vz[tau_list[k]] + 0.915959158786495*m.vz[tau_list[k+1]] + 0.524040817738988*m.vz[tau_list[k+2]] - 0.0799999966464974*m.vz[tau_list[k+3]]) + 0.20160690516595*m.rz[tau_list[k]] - m.rz[tau_list[k+1]] + 0.865595397495047*m.rz[tau_list[k+2]] - 0.0672023026609962*m.rz[tau_list[k+3]])**2 + (0.336011527390175*(tau_list[k+3]-tau_list[k])*m.alpha_m_rp1*m.tf[j]*(0.355051027519449*m.thrust[tau_list[k+1]] + 0.844948964096795*m.thrust[tau_list[k+2]] - 0.199999991616244*m.thrust[tau_list[k+3]]) + 0.20160690516595*m.mass_prop[tau_list[k]] - m.mass_prop[tau_list[k+1]] + 0.865595397495047*m.mass_prop[tau_list[k+2]] - 0.0672023026609962*m.mass_prop[tau_list[k+3]])**2 + (-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.355051027519449*m.thrust[tau_list[k+1]] + 0.844948964096795*m.thrust[tau_list[k+2]] - 0.199999991616244*m.thrust[tau_list[k+3]])*(0.355051027519449*m.uy[tau_list[k+1]] + 0.844948964096795*m.uy[tau_list[k+2]] - 0.199999991616244*m.uy[tau_list[k+3]])/(m.M_dry - 0.359999979878985*m.mass_prop[tau_list[k]] + 0.915959158786495*m.mass_prop[tau_list[k+1]] + 0.524040817738988*m.mass_prop[tau_list[k+2]] - 0.0799999966464974*m.mass_prop[tau_list[k+3]]) + 0.20160690516595*m.vy[tau_list[k]] - m.vy[tau_list[k+1]] + 0.865595397495047*m.vy[tau_list[k+2]] - 0.0672023026609962*m.vy[tau_list[k+3]])**2 + (-0.336011527390175*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.355051027519449*m.thrust[tau_list[k+1]] + 0.844948964096795*m.thrust[tau_list[k+2]] - 0.199999991616244*m.thrust[tau_list[k+3]])*(0.355051027519449*m.uz[tau_list[k+1]] + 0.844948964096795*m.uz[tau_list[k+2]] - 0.199999991616244*m.uz[tau_list[k+3]])/(m.M_dry - 0.359999979878985*m.mass_prop[tau_list[k]] + 0.915959158786495*m.mass_prop[tau_list[k+1]] + 0.524040817738988*m.mass_prop[tau_list[k+2]] - 0.0799999966464974*m.mass_prop[tau_list[k+3]]) + 0.20160690516595*m.vz[tau_list[k]] - m.vz[tau_list[k+1]] + 0.865595397495047*m.vz[tau_list[k+2]] - 0.0672023026609962*m.vz[tau_list[k+3]])**2))\
                                                    + (0.481350762238331*sqrt((-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(m.g_M + (-0.0761351956763457*m.thrust[tau_list[k+1]] + 0.681186223195795*m.thrust[tau_list[k+2]] + 0.394948972480551*m.thrust[tau_list[k+3]])*(-0.0761351956763457*m.ux[tau_list[k+1]] + 0.681186223195795*m.ux[tau_list[k+2]] + 0.394948972480551*m.ux[tau_list[k+3]])/(m.M_dry + 0.210340556275919*m.mass_prop[tau_list[k]] - 0.403862237189116*m.mass_prop[tau_list[k+1]] + 0.868686228170977*m.mass_prop[tau_list[k+2]] + 0.32483545274222*m.mass_prop[tau_list[k+3]])) + 0.116230651101687*m.vx[tau_list[k]] - 0.181096447081233*m.vx[tau_list[k+1]] - 0.935134204020455*m.vx[tau_list[k+2]] + m.vx[tau_list[k+3]])**2 + (-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.210340556275919*m.vx[tau_list[k]] - 0.403862237189116*m.vx[tau_list[k+1]] + 0.868686228170977*m.vx[tau_list[k+2]] + 0.32483545274222*m.vx[tau_list[k+3]]) + 0.116230651101687*m.rx[tau_list[k]] - 0.181096447081233*m.rx[tau_list[k+1]] - 0.935134204020455*m.rx[tau_list[k+2]] + m.rx[tau_list[k+3]])**2 + (-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.210340556275919*m.vy[tau_list[k]] - 0.403862237189116*m.vy[tau_list[k+1]] + 0.868686228170977*m.vy[tau_list[k+2]] + 0.32483545274222*m.vy[tau_list[k+3]]) + 0.116230651101687*m.ry[tau_list[k]] - 0.181096447081233*m.ry[tau_list[k+1]] - 0.935134204020455*m.ry[tau_list[k+2]] + m.ry[tau_list[k+3]])**2 + (-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(0.210340556275919*m.vz[tau_list[k]] - 0.403862237189116*m.vz[tau_list[k+1]] + 0.868686228170977*m.vz[tau_list[k+2]] + 0.32483545274222*m.vz[tau_list[k+3]]) + 0.116230651101687*m.rz[tau_list[k]] - 0.181096447081233*m.rz[tau_list[k+1]] - 0.935134204020455*m.rz[tau_list[k+2]] + m.rz[tau_list[k+3]])**2 + (0.368806967655952*(tau_list[k+3]-tau_list[k])*m.alpha_m_rp1*m.tf[j]*(-0.0761351956763457*m.thrust[tau_list[k+1]] + 0.681186223195795*m.thrust[tau_list[k+2]] + 0.394948972480551*m.thrust[tau_list[k+3]]) + 0.116230651101687*m.mass_prop[tau_list[k]] - 0.181096447081233*m.mass_prop[tau_list[k+1]] - 0.935134204020455*m.mass_prop[tau_list[k+2]] + m.mass_prop[tau_list[k+3]])**2 + (-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(-0.0761351956763457*m.thrust[tau_list[k+1]] + 0.681186223195795*m.thrust[tau_list[k+2]] + 0.394948972480551*m.thrust[tau_list[k+3]])*(-0.0761351956763457*m.uy[tau_list[k+1]] + 0.681186223195795*m.uy[tau_list[k+2]] + 0.394948972480551*m.uy[tau_list[k+3]])/(m.M_dry + 0.210340556275919*m.mass_prop[tau_list[k]] - 0.403862237189116*m.mass_prop[tau_list[k+1]] + 0.868686228170977*m.mass_prop[tau_list[k+2]] + 0.32483545274222*m.mass_prop[tau_list[k+3]]) + 0.116230651101687*m.vy[tau_list[k]] - 0.181096447081233*m.vy[tau_list[k+1]] - 0.935134204020455*m.vy[tau_list[k+2]] + m.vy[tau_list[k+3]])**2 + (-0.368806967655952*(tau_list[k+3]-tau_list[k])*m.tf[j]*(-0.0761351956763457*m.thrust[tau_list[k+1]] + 0.681186223195795*m.thrust[tau_list[k+2]] + 0.394948972480551*m.thrust[tau_list[k+3]])*(-0.0761351956763457*m.uz[tau_list[k+1]] + 0.681186223195795*m.uz[tau_list[k+2]] + 0.394948972480551*m.uz[tau_list[k+3]])/(m.M_dry + 0.210340556275919*m.mass_prop[tau_list[k]] - 0.403862237189116*m.mass_prop[tau_list[k+1]] + 0.868686228170977*m.mass_prop[tau_list[k+2]] + 0.32483545274222*m.mass_prop[tau_list[k+3]]) + 0.116230651101687*m.vz[tau_list[k]] - 0.181096447081233*m.vz[tau_list[k+1]] - 0.935134204020455*m.vz[tau_list[k+2]] + m.vz[tau_list[k+3]])**2)))

    @staticmethod
    def _noncollocation_error_magcon(m):
        return sum(m.noncollocation_error_ref[i] - m.noncollocation_error[i] for i in sorted(m.Vec_nfe)) >= m.virtual_noncollocation_error_sum

    ############################## objective function ##########################
    @staticmethod
    def _ObjRule(m, mu_list):
        return mu_list[0]*m.mass_prop[1] + mu_list[1]*(m.virtual_noncollocation_error_sum)

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
    def _load_initial_data(m, data_path, nfe, ncp, error_upper_flag, load_MFE_data_flag):
        initial_data_dict = create_MFE_initial_data_for_Lander(data_path, nfe, ncp, list(m.tau),load_MFE_data_flag)

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
            if j % 3 == 0 and j != 0:
                if load_MFE_data_flag:
                    m.tf[int(j / 3) - 1] = initial_data_dict.get('tf_MFE_init')[j, 0]
                    m.noncollocation_error[int(j / 3) - 1] = initial_data_dict.get('noncollocation_error_init')[j, 0]
                    m.noncollocation_error_ref[int(j / 3) - 1] = initial_data_dict.get('noncollocation_error_init')[j, 0]
                    if error_upper_flag == 0:
                        m.noncollocation_error[int(j / 3) - 1].setub(np.max(initial_data_dict['noncollocation_error_init']))
                    elif error_upper_flag == -1:
                        m.noncollocation_error[int(j / 3) - 1].setub(100)
                    else:
                        m.noncollocation_error[int(j / 3) - 1].setub(initial_data_dict.get('max_error_init')[j, 0])

                else:
                    m.tf[int(j / 3) - 1] = initial_data_dict.get('time_init')[-1, 0]
                    m.noncollocation_error[int(j / 3) - 1] = 0
                    m.noncollocation_error_ref[int(j / 3) - 1] = 0
                    m.tf[int(j / 3) - 1].fixed = True

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

    def add_numerical_error_constraints(self):
        self.m.control_gradient_con = ConstraintList(rule=self._control_gradient_con(self.m, self.ncp))
        self.m.noncollocation_point_error_con = ConstraintList(rule=self._noncollocation_point_error_con(self.m, self.ncp))
        self.m.noncollocation_error_magcon = Constraint(rule=self._noncollocation_error_magcon)

    def add_initial_final_state_constraints(self):
        self.m.initstate = ConstraintList(rule=self._initstate)

    ############################## add objective ##########################
    def add_objective(self, mu_list):
        self.m.obj = Objective(rule=self._ObjRule(self.m, mu_list), sense=maximize)

    ############################## variables initialize ##########################

    def discretize_variable(self):
        self._discretize(self.m, self.nfe, self.ncp)

    def initialize_variable(self, data_path, error_upper_flag, load_MFE_data_flag):
        self._load_initial_data(self.m, data_path + '/data.csv', self.nfe, self.ncp, error_upper_flag, load_MFE_data_flag)
