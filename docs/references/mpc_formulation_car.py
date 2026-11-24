import casadi as cs
import numpy as np
import torch
import l4casadi as l4c
from acados_template import AcadosSimSolver, AcadosOcpSolver, AcadosSim, AcadosOcp, AcadosModel
import time
import os
import scipy
import matplotlib.pyplot as plt
from dataclasses import dataclass
import random
import string



class MPC:
    def __init__(self, model, dt, cfg_mpc, random_name=False, sensitivity=False, sensitivity_init_iterate=False, p_global=None):
        self.N = cfg_mpc.N
        self.horizon = dt * self.N
        self.cfg_mpc = cfg_mpc
        self.model = model
        self.sensitivity = sensitivity
        self.sensitivity_init_iterate = sensitivity_init_iterate
        self.p_global = p_global
        
        # self.export_code_dir = 'c_generated_code/'

        # if random_name:
        #     self.export_code_dir += ''.join(random.choices(
        #         string.ascii_uppercase + string.digits, k=10))

    @property
    def solver(self):
        return AcadosOcpSolver(self.ocp(), verbose=True)
    
    @property
    def simulator(self):
        self.sim = AcadosSim()
        self.sim.model = self.acados_model(self.model)
        if not self.sensitivity:
            self.sim.parameter_values = self.model.parameter_values
        self.sim.solver_options.integrator_type = 'ERK'
        self.sim.solver_options.num_stages = 4
        self.sim.solver_options.num_steps = 10
        self.sim.solver_options.T = self.horizon / self.N
        self.integrator = AcadosSimSolver(self.sim, verbose=True)
        return self.integrator


    def ocp(self):
        model = self.model

        t_horizon = self.horizon
        N = self.N

        # Get model
        model_ac = self.acados_model(model=model)

        # Dimensions
        nx = 3+4+2
        nu = 2
        ny = nx + nu
        ny_e = nx
        
        # Create OCP object to formulate the optimization
        ocp = AcadosOcp()
        ocp.model = model_ac
        # ocp.dims.nx = nx
        # ocp.dims.nu = nu
        # ocp.dims.ny = ny
        ocp.solver_options.tf = t_horizon
        ocp.solver_options.N_horizon = N

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        ocp.model.cost_expr_ext_cost = model.cost_expr_ext_cost
        ocp.model.cost_expr_ext_cost_e = model.cost_expr_ext_cost_e * self.cfg_mpc.q_e_scaler

        # state constrains
        ocp.constraints.lbx = np.array([])
        ocp.constraints.ubx = np.array([])
        ocp.constraints.idxbx = np.array([])
        ocp.constraints.x0 = model.x_start

        # control constraints
        max_steering_ref = 0.5
        min_wheel_speed_ref = self.cfg_mpc.min_wheel_speed_ref
        max_wheel_speed_ref = self.cfg_mpc.max_wheel_speed_ref
        ocp.constraints.lbu = np.array([min_wheel_speed_ref, -max_steering_ref])
        ocp.constraints.ubu = np.array([max_wheel_speed_ref, max_steering_ref])
        ocp.constraints.idxbu = np.array([0, 1])

        # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_warm_start = 0
        ocp.solver_options.hpipm_mode = 'BALANCE' 
        
        ocp.solver_options.hessian_approx = 'EXACT'
        ocp.solver_options.regularize_method = 'PROJECT'  # 'NO_REGULARIZE'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.integrator_type = 'DISCRETE'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        
        ocp.parameter_values = model.parameter_values    
        
        if self.sensitivity_init_iterate:
            ocp.solver_options.regularize_method = 'PROJECT'
            ocp.solver_options.nlp_solver_type = 'SQP'
            ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
            ocp.solver_options.tol = 5e-5
            ocp.p_global_values = model.p_global_values
            ocp.solver_options.with_solution_sens_wrt_params = True
            ocp.solver_options.nlp_solver_max_iter = 500
            
        if self.sensitivity: 
           ocp.solver_options.nlp_solver_type = 'SQP'
           ocp.solver_options.globalization_fixed_step_length = 0.5
           ocp.solver_options.tol = 5e-5
           ocp.solver_options.nlp_solver_max_iter = 500
           ocp.solver_options.qp_solver_ric_alg = 1
           ocp.solver_options.regularize_method = 'NO_REGULARIZE'
           ocp.solver_options.globalization = 'FIXED_STEP'
           ocp.solver_options.with_solution_sens_wrt_params = True
           ocp.p_global_values = model.p_global_values
           # ocp.solver_options.qp_solver_mu0 = 1e3
           
        #    ocp.solver_options.with_value_sens_wrt_params = True
           
        # ocp.solver_options.qp_solver_mu0 = 1e3  # makes HPIPM converge more robustly
        # ocp.solver_options.globalization_fixed_step_length = 0.1
        # ocp.solver_options.qp_tol = 1e-4
        
        # ocp.code_export_directory = self.export_code_dir
        # ocp.solver_options.levenberg_marquardt = 1e-6
        # ocp.solver_options.model_external_shared_lib_dir = model.external_shared_lib_dir
        # ocp.solver_options.model_external_shared_lib_name = model.external_shared_lib_name
        return ocp

    def acados_model(self, model):
        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.xdot - model.f_expl
        model_ac.f_expl_expr = model.f_expl
        model_ac.disc_dyn_expr = model.disc_dyn_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.name = model.name
        
        # model_ac.p = model.p
        # model_ac.parameter_values = model.parameter_values
        # model_ac.p_global = model.p_global
            
        if self.sensitivity or self.sensitivity_init_iterate:
            model_ac.p = model.p
            model_ac.parameter_values = model.parameter_values
            model_ac.p_global = model.p_global
        else:
            model_ac.p = model.p
            model_ac.parameter_values = model.parameter_values
        
        print(f"model_ac.p: {model_ac.p.shape}")
        print(f"model_ac.parameter_values: {model_ac.parameter_values.shape}")
        return model_ac
