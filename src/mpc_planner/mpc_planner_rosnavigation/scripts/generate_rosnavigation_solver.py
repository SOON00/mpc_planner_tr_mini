#!/usr/bin/python3

import os
import sys
import yaml

# TODO: Import packages through pypi
sys.path.append(os.path.join(sys.path[0], "..", "..", "solver_generator"))
sys.path.append(os.path.join(sys.path[0], "..", "..", "mpc_planner_modules", "scripts"))

from control_modules import ModuleManager
from generate_solver import generate_solver

# Import modules here from mpc_planner_modules
from mpc_base import MPCBaseModule
from contouring import ContouringModule

from decomp_constraints import DecompConstraintModule

# Import solver models that you want to use
from solver_model import ContouringSecondOrderUnicycleModelWithSlack




def configuration_tmpc(settings):
    modules = ModuleManager()
    model = ContouringSecondOrderUnicycleModelWithSlack()

    base_module = modules.add_module(MPCBaseModule(settings))
    base_module.weigh_variable(var_name="a", weight_names="acceleration")
    base_module.weigh_variable(var_name="w", weight_names="angular_velocity")
    base_module.weigh_variable(var_name="slack", weight_names="slack")
    if not settings["contouring"]["dynamic_velocity_reference"]:
        base_module.weigh_variable(var_name="v",    
                                weight_names=["velocity", "reference_velocity"], 
                                cost_function=lambda x, w: w[0] * (x-w[1])**2)

    modules.add_module(ContouringModule(settings))
    modules.add_module(DecompConstraintModule(settings))

    return model, modules

def load_settings():
    path = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), "../config/settings.yaml")
    with open(path, "r") as stream:
        settings = yaml.safe_load(stream)
    print("Settings :", path)
    return settings


settings = load_settings()

model, modules = configuration_tmpc(settings)

generate_solver(modules, model, settings)
