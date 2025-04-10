import yaml
import os

current_path = os.path.dirname(os.path.abspath(__file__))
parent_path = os.path.dirname(current_path) 
print(parent_path)

with open(parent_path+'/config/default.yaml', 'r') as f:
    default_values = yaml.load(f, Loader=yaml.Loader)
    
print(default_values)
print(default_values['mpc_params']['objective']['alpha_CL'])