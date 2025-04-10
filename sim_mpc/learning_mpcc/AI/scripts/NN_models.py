import os
import numpy as np
import datetime
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.optim.lr_scheduler import StepLR
from torch.utils.data import Dataset

class Model18(nn.Module):
    def __init__(self):
        super(Model18, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 32)
        self.fc2 = nn.Linear(32, 16)
        
        # RNN
        self.rnn_layer = nn.RNN(16, 16, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(16, 32)
        self.fc1_reversed = nn.Linear(32, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        # nn.init.xavier_uniform_(self.fc1.weight)
        # nn.init.constant_(self.fc1.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc2.weight)
        # nn.init.constant_(self.fc2.bias, 0.0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_uniform_(self.fc2_reversed.weight)
        # nn.init.constant_(self.fc2_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc1_reversed.weight)
        # nn.init.constant_(self.fc1_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
                # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 16)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model20(nn.Module):
    def __init__(self):
        super(Model20, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 16)
        self.fc2 = nn.Linear(16, 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(8, 16)
        self.fc1_reversed = nn.Linear(16, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model21(nn.Module):
    def __init__(self):
        super(Model21, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 8)
        self.fc2 = nn.Linear(8, 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(8, 8)
        self.fc1_reversed = nn.Linear(8, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model22(nn.Module):
    def __init__(self):
        super(Model22, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc1_reversed = nn.Linear(8, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        
        x2 = self.GELU(self.fc1(input2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc1_reversed(rnn_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model23(nn.Module):
    def __init__(self):
        super(Model23, self).__init__()
        
        input_shape = (2, 5)
        
        # RNN
        self.rnn_layer = nn.RNN(5, 5, 1, batch_first=True)
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        
        # RNN
        _, rnn_output = self.rnn_layer(inputs)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Output layer
        output = self.output(rnn_output)
        
        return output
    
class Model24(nn.Module):
    def __init__(self):
        super(Model24, self).__init__()
        
        self.fc1 = nn.Linear(5,32)
        self.fc2 = nn.Linear(32,32)
        self.fc3 = nn.Linear(32,32)
        self.fc4 = nn.Linear(32,3)
        self.relu = nn.ReLU()
        
    
    def forward(self, x):
        
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        x = self.relu(x)
        x = self.fc3(x)
        x = self.relu(x)
        x = self.fc4(x)
        return x
    
class Model25(nn.Module):
    def __init__(self):
        super(Model25, self).__init__()
        
        self.fc1 = nn.Linear(5,64)
        self.fc2 = nn.Linear(64,64)
        self.fc3 = nn.Linear(64,64)
        self.fc4 = nn.Linear(64,3)
        self.relu = nn.ReLU()
        
    
    def forward(self, x):
        
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        x = self.relu(x)
        x = self.fc3(x)
        x = self.relu(x)
        x = self.fc4(x)
        return x
    
class Model26(nn.Module):
    def __init__(self):
        super(Model26, self).__init__()
        
        # RNN
        self.rnn_layer = nn.RNN(5, 32, 1, batch_first=True)
        
        # Output layer
        self.output = nn.Linear(32, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        
        # RNN
        _, rnn_output = self.rnn_layer(inputs)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Output layer
        output = self.output(rnn_output)
        
        return output
    
class Model27(nn.Module):
    def __init__(self):
        super(Model27, self).__init__()
        
        # RNN
        self.rnn_layer = nn.LSTM(5, 32, 1, batch_first=True)
        
        # Output layer
        self.output = nn.Linear(32, 3)
        
        # # Initialize the weights using Xavier initialization
        # nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_normal_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        
        # RNN
        _, (rnn_output,_) = self.rnn_layer(inputs)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Output layer
        output = self.output(rnn_output)
        
        return output
    
class Model28(nn.Module):
    def __init__(self):
        super(Model28, self).__init__()
        
        # RNN
        self.rnn_layer = nn.GRU(5, 32, 1, batch_first=True)
        
        # Output layer
        self.output = nn.Linear(32, 3)
        
        # # Initialize the weights using Xavier initialization
        # nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_normal_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        
        # RNN
        _, rnn_output = self.rnn_layer(inputs)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Output layer
        output = self.output(rnn_output)
        
        return output
    
class Model29(nn.Module):
    def __init__(self):
        super(Model29, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 32)
        self.fc2 = nn.Linear(32, 32)
        
        # RNN
        self.rnn_layer = nn.RNN(32, 32, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(32, 32)
        self.fc1_reversed = nn.Linear(32, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        # nn.init.xavier_uniform_(self.fc1.weight)
        # nn.init.constant_(self.fc1.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc2.weight)
        # nn.init.constant_(self.fc2.bias, 0.0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_uniform_(self.fc2_reversed.weight)
        # nn.init.constant_(self.fc2_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc1_reversed.weight)
        # nn.init.constant_(self.fc1_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
                # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 32)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model30(nn.Module):
    def __init__(self):
        super(Model30, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 32)
        self.fc2 = nn.Linear(32, 32)
        
        # RNN
        self.rnn_layer = nn.LSTM(32, 32, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(32, 32)
        self.fc1_reversed = nn.Linear(32, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        # nn.init.xavier_uniform_(self.fc1.weight)
        # nn.init.constant_(self.fc1.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc2.weight)
        # nn.init.constant_(self.fc2.bias, 0.0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_uniform_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_uniform_(self.fc2_reversed.weight)
        # nn.init.constant_(self.fc2_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.fc1_reversed.weight)
        # nn.init.constant_(self.fc1_reversed.bias, 0.0)
        # nn.init.xavier_uniform_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
                # Initialize the weights using Xavier initialization
        # nn.init.xavier_normal_(self.fc1.weight)
        # nn.init.constant_(self.fc1.bias, 0.0)
        # nn.init.xavier_normal_(self.fc2.weight)
        # nn.init.constant_(self.fc2.bias, 0.0)
        # nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        # nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        # nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        # nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        # nn.init.xavier_normal_(self.fc2_reversed.weight)
        # nn.init.constant_(self.fc2_reversed.bias, 0.0)
        # nn.init.xavier_normal_(self.fc1_reversed.weight)
        # nn.init.constant_(self.fc1_reversed.bias, 0.0)
        # nn.init.xavier_normal_(self.output.weight)
        # nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 32)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, (rnn_output,_) = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model31(nn.Module):
    def __init__(self):
        super(Model31, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 16)
        self.fc2 = nn.Linear(16, 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(8, 16)
        self.fc1_reversed = nn.Linear(16, 5)
        
        # GELU
        self.RELU = nn.ReLU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.RELU(self.fc1(input1))
        x1 = self.RELU(self.fc2(x1))
        
        x2 = self.RELU(self.fc1(input2))
        x2 = self.RELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.RELU(self.fc2_reversed(rnn_output))
        reversed_output = self.RELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model32(nn.Module):
    def __init__(self):
        super(Model32, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 16)
        self.fc2 = nn.Linear(16, 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(8, 16)
        self.fc1_reversed = nn.Linear(16, 5)
        
        # GELU
        self.softplus = nn.Softplus()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.softplus(self.fc1(input1))
        x1 = self.softplus(self.fc2(x1))
        
        x2 = self.softplus(self.fc1(input2))
        x2 = self.softplus(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.softplus(self.fc2_reversed(rnn_output))
        reversed_output = self.softplus(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model33(nn.Module):
    def __init__(self):
        super(Model33, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 8)
        self.fc2 = nn.Linear(8, 16)
        
        # RNN
        self.rnn_layer = nn.RNN(16, 16, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(16, 8)
        self.fc1_reversed = nn.Linear(8, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.GELU(self.fc2(x1))
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.GELU(self.fc2(x2))
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 16)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.GELU(self.fc1_reversed(reversed_output))
        
        # Output layer
        output = self.output(reversed_output)
        
        return output
    
class Model34(nn.Module):
    def __init__(self):
        super(Model34, self).__init__()
        embedding_dim = 30
        
        # LSTM
        self.lstm_layer = nn.LSTM(5, embedding_dim, 1, batch_first=True)
        
        # MultiHead Attention
        self.attention = nn.MultiheadAttention(embedding_dim, 3, batch_first=True)
        
        # Reversed FC layers
        self.fc1 = nn.Linear(embedding_dim, embedding_dim)
        self.fc2 = nn.Linear(embedding_dim, embedding_dim)
        
        # Layer Norm
        self.norm1 = nn.LayerNorm(embedding_dim)
        # self.norm2 = nn.LayerNorm(embedding_dim)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(embedding_dim, 3)        
    
    def forward(self, input):
        # LSTM
        _,(h_n,c_n) = self.lstm_layer(input)
        h_n = torch.squeeze(h_n,0)
        c_n = torch.squeeze(c_n,0)
        
        # Attention
        out,_ = self.attention(h_n, c_n, c_n)
        
        # Add and Norm
        out = self.norm1(out + h_n)
        
        # Fully Connected
        fc_out = self.fc2(self.GELU(self.fc1(out)))
        
        # Add and Norm
        out = self.norm1(out + fc_out)
        
        # Linear
        output = self.output(out)
        
        return output
    
    #USAR SÓ ATTENTION
    
class Model35(nn.Module):
    def __init__(self):
        super(Model35, self).__init__()
        embedding_dim = 30
        
        # MLP
        self.fc_emb = nn.Linear(5, embedding_dim)
        
        # MultiHead Attention
        self.attention = nn.MultiheadAttention(embedding_dim, 3, batch_first=True)
        
        # Reversed FC layers
        self.fc1 = nn.Linear(embedding_dim, embedding_dim)
        self.fc2 = nn.Linear(embedding_dim, embedding_dim)
        
        # Layer Norm
        self.norm1 = nn.LayerNorm(embedding_dim)
        # self.norm2 = nn.LayerNorm(embedding_dim)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(embedding_dim, 3)        
    
    def forward(self, input):
        input1 = input[:, 0, :]
        input2 = input[:, 1, :]
        
        # MLP
        input1 = self.GELU(self.fc_emb(input1))
        input2 = self.GELU(self.fc_emb(input2))
        input1 = input1.unsqueeze(1)
        input2 = input2.unsqueeze(1)
        input = torch.cat((input1, input2), dim=1)
        
        # Attention
        out,_ = self.attention(input, input, input)
        
        # Add and Norm
        out = self.norm1(out + input)
        
        # Fully Connected
        fc_out1 = self.fc2(self.GELU(self.fc1(out[:,0,:])))
        fc_out2 = self.fc2(self.GELU(self.fc1(out[:,1,:])))
        fc_out1 = fc_out1.unsqueeze(1)
        fc_out2 = fc_out2.unsqueeze(1)
        fc_out = torch.cat((fc_out1, fc_out2), dim=1)
        
        # Add and Norm
        out = self.norm1(out + fc_out)
        
        # Linear
        output = self.output(out)
        output = output[:,1,:]
        
        return output
    
    
class Model36(nn.Module): # Not working - Transformer
    def __init__(self):
        super(Model37, self).__init__()
        self.embedding_dim = 32
        
        # MLP
        self.fc_emb = nn.Linear(5, self.embedding_dim)
        # GELU
        self.GELU = nn.GELU()
        
        # MultiHead Attention
        self.transformer = nn.Transformer(d_model=self.embedding_dim,nhead=1,num_encoder_layers=1,num_decoder_layers=1,dim_feedforward=self.embedding_dim, activation='gelu',batch_first=True)       
    
    def forward(self, input):
        input1 = input[:, 0, :]
        input2 = input[:, 1, :]
        
        # MLP
        input1 = self.GELU(self.fc_emb(input1))
        input2 = self.GELU(self.fc_emb(input2))
        input1 = input1.unsqueeze(1)
        input2 = input2.unsqueeze(1)
        input = torch.cat((input1, input2), dim=1)
        
        # Transformer
        tgt = np.zeros((input.shape[0],1,self.embedding_dim))
        output = self.transformer(input, tgt)
        
        return output    
    
class Model37(nn.Module): # Attention with RNN at the end
    def __init__(self):
        super(Model37, self).__init__()
        
        # MultiHead Attention
        self.attention = nn.MultiheadAttention(5, 5, batch_first=True)
        
        # RNN
        self.rnn_layer = nn.RNN(5, 16, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc1 = nn.Linear(16, 8)
        self.fc2 = nn.Linear(8, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)        
    
    def forward(self, input):
        # Attention
        out,_ = self.attention(input, input, input)
        
        # RNN
        _, rnn_output = self.rnn_layer(out)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Fully Connected
        out = self.GELU(self.fc2(self.GELU(self.fc1(rnn_output))))
        
        # Linear
        output = self.output(out)
        
        return output
    
class Model38(nn.Module): # Attention with RNN at the end
    def __init__(self):
        super(Model38, self).__init__()
        
        self.embedding_dim = 32
        # MLP
        self.fc_emb = nn.Linear(5, self.embedding_dim)
        
        # MultiHead Attention
        self.attention = nn.MultiheadAttention(self.embedding_dim, 1, batch_first=True)
        
        # RNN
        self.rnn_layer = nn.RNN(self.embedding_dim, self.embedding_dim, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc_dec = nn.Linear(self.embedding_dim, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)        
    
    def forward(self, input):
        input1 = input[:, 0, :]
        input2 = input[:, 1, :]
        
        # MLP
        input1 = self.GELU(self.fc_emb(input1))
        input2 = self.GELU(self.fc_emb(input2))
        input1 = input1.unsqueeze(1)
        input2 = input2.unsqueeze(1)
        input = torch.cat((input1, input2), dim=1)
        
        # Attention
        out,_ = self.attention(input, input, input)
        
        # RNN
        _, rnn_output = self.rnn_layer(out)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Fully Connected
        out = self.GELU(self.fc_dec(rnn_output))
        
        # Linear
        output = self.output(out)
        
        return output
    
class Model39(nn.Module):
    def __init__(self):
        super(Model39, self).__init__()
        
        input_shape = (2, 5)
        
        # Reshape the input
        self.fc1 = nn.Linear(input_shape[1], 16)
        self.fc2 = nn.Linear(16, 8)
        
        # RNN
        self.rnn_layer = nn.RNN(8, 8, 1, batch_first=True)
        
        # Reversed FC layers
        self.fc2_reversed = nn.Linear(8, 16)
        self.fc1_reversed = nn.Linear(16, 5)
        
        # GELU
        self.GELU = nn.GELU()
        
        # Output layer
        self.output = nn.Linear(5, 3)
        
        # # Initialize the weights using Xavier initialization
        nn.init.xavier_normal_(self.fc1.weight)
        nn.init.constant_(self.fc1.bias, 0.0)
        nn.init.xavier_normal_(self.fc2.weight)
        nn.init.constant_(self.fc2.bias, 0.0)
        nn.init.xavier_normal_(self.rnn_layer.weight_ih_l0)
        nn.init.xavier_normal_(self.rnn_layer.weight_hh_l0)
        nn.init.constant_(self.rnn_layer.bias_ih_l0, 0.0)
        nn.init.constant_(self.rnn_layer.bias_hh_l0, 0.0)
        nn.init.xavier_normal_(self.fc2_reversed.weight)
        nn.init.constant_(self.fc2_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.fc1_reversed.weight)
        nn.init.constant_(self.fc1_reversed.bias, 0.0)
        nn.init.xavier_normal_(self.output.weight)
        nn.init.constant_(self.output.bias, 0.0)
        
    
    def forward(self, inputs):
        input1 = inputs[:, 0, :]
        input2 = inputs[:, 1, :]
        
        x1 = self.GELU(self.fc1(input1))
        x1 = self.fc2(x1)
        
        x2 = self.GELU(self.fc1(input2))
        x2 = self.fc2(x2)
        
        # Concatenate inputs
        concat = torch.cat([x1, x2], dim=1)
        
        # Reshape
        concat = concat.view(concat.size(0), 2, 8)
        
        # RNN
        # self.rnn_layer.flatten_parameters()
        _, rnn_output = self.rnn_layer(concat)
        rnn_output = torch.squeeze(rnn_output,0)
        
        # Reversed FC layers
        reversed_output = self.GELU(self.fc2_reversed(rnn_output))
        reversed_output = self.fc1_reversed(reversed_output)
        
        # Output layer
        output = self.output(reversed_output)
        
        return output