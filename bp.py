import torch
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
import numpy as np


def network_train(CArray):
    # theta = torch.from_numpy(CArray[:243, 0:7]).to(torch.float32)
    theta = pd.read_excel("theta.xlsx")
    data230_0 = pd.read_excel("230data0.xlsx")
    data230_1 = pd.read_excel("230data1.xlsx")
    data290_0 = pd.read_excel("290data0.xlsx")
    data290_1 = pd.read_excel("290data1.xlsx")
    data344_0 = pd.read_excel("344data0.xlsx")
    data344_1 = pd.read_excel("344data1.xlsx")

    theta = theta.values
    data0 = np.concatenate(
        (data230_0.values, data290_0.values, data344_0.values), axis=0)
    data1 = np.concatenate(
        (data230_1.values, data290_1.values, data344_1.values), axis=0)
    x = (data0[:, 0] - data1[:, 0]) * 0.01
    y = (data0[:, 1] - data1[:, 1]) * 0.01
    z = (data0[:, 2] - data1[:, 2]) * 0.01

    print(x.size)

    theta = torch.from_numpy(theta).to(torch.float32)
    x = torch.from_numpy(x).to(torch.float32).unsqueeze(1)
    y = torch.from_numpy(y).to(torch.float32).unsqueeze(1)
    z = torch.from_numpy(z).to(torch.float32).unsqueeze(1)

    # print(x)
    # print(y)
    # print(z)

    # net_x = Net(7, 50, 1)
    # net_y = Net(7, 50, 1)
    # net_z = Net(7, 50, 1)
    net_x = torch.load('net_x.pkl')
    net_y = torch.load('net_y.pkl')
    net_z = torch.load('net_z.pkl')

    optimizer_x = torch.optim.SGD(net_x.parameters(), lr=0.001)
    optimizer_y = torch.optim.SGD(net_y.parameters(), lr=0.001)
    optimizer_z = torch.optim.SGD(net_z.parameters(), lr=0.001)
    loss_func = torch.nn.MSELoss()

    for t in range(100000):
        prediction_x = net_x(theta)
        prediction_y = net_y(theta)
        prediction_z = net_z(theta)
        loss_x = loss_func(prediction_x, x)
        loss_y = loss_func(prediction_y, y)
        loss_z = loss_func(prediction_z, z)
        optimizer_x.zero_grad()
        optimizer_y.zero_grad()
        optimizer_z.zero_grad()
        loss_x.backward()
        loss_y.backward()
        loss_z.backward()
        optimizer_x.step()
        optimizer_y.step()
        optimizer_z.step()
        if (t % 1000 == 0):
            print("enpoch: %d, loss_x: %.5f, loss_y: %.5f, loss_z: %.5f, " %
                  (t, loss_x, loss_y, loss_z))

    torch.save(net_x, 'net_x.pkl')  # 保存所有的网络参数
    torch.save(net_y, 'net_y.pkl')  # 保存所有的网络参数
    torch.save(net_z, 'net_z.pkl')  # 保存所有的网络参数


def network_caculate(thetaL):
    net_x = torch.load('net_x.pkl')
    net_y = torch.load('net_y.pkl')
    net_z = torch.load('net_z.pkl')
    # print("net")
    theta = torch.from_numpy(thetaL[0, :]).to(torch.float32)
    # print(theta)
    x = net_x(theta)
    y = net_y(theta)
    z = net_z(theta)
    # print("x: %.5f, y: %.5f, z: %.5f" % (x, y, z))
    return [x, y, z]


class Net(nn.Module):
    def __init__(self, n_input, n_hidden, n_output):
        super(Net, self).__init__()
        self.hidden1 = nn.Linear(n_input, n_hidden)
        self.hidden2 = nn.Linear(n_hidden, n_hidden)
        self.predict1 = nn.Linear(n_hidden, n_output)

    def forward(self, input):
        out1 = self.hidden1(input)
        out1 = F.relu(out1)
        out1 = self.hidden2(out1)
        out1 = F.relu(out1)
        out1 = self.predict1(out1)
        return out1
