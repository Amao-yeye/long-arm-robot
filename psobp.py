import numpy as np
import random
import matplotlib.pyplot as plt
import neurolab as nl
import torch
import torch.nn.functional as F
from torch.autograd import Variable


class PSO():
    def __init__(self, max_iter):
        #self.w = 0.8
        self.c1 = 2
        self.c2 = 2
        self.pN = 10  #粒子数量
        self.dim = 1  #搜索维度
        self.max_iter = max_iter  #迭代次数
        self.X = np.ones((self.pN, self.dim))  #所有粒子的位置和速度
        self.V = np.zeros((self.pN, self.dim))
        self.pbest = np.zeros((self.pN, self.dim))  #个体经历的最佳位置和全局最佳位置
        self.gbest = np.zeros((1, self.dim))
        self.p_fit = np.zeros(self.pN)  #每个个体的历史最佳适应值
        self.fit = 1e10  #全局最佳适应值
        self.wmax = 0.9
        self.wmin = 0.4

    #目标函数
    def fun(self, err):
        fitness = err
        return fitness

    #初始化种群
    def init_Population(self, myinput, mytarget):
        for i in range(self.pN):
            for j in range(self.dim):
                self.X[i][j] = random.uniform(1, 20)
                self.V[i][j] = random.uniform(0, 2)
            self.pbest[i] = self.X[i]

            for x in self.pbest[i]:
                return x
            bpnet = nl.net.newff([[-2 * np.pi, 2 * np.pi], [0, 1], [0, 1],
                                  [0, 1], [0, 1], [0, 1], [0, 1]],
                                 [int(x) + 1, 1])
            err = bpnet.train(myinput,
                              mytarget,
                              epochs=800,
                              show=100,
                              goal=0.02)
            #out=net.sim(input)
            tmp = self.fun(err)
            self.p_fit[i] = tmp
            if (tmp < self.fit):
                self.fit = tmp
                self.gbest = self.X[i]

    #更新粒子位置
    def iterator(self, myinput, mytarget):
        fitness = []
        for t in range(self.max_iter):
            w = self.wmax - (self.wmax - self.wmin) * (float(t) /
                                                       self.max_iter)
            for i in range(self.pN):
                for x in self.pbest[i]:
                    return x
                print(x)
                bpnet = nl.net.newff([[-2 * np.pi, 2 * np.pi], [0, 1], [0, 1],
                                      [0, 1], [0, 1], [0, 1], [0, 1]],
                                     [int(x) + 1, 1])
                err = bpnet.train(myinput,
                                  mytarget,
                                  epochs=800,
                                  show=100,
                                  goal=0.02)
                temp = self.fun(err)
                if (temp < self.p_fit[i]):  #更新个体最优
                    self.p_fit[i] = temp
                    self.pbest[i] = self.X[i]
                    if (self.p_fit[i] < self.fit):  #更新全局最优
                        self.gbest = self.X[i]
                        self.fit = self.p_fit[i]
            for i in range(self.pN):
                self.V[i] = w*self.V[i] + self.c1*np.random.uniform(0,1)*(self.pbest[i] - self.X[i])\
                       + self.c2*np.random.uniform(0,1)*(self.gbest - self.X[i])
                self.X[i] = self.X[i] + self.V[i]
            fitness.append(self.fit)
            #print(self.fit)                   #输出最优值
        return x


def network_train(CArray):
    print("network train")
    train_x = []
    d = []
    samplescount = 1000
    myrndsmp = np.random.rand(samplescount)
    #train
    for yb_i in range(0, samplescount):
        train_x.append(
            [myrndsmp[yb_i] * 4 * np.pi, 0.5, 0.5, 0.5, 0.5, 0.5, 0.433])
        d = np.random.uniform(0, 0.1, (1000, 1))
    myinput = np.array(train_x)
    mytarget = np.array(d)
    # #PSO参数设置
    # #-程序执行
    # my_pso = PSO(max_iter=100)
    # print("init_Population")
    # my_pso.init_Population(myinput, mytarget)
    # print("iterator")
    # x = my_pso.iterator(myinput, mytarget)
    # print(int(x) + 1)
    # a = myinput.max(axis=0)
    # b = myinput.min(axis=0)
    # c = mytarget.max(axis=0)
    # d = mytarget.min(axis=0)

    #bpnet = nl.net.newff([[a[0], b[0]],[a[1], b[1]],[a[2], b[2]], [a[3], b[3]],[a[4], b[4]],[a[5], b[5]],[a[6], b[6]]], [c,d])
    bpnet = nl.net.newff(
        [[0, 1], [0, 1], [0, 1], [0, 1], [0, 1], [0, 1], [0, 1]], [3, 1])
    bpnet.trainf = nl.train.train_gd
    print("net")
    err = bpnet.train(myinput, mytarget, epochs=500, show=10, goal=0.02)
    print("err")
    #误差曲线
    plt.title("pso-bp")
    plt.plot(range(len(err)), err)
    plt.xlabel('Epoch number')
    plt.ylabel('err (default SSE)')
    #可视化图
    plt.show()
    torch.save(bpnet, 'net.pkl')  # 保存整个网络


def network_caculate(thetaL):
    x = thetaL
    #再次使用，提取网络
    net2 = torch.load('net.pkl')
    prediction = net2(x)
    return prediction
