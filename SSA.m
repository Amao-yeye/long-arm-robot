% 清空环境
testdata=xlsread('C:\Users\YING\Desktop\大挠度机械臂误差补偿\真实数据和理想数据\训练集3.xlsx');

%读取数据
%load data input output
input=testdata(:,1:7);
output1=testdata(:,14);%errx
output2=testdata(:,15);%erry
output3=testdata(:,16);%errz

%选择拟合谁
output=output3;
[m,n]=size(input);
[mm,nn]=size(output);
%节点个数
inputnum=n;
outputnum=nn;
%隐含层节点个数经验公式hiddennum=sqrt(m+n)+a，m为输入层节点个数，n为输出层节点个数，a一般取为1-10之间的整数
%hiddennum=fix(sqrt(inputnum+outputnum))+1;
hiddennum=3;

%训练数据和预测数据
input_train=input(400:549,:)';
input_test=input(1:40,:)';
output_train=output(400:549)';
output_test=output(1:40)';

%选连样本输入输出数据归一化: 将矩阵的每一行处理成[-1,1]区间
[inputn,inputps]=mapminmax(input_train);
[outputn,outputps]=mapminmax(output_train);

%构建网络
net=newff(inputn,outputn,hiddennum);

% 参数初始化
dim=30;
maxgen=30;   % 进化次数  
sizepop=30;   %种群规模

popmax= 5;
popmin=-5;
P_percent = 0.2;    % The population size of producers accounts for "P_percent" percent of the total population size       
pNum = round( sizepop *  P_percent );    % The population size of the producers   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:sizepop
    pop(i,:)=7*rands(1,30);
%     V(i,:)=rands(1,21);
    fitness(i)=fun(pop(i,:),inputnum,hiddennum,outputnum,net,inputn,outputn); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pFit = fitness;                      
[ fMin, bestI ] = min( fitness );      % fMin denotes the global optimum fitness value
bestX = pop( bestI, : );             % bestX denotes the global optimum position corresponding to fMin

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%麻雀搜索算法 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t = 1 : maxgen 
  [ ans, sortIndex ] = sort( pFit );% Sort.
  [fmax,B]=max( pFit );
   worse= pop(B,:);      
   r2=rand(1);
if(r2<0.8)
    for i = 1 : pNum                                                   % Equation (3)
        r1=rand(1);
        pop( sortIndex( i ), : ) = pop( sortIndex( i ), : )*exp(-(i)/(r1*maxgen));
        fitness(sortIndex( i ))=fun(pop(sortIndex( i ),:),inputnum,hiddennum,outputnum,net,inputn,outputn); 
    end
  else
  for i = 1 : pNum         
        pop( sortIndex( i ), : ) = pop( sortIndex( i ), : )+randn(1)*ones(1,dim);
        fitness(sortIndex( i ))=fun(pop(sortIndex( i ),:),inputnum,hiddennum,outputnum,net,inputn,outputn);   
  end      
end

[ fMMin, bestII ] = min( fitness );      
bestXX = pop( bestII, : ); 

for i = ( pNum + 1 ) : sizepop% Equation (4)
    A=floor(rand(1,dim)*2)*2-1;
    if( i>(sizepop/2))
        pop( sortIndex(i ), : )=randn(1)*exp((worse-pop( sortIndex( i ), : ))/(i)^2);
    else
        pop( sortIndex( i ), : )=bestXX+(abs(( pop( sortIndex( i ), : )-bestXX)))*(A'*(A*A')^(-1))*ones(1,dim);  
    end   
        fitness(sortIndex( i ))=fun(pop(sortIndex( i ),:),inputnum,hiddennum,outputnum,net,inputn,outputn);      
end
   c=randperm(numel(sortIndex));
   b=sortIndex(c(1:3));
   
for j =  1  : length(b)% Equation (5)
    if( pFit( sortIndex( b(j) ) )>(fMin) )
        pop( sortIndex( b(j) ), : )=bestX+(randn(1,dim)).*(abs(( pop( sortIndex( b(j) ), : ) -bestX)));
    else
         pop( sortIndex( b(j) ), : ) =pop( sortIndex( b(j) ), : )+(2*rand(1)-1)*(abs(pop( sortIndex( b(j) ), : )-worse))/ ( pFit( sortIndex( b(j) ) )-fmax+1e-50);
    end   
       fitness(sortIndex( i ))=fun(pop(sortIndex( i ),:),inputnum,hiddennum,outputnum,net,inputn,outputn);
    end
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for i = 1 : sizepop 
        if ( fitness( i ) < pFit( i ) )
            pFit( i ) = fitness( i );
             pop(i,:) = pop(i,:);
        end
        
        if( pFit( i ) < fMin )
           fMin= pFit( i );
            bestX =pop( i, : );
         
            
        end
    end
    yy(t)=fMin;    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 迭代寻优
x=bestX
%% 结果分析
plot(yy)
title(['适应度曲线  ' '终止代数＝' num2str(maxgen)]);
xlabel('进化代数');ylabel('适应度');
%% 把最优初始阀值权值赋予网络预测
% %用麻雀搜索算法优化的BP网络进行值预测
w1=x(1:inputnum*hiddennum);
B1=x(inputnum*hiddennum+1:inputnum*hiddennum+hiddennum);
w2=x(inputnum*hiddennum+hiddennum+1:inputnum*hiddennum+hiddennum+hiddennum*outputnum);
B2=x(inputnum*hiddennum+hiddennum+hiddennum*outputnum+1:inputnum*hiddennum+hiddennum+hiddennum*outputnum+outputnum);

net.iw{1,1}=reshape(w1,hiddennum,inputnum);
net.lw{2,1}=reshape(w2,outputnum,hiddennum);
net.b{1}=reshape(B1,hiddennum,1);
net.b{2}=B2;

%% 训练
%网络进化参数
net.trainParam.epochs=100;
net.trainParam.lr=0.1;
net.trainParam.goal=0.00001;

%网络训练
[net,tr]=train(net,inputn,outputn);

%%预测
%数据归一化
inputn_test=mapminmax('apply',input_test,inputps);
an=sim(net,inputn_test);
test_simu=mapminmax('reverse',an,outputps);
error=test_simu-output_test;
figure(2)
plot(error)
title('仿真预测误差','fontsize',12);
xlabel('仿真次数','fontsize',12);ylabel('误差百分值','fontsize',12);


%%保存
%save(['C:\Users\YING\Desktop\交叉创新课--大挠度机械臂\神经网络补偿' 'mynet.mat'], 'net');
%load ('C:\Users\YING\Desktop\交叉创新课--大挠度机械臂\神经网络补偿\mynet.mat')
%===========误差分析=====================
errorPercent=abs(error)./output_test;   %计算相对误差 ‘
[c,l]=size(output_test);            %%计算测试集样本总数l
% 决定系数R^2
R2 = (l* sum(test_simu .* output_test) - sum(test_simu) * sum(output_test))^2 / ((l * sum((test_simu).^2) - (sum(test_simu))^2) * (l * sum((output_test).^2) - (sum(output_test))^2)); 
%% 网络预测图形
%% 期望值与预测值误差比较
figure('name','测试集预测值与实际值的对比图')
plot(output_test,'ro-')
mydaini=output_test
hold on
plot(test_simu,'b*-')
mynihe=test_simu
title('测试集预测值与实际值的对比','fontsize',12)
xlim([1 l])   %限制x轴范围
legend('实际值','预测值')
xlabel('测试样本')
ylabel('z方向偏差')

figure('name','误差图')
plot(error,'b*-')
title('SSA-BP神经网络预测值和实际值的误差图','fontsize',12)
xlim([1 l])   %限制x轴范围
xlabel('样本编号')
ylabel('误差')
%误差分析

MAE1=sum(abs(error))/l;
MSE1=error*error'/l;
RMSE1=MSE1^(1/2);
errorPmin=min(errorPercent);
errorPmax=max(errorPercent);
errorPav=sum(errorPercent)/l;
%输出结果
disp(['-----------------------误差计算--------------------------'])
disp(['平均绝对误差MAE为：',num2str(MAE1)])
disp(['均方误差MSE为：       ',num2str(MSE1)])
disp(['均方根误差RMSE为：  ',num2str(RMSE1)])
disp(['最小相对误差为：       ',num2str(errorPmin)])
disp(['最大相对误差为：        ',num2str(errorPmax)])
disp(['平均相对误差为：        ',num2str(errorPav)])
disp(['决定系数R方为：         ',num2str(R2)])
