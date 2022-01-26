% ��ջ���
testdata=xlsread('C:\Users\YING\Desktop\���ӶȻ�е������\��ʵ���ݺ���������\ѵ����3.xlsx');

%��ȡ����
%load data input output
input=testdata(:,1:7);
output1=testdata(:,14);%errx
output2=testdata(:,15);%erry
output3=testdata(:,16);%errz

%ѡ�����˭
output=output3;
[m,n]=size(input);
[mm,nn]=size(output);
%�ڵ����
inputnum=n;
outputnum=nn;
%������ڵ�������鹫ʽhiddennum=sqrt(m+n)+a��mΪ�����ڵ������nΪ�����ڵ������aһ��ȡΪ1-10֮�������
%hiddennum=fix(sqrt(inputnum+outputnum))+1;
hiddennum=3;

%ѵ�����ݺ�Ԥ������
input_train=input(400:549,:)';
input_test=input(1:40,:)';
output_train=output(400:549)';
output_test=output(1:40)';

%ѡ����������������ݹ�һ��: �������ÿһ�д����[-1,1]����
[inputn,inputps]=mapminmax(input_train);
[outputn,outputps]=mapminmax(output_train);

%��������
net=newff(inputn,outputn,hiddennum);

% ������ʼ��
dim=30;
maxgen=30;   % ��������  
sizepop=30;   %��Ⱥ��ģ

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%��ȸ�����㷨 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% ����Ѱ��
x=bestX
%% �������
plot(yy)
title(['��Ӧ������  ' '��ֹ������' num2str(maxgen)]);
xlabel('��������');ylabel('��Ӧ��');
%% �����ų�ʼ��ֵȨֵ��������Ԥ��
% %����ȸ�����㷨�Ż���BP�������ֵԤ��
w1=x(1:inputnum*hiddennum);
B1=x(inputnum*hiddennum+1:inputnum*hiddennum+hiddennum);
w2=x(inputnum*hiddennum+hiddennum+1:inputnum*hiddennum+hiddennum+hiddennum*outputnum);
B2=x(inputnum*hiddennum+hiddennum+hiddennum*outputnum+1:inputnum*hiddennum+hiddennum+hiddennum*outputnum+outputnum);

net.iw{1,1}=reshape(w1,hiddennum,inputnum);
net.lw{2,1}=reshape(w2,outputnum,hiddennum);
net.b{1}=reshape(B1,hiddennum,1);
net.b{2}=B2;

%% ѵ��
%�����������
net.trainParam.epochs=100;
net.trainParam.lr=0.1;
net.trainParam.goal=0.00001;

%����ѵ��
[net,tr]=train(net,inputn,outputn);

%%Ԥ��
%���ݹ�һ��
inputn_test=mapminmax('apply',input_test,inputps);
an=sim(net,inputn_test);
test_simu=mapminmax('reverse',an,outputps);
error=test_simu-output_test;
figure(2)
plot(error)
title('����Ԥ�����','fontsize',12);
xlabel('�������','fontsize',12);ylabel('���ٷ�ֵ','fontsize',12);


%%����
%save(['C:\Users\YING\Desktop\���洴�¿�--���ӶȻ�е��\�����粹��' 'mynet.mat'], 'net');
%load ('C:\Users\YING\Desktop\���洴�¿�--���ӶȻ�е��\�����粹��\mynet.mat')
%===========������=====================
errorPercent=abs(error)./output_test;   %���������� ��
[c,l]=size(output_test);            %%������Լ���������l
% ����ϵ��R^2
R2 = (l* sum(test_simu .* output_test) - sum(test_simu) * sum(output_test))^2 / ((l * sum((test_simu).^2) - (sum(test_simu))^2) * (l * sum((output_test).^2) - (sum(output_test))^2)); 
%% ����Ԥ��ͼ��
%% ����ֵ��Ԥ��ֵ���Ƚ�
figure('name','���Լ�Ԥ��ֵ��ʵ��ֵ�ĶԱ�ͼ')
plot(output_test,'ro-')
mydaini=output_test
hold on
plot(test_simu,'b*-')
mynihe=test_simu
title('���Լ�Ԥ��ֵ��ʵ��ֵ�ĶԱ�','fontsize',12)
xlim([1 l])   %����x�᷶Χ
legend('ʵ��ֵ','Ԥ��ֵ')
xlabel('��������')
ylabel('z����ƫ��')

figure('name','���ͼ')
plot(error,'b*-')
title('SSA-BP������Ԥ��ֵ��ʵ��ֵ�����ͼ','fontsize',12)
xlim([1 l])   %����x�᷶Χ
xlabel('�������')
ylabel('���')
%������

MAE1=sum(abs(error))/l;
MSE1=error*error'/l;
RMSE1=MSE1^(1/2);
errorPmin=min(errorPercent);
errorPmax=max(errorPercent);
errorPav=sum(errorPercent)/l;
%������
disp(['-----------------------������--------------------------'])
disp(['ƽ���������MAEΪ��',num2str(MAE1)])
disp(['�������MSEΪ��       ',num2str(MSE1)])
disp(['���������RMSEΪ��  ',num2str(RMSE1)])
disp(['��С������Ϊ��       ',num2str(errorPmin)])
disp(['���������Ϊ��        ',num2str(errorPmax)])
disp(['ƽ��������Ϊ��        ',num2str(errorPav)])
disp(['����ϵ��R��Ϊ��         ',num2str(R2)])
