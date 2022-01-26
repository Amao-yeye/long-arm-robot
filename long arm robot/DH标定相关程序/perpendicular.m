%���룺����ϵi��x�ᡢz�ᵥλ����������ԭ�㣬������i+1��z�ᵥλ�����������ϵ�����һ��
%���������ϵi+1��x�ᡢz�ᵥλ����������ԭ�㣬�ȣ�d_x, d_z��������
function [e_z_i_1,e_x_i_1,o_i_1,theta_z,d_x,d_z,alpha_x,beita_y,o_temp] = perpendicular(e_z_i,e_x_i,o_i,e_z,point_i_1)
e_z = array_one(e_z);
e_z_i = array_one(e_z_i);
e_x_i = array_one(e_x_i);
e_z_i_1 = e_z;
[o_i_1,o_temp] = two_points(e_z_i,o_i,e_z_i_1,point_i_1);%ȷ������ϵi+1������ԭ��
e_x_i_1 = array_one(o_i_1-o_temp);%ȷ������ϵi+1��x�ᵥλ����
%����ת��
alpha_x = rotate(e_z_i,e_z_i_1,e_x_i_1);
beita_y = 0;
theta_z = rotate(e_x_i,e_x_i_1,e_z_i);
%�������
array = o_temp-o_i_1;
d_x = (array(1)^2+array(2)^2+array(3)^2)^0.5;
array = o_temp-o_i;
d_z = (array(1)^2+array(2)^2+array(3)^2)^0.5;
if array*e_z_i' <=0
    d_z = -d_z;
end
end
function [o_1,o_2]=two_points(e_1,p_1,e_2,p_2)%���ݿռ��е�������ƽ��ֱ��ȷ�������ߣ������ع�����������ֱ�ߵ���������
A = p_1;
C = p_2;
H=e_1(1);
I = e_1(2);
J = e_1(3);
K = e_2(1);
L = e_2(2);
M = e_2(3);
N = H*I*L-I*I*K-J*J*K+H*J*M;
O = H*H*L-H*I*K-I*J*M+J*J*L;
P = H*J*K-H*H*M-I*I*M+I*J*L;
Q = -A(1)*N+A(2)*O-A(3)*P;
k = ( O*C(2)-N*C(1)-P*C(3)-Q)/( N*K-O*L+P*M);
o_1=[K*k+C(1),L*k+C(2),M*k+C(3)];
A = p_2;
C = p_1;
H=e_2(1);
I = e_2(2);
J = e_2(3);
K = e_1(1);
L = e_1(2);
M = e_1(3);
N = H*I*L-I*I*K-J*J*K+H*J*M;
O = H*H*L-H*I*K-I*J*M+J*J*L;
P = H*J*K-H*H*M-I*I*M+I*J*L;
Q = -A(1)*N+A(2)*O-A(3)*P;
k = ( O*C(2)-N*C(1)-P*C(3)-Q)/( N*K-O*L+P*M);
o_2=[K*k+C(1),L*k+C(2),M*k+C(3)];
end
function gama = rotate(array_1,array_2,rotation_shaft)%array_1��rotation_shaftΪת��ת��array_2�����ת���Ƕ�
array_1=array_one(array_1);
array_2=array_one(array_2);
rotation_shaft=array_one(rotation_shaft);
E = array_one(cross(array_1,array_2));
if dot(array_1,array_2)>=1
    gama=0;
else
    gama = acos(dot(array_1,array_2));
end
if dot(rotation_shaft,E)<0
    gama = -1*gama;
end
end
function E = array_one(array)
L = (array(1)^2+array(2)^2+array(3)^2)^0.5;
E = array./L;
end
