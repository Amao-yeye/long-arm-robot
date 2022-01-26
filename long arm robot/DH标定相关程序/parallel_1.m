%���룺����ϵi��x�ᡢz�ᵥλ����������ԭ�㣬������i+1��z�ᵥλ�����������ϵ�����һ��
%���������ϵi+1��x�ᡢz�ᵥλ����������ԭ�㣬�ȣ�d_x, d_z��������
function [e_z_i_1,e_x_i_1,o_i_1,theta_z,d_x,d_z,alpha_x,beita_y] = parallel_1(e_z_i,e_x_i,o_i,e_z,point_i_1)
e_z = array_one(e_z);
e_z_i = array_one(e_z_i);
e_x_i = array_one(e_x_i);
e_z_i_1 = e_z;
%�ÿռ���ֱ�ߵĲ�������[x,y,z] = [x_0,y_0,z_0]+t*e_z��ȷ������ϵi+1������ԭ��
M = point_i_1-o_i;
t = (e_z_i(1)*M(1)+e_z_i(2)*M(2)+e_z_i(3)*M(3))/(e_z_i(1)*e_z(1)+e_z_i(2)*e_z(2)+e_z_i(3)*e_z(3));
o_i_1 = point_i_1 - t*e_z;
%��������������
array= o_i_1-o_i;
e_x_temp = array_one(array);
e_y_temp = array_one(cross(e_z_i,e_x_temp));
%ȷ������ϵi+1��x�ᡢy�ᵥλ����
e_y_i_1 = array_one(cross(e_z_i_1,e_x_temp));
e_x_i_1 = array_one(cross(e_y_i_1,e_z_i_1));
%����ת��
alpha_x = rotate(e_y_temp,e_y_i_1,e_x_temp);
beita_y = rotate(e_x_temp,e_x_i_1,e_y_i_1);
theta_z = rotate(e_x_i,e_x_temp,e_z_i);
%�������
d_x = (array(1)^2+array(2)^2+array(3)^2)^0.5;
d_z = 0;
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