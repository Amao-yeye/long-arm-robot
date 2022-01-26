%输入：坐标系i的x轴、z轴单位向量和坐标原点，坐标轴i+1的z轴单位向量和轴线上的任意一点
%输出：坐标系i+1的x轴、z轴单位向量和坐标原点，θ，d_x, d_z，α，β
function [e_z_i_1,e_x_i_1,o_i_1,theta_z,d_x,d_z,alpha_x,beita_y] = parallel_1(e_z_i,e_x_i,o_i,e_z,point_i_1)
e_z = array_one(e_z);
e_z_i = array_one(e_z_i);
e_x_i = array_one(e_x_i);
e_z_i_1 = e_z;
%用空间中直线的参数方程[x,y,z] = [x_0,y_0,z_0]+t*e_z，确定坐标系i+1的坐标原点
M = point_i_1-o_i;
t = (e_z_i(1)*M(1)+e_z_i(2)*M(2)+e_z_i(3)*M(3))/(e_z_i(1)*e_z(1)+e_z_i(2)*e_z(2)+e_z_i(3)*e_z(3));
o_i_1 = point_i_1 - t*e_z;
%建立过渡坐标轴
array= o_i_1-o_i;
e_x_temp = array_one(array);
e_y_temp = array_one(cross(e_z_i,e_x_temp));
%确定坐标系i+1的x轴、y轴单位向量
e_y_i_1 = array_one(cross(e_z_i_1,e_x_temp));
e_x_i_1 = array_one(cross(e_y_i_1,e_z_i_1));
%计算转角
alpha_x = rotate(e_y_temp,e_y_i_1,e_x_temp);
beita_y = rotate(e_x_temp,e_x_i_1,e_y_i_1);
theta_z = rotate(e_x_i,e_x_temp,e_z_i);
%计算距离
d_x = (array(1)^2+array(2)^2+array(3)^2)^0.5;
d_z = 0;
end
function gama = rotate(array_1,array_2,rotation_shaft)%array_1以rotation_shaft为转轴转到array_2所需的转动角度
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