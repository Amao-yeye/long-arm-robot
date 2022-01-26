#include "deflection.h"

using namespace std;

Robot::Robot() : DH(7, 4), Jacob_final(6, 7), Jacob_deflection(6, 7), T_final(56, 4), T_deflection(56, 4), current_theta(7), F_end(3), Me_end(3)
{
    // T_final用于存放未计算挠度时的关节角数据，并且每个关节a和d均记录一个矩阵
    // T_final用于存放计算挠度之后的关节角数据，并且每个关节a和d均记录一个矩阵
    // 格式为(4x7x2)x4，其中4x4为位姿矩阵，7为七个关节包含末端，先存放经过长度a，再存放经过长度d
    current_theta << 0.0, 90.0, 2.30, 0.0, 0.0, 0.0, 0.0;
    current_theta = current_theta / 180 * M_PI;
    current_theta(2) = current_theta(2) * 180 / M_PI;
    F_end << 0.0, 0.0, 0.0;
    Me_end << 0.0, 0.0, 0.0;
    forward_deflection_caculate(current_theta);
    // python初始化，可视化界面需要
    Py_Initialize();
    if (_import_array() < 0)
    {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
    }
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('./src')");
    if (!Py_IsInitialized())
        printf("init failed/n");
};

Robot::~Robot()
{
    //python结束
    Py_Finalize();
}

void Robot::forward_kinematics_caculate(const Eigen::VectorXd &thetaL)
{
    double theta1 = -thetaL(0);
    double theta2 = -thetaL(1) + M_PI;
    double d3 = thetaL(2) + L6;
    double theta4 = -thetaL(3) + M_PI;
    double theta5 = thetaL(4);
    double theta6 = thetaL(5);
    double d7 = thetaL(6) + L10;

    DH << 0, 0, theta1, L5,
        L4, M_PI / 2.0, theta2, 0,
        0, M_PI / 2.0, 0, d3,
        0, 0, theta4, 0,
        0, -M_PI / 2.0, theta5, -L7,
        L8, -M_PI / 2.0, theta6, -L9,
        d7, 0, 0, 0;
    //cout << "    DH finished" << endl;

    Eigen::VectorXd a = DH.col(0);
    Eigen::VectorXd alpha = DH.col(1);
    Eigen::VectorXd theta = DH.col(2);
    Eigen::VectorXd d = DH.col(3);

    Eigen::Matrix4d T;
    T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Matrix4d T_temp1;
    Eigen::Matrix4d T_temp2;
    Eigen::Matrix4d T_temp[7];
    Eigen::Matrix4d T_joint[14];
    for (int i = 0; i < 7; i++)
    {
        //先计算a，即机械臂中间节点
        T_temp1 << 1, 0, 0, a(i),
            0, cos(alpha(i)), -sin(alpha(i)), 0,
            0, sin(alpha(i)), cos(alpha(i)), 0,
            0, 0, 0, 1;
        T = T * T_temp1;
        T_joint[i * 2] = T;

        //先计算d，即机械臂实际关节点
        T_temp2 << cos(theta(i)), -sin(theta(i)), 0, 0,
            sin(theta(i)), cos(theta(i)), 0, 0,
            0, 0, 1, d(i),
            0, 0, 0, 1;
        T = T * T_temp2;
        T_joint[i * 2 + 1] = T;
        T_temp[i] = T_temp1 * T_temp2;
    }
    //cout << "    T_joint finished" << endl;

    Jacob_final = JacobSpace(T_temp[0], T_temp[1], T_temp[2], T_temp[3], T_temp[4], T_temp[5], T_temp[6]);
    T07_final = T_joint[13];
    T_final << T_joint[0],
        T_joint[1],
        T_joint[2],
        T_joint[3],
        T_joint[4],
        T_joint[5],
        T_joint[6],
        T_joint[7],
        T_joint[8],
        T_joint[9],
        T_joint[10],
        T_joint[11],
        T_joint[12],
        T_joint[13];
    //cout << "    T_final finished" << endl;

    return;
}

void Robot::forward_deflection_caculate(const Eigen::VectorXd &thetaL)
{
    forward_kinematics_caculate(thetaL);
    Eigen::VectorXd a = DH.col(0);
    Eigen::VectorXd alpha = DH.col(1);
    Eigen::VectorXd theta = DH.col(2);
    Eigen::VectorXd d = DH.col(3);

    Eigen::Matrix4d T;
    T << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Vector3d V_L;
    Eigen::Vector3d V_F;
    Eigen::Vector3d V_Me;
    Eigen::Vector3d V_G;
    Eigen::Vector3d V_omega; //挠度偏移方向（关节坐标下）
    Eigen::Vector3d V_theta; //转角旋转方向（关节坐标下）
    Eigen::Matrix4d T_temp1;
    Eigen::Matrix4d T_temp2;
    Eigen::Matrix4d T_joint[14];
    Eigen::Matrix4d T_temp[7];

    Eigen::Vector3d F_joint[14];
    Eigen::Vector3d Me_joint[14];
    Eigen::Vector3d F_temp;
    Eigen::Vector3d Me_temp;
    Eigen::Vector3d P_temp;

    double L;
    Eigen::Vector3d F;
    Eigen::Vector3d G;
    Eigen::Vector3d Me;
    double R_theta;
    double kx, ky, kz, ctheta, stheta, vtheta;

    //cout << "    F_joint Me_joint caculate start" << endl;
    F_joint[13] = F_end;
    Me_joint[13] = Me_end;
    //cout << "    13";
    for (int i = 12; i >= 0; i--)
    {
        P_temp << T_final(4 * i + 4, 3) - T_final(4 * i + 0, 3),
            T_final(4 * i + 5, 3) - T_final(4 * i + 1, 3),
            T_final(4 * i + 6, 3) - T_final(4 * i + 2, 3);

        F_temp << 0, 0, -P_temp.norm() * density * g;
        Me_temp << P_temp.cross(F_temp) / 2 + P_temp.cross(F_joint[i + 1]);

        F_joint[i] = F_joint[i + 1] + F_temp;
        Me_joint[i] = Me_joint[i + 1] + Me_temp;
        //cout << "    " << i;
    }
    //cout << "\n    F_joint Me_joint finished" << endl;

    //cout << "    T_joint caculate start" << endl;
    for (int i = 0; i < 7; i++)
    {
        //计算长度 a 的挠度和转角
        V_L << 1, 0, 0;
        V_F = T.block(0, 0, 3, 3).inverse() * F_joint[i * 2];
        V_Me = T.block(0, 0, 3, 3).inverse() * Me_joint[i * 2];
        V_G = -T.block(0, 0, 3, 3).inverse().block(0, 2, 3, 1);

        L = a(i);
        F = V_F - V_L * V_L.dot(V_F);
        G = g * V_G;
        Me = V_Me.cross(V_L);
        V_omega = F * L * L * L / 3 / EI + Me * L * L / 2 / EI + density * G * L * L * L * L / 8 / EI;
        V_theta = F * L * L * L / 2 / EI + Me * L / EI + density * G * L * L * L / 6 / EI;
        V_theta = V_L.cross(V_theta);
        R_theta = V_theta.norm();

        if (V_theta.norm() == 0)
            V_theta << 0, 1, 0;
        else
            V_theta = V_theta / V_theta.norm();

        kx = V_theta(0);
        ky = V_theta(1);
        kz = V_theta(2);
        ctheta = cos(R_theta);
        stheta = sin(R_theta);
        vtheta = 1 - cos(R_theta);

        T_temp1 << 1, 0, 0, 0,
            0, cos(alpha(i)), -sin(alpha(i)), 0,
            0, sin(alpha(i)), cos(alpha(i)), 0,
            0, 0, 0, 1;
        T_temp2 << kx * kx * vtheta + ctheta, kx * ky * vtheta - kz * stheta, kx * kz * vtheta + ky * stheta, a(i) + V_omega(0),
            kx * ky * vtheta + kz * stheta, ky * ky * vtheta + ctheta, ky * kz * vtheta - kx * stheta, V_omega(1),
            kx * kz * vtheta - ky * stheta, ky * kz * vtheta + kx * stheta, kz * kz * vtheta + ctheta, V_omega(2),
            0, 0, 0, 1;

        T = T * T_temp2 * T_temp1;
        T_joint[i * 2] = T;
        T_temp[i] = T_temp2 * T_temp1;

        //cout << "    a" << i;

        //计算长度 d 的挠度和转角
        V_L << 0, 0, 1;
        V_F = T.block(0, 0, 3, 3).inverse() * F_joint[i * 2 + 1];
        V_Me = T.block(0, 0, 3, 3).inverse() * Me_joint[i * 2 + 1];
        V_G = -T.block(0, 0, 3, 3).inverse().block(0, 2, 3, 1);

        L = d(i);
        F = V_F - V_L * V_L.dot(V_F);
        G = g * V_G;
        Me = V_Me.cross(V_L);
        V_omega = F * L * L * L / 3 / EI + Me * L * L / 2 / EI + density * G * L * L * L * L / 8 / EI;
        V_theta = F * L * L * L / 2 / EI + Me * L / EI + density * G * L * L * L / 6 / EI;
        V_theta = V_L.cross(V_theta);
        R_theta = V_theta.norm();

        if (V_theta.norm() == 0)
            V_theta << 0, 1, 0;
        else
            V_theta = V_theta / V_theta.norm();

        kx = V_theta(0);
        ky = V_theta(1);
        kz = V_theta(2);
        ctheta = cos(R_theta);
        stheta = sin(R_theta);
        vtheta = 1 - cos(R_theta);

        T_temp1 << cos(theta(i)), -sin(theta(i)), 0, 0,
            sin(theta(i)), cos(theta(i)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        T_temp2 << kx * kx * vtheta + ctheta, kx * ky * vtheta - kz * stheta, kx * kz * vtheta + ky * stheta, V_omega(0),
            kx * ky * vtheta + kz * stheta, ky * ky * vtheta + ctheta, ky * kz * vtheta - kx * stheta, V_omega(1),
            kx * kz * vtheta - ky * stheta, ky * kz * vtheta + kx * stheta, kz * kz * vtheta + ctheta, d(i) + V_omega(2),
            0, 0, 0, 1;

        T = T * T_temp2 * T_temp1;
        T_joint[i * 2 + 1] = T;
        T_temp[i] = T_temp[i] * T_temp2 * T_temp1;

        //cout << "    d" << i;
    }
    //cout << "\n    T_joint caculate finished" << endl;

    //cout << "    T_deflection write start" << endl;
    Jacob_deflection = JacobSpace(T_temp[0], T_temp[1], T_temp[2], T_temp[3], T_temp[4], T_temp[5], T_temp[6]);
    T07_deflection = T_joint[13];
    T_deflection << T_joint[0],
        T_joint[1],
        T_joint[2],
        T_joint[3],
        T_joint[4],
        T_joint[5],
        T_joint[6],
        T_joint[7],
        T_joint[8],
        T_joint[9],
        T_joint[10],
        T_joint[11],
        T_joint[12],
        T_joint[13];
    //cout << "    T_deflection write finished" << endl;

    return;
}

void Robot::inverse_kinematics_caculate(const Eigen::MatrixXd &T_target)
{
    Eigen::MatrixXd T = T_target;
    Eigen::MatrixXd Final;
    Eigen::VectorXd diffe(6);
    Eigen::MatrixXd Jacob_Ni;
    Eigen::Vector3d Eular;
    Eigen::VectorXd thetalist(7), dq;

    Eigen::Vector3d F_end;
    F_end << 0.0, 0.0, 0.0;
    Eigen::Vector3d Me_end;
    Me_end << 0.0, 0.0, 0.0;

    Eigen::Vector3d EularTarget = Rotate2Eular(T);
    //Eigen::Vector3d EularTarget = T.block<3,3>(0,0).eulerAngles(0,1,2);
    forward_kinematics_caculate(current_theta); // the position of now
    Final = T07_final;
    Jacob_Ni = pinv(Jacob_final);
    //Jacob_Ni = Jacob.inverse();
    //Eular = Final.block<3,3>(0,0).eulerAngles(2,1,0);

    Eular = Rotate2Eular(Final);
    diffe.block<3, 1>(0, 0) = (T - Final).block<3, 1>(0, 3);
    diffe.block<3, 1>(3, 0) = EularTarget - Eular;
    //        F = pinv(Final) * T - Eigen::Matrix4d::Identity();
    //        std::cout << F << std::endl;
    //        diffe.block<3,1>(3,0);
    dq = Jacob_Ni * diffe;
    //std::cout << "diffe" << std::endl;
    //std::cout << diffe << std::endl;
    //std::cout << "dq" << std::endl;
    //std::cout << dq << std::endl;
    thetalist = dq + current_theta;

    int num = 0;
    bool flag = false;

    while (!flag && num < max_iterate_n)
    {
        num++;
        forward_kinematics_caculate(thetalist);
        Final = T07_final;
        //Eular = Final.block<3,3>(0,0).eulerAngles(0,1,2);
        Eular = Rotate2Eular(Final);
        Jacob_Ni = pinv(Jacob_final);
        // Jacob_Ni = Jacob.inverse();
        diffe.block<3, 1>(0, 0) = (T - Final).block<3, 1>(0, 3);
        diffe.block<3, 1>(3, 0) = EularTarget - Eular;
        flag = Is_OK(diffe);
        dq = Jacob_Ni * diffe;
        thetalist = (dq + thetalist);
        Normalize(thetalist);
    }

    std::cout << "flag: " << flag << std::endl;
    current_theta = thetalist;
}

void Robot::inverse_deflection_caculate(const Eigen::MatrixXd &T_target)
{
    Eigen::MatrixXd T = T_target;
    Eigen::MatrixXd Final;
    Eigen::VectorXd diffe(6);
    Eigen::MatrixXd Jacob_Ni;
    Eigen::Vector3d Eular;
    Eigen::VectorXd thetalist(7), dq;

    Eigen::Vector3d F_end;
    F_end << 0.0, 0.0, 0.0;
    Eigen::Vector3d Me_end;
    Me_end << 0.0, 0.0, 0.0;

    Eigen::Vector3d EularTarget = Rotate2Eular(T);
    //Eigen::Vector3d EularTarget = T.block<3,3>(0,0).eulerAngles(0,1,2);
    forward_deflection_caculate(current_theta); // the position of now
    Final = T07_deflection;
    Jacob_Ni = pinv(Jacob_deflection);
    //Jacob_Ni = Jacob.inverse();
    //Eular = Final.block<3,3>(0,0).eulerAngles(2,1,0);

    Eular = Rotate2Eular(Final);
    diffe.block<3, 1>(0, 0) = (T - Final).block<3, 1>(0, 3);
    diffe.block<3, 1>(3, 0) = EularTarget - Eular;
    //        F = pinv(Final) * T - Eigen::Matrix4d::Identity();
    //        std::cout << F << std::endl;
    //        diffe.block<3,1>(3,0);
    dq = Jacob_Ni * diffe;
    //std::cout << "diffe" << std::endl;
    //std::cout << diffe << std::endl;
    //std::cout << "dq" << std::endl;
    //std::cout << dq << std::endl;
    thetalist = dq + current_theta;

    int num = 0;
    bool flag = false;

    while (!flag && num < max_iterate_n)
    {
        num++;
        forward_deflection_caculate(thetalist);
        Final = T07_deflection;
        //Eular = Final.block<3,3>(0,0).eulerAngles(0,1,2);
        Eular = Rotate2Eular(Final);
        Jacob_Ni = pinv(Jacob_deflection);
        // Jacob_Ni = Jacob.inverse();
        diffe.block<3, 1>(0, 0) = (T - Final).block<3, 1>(0, 3);
        diffe.block<3, 1>(3, 0) = EularTarget - Eular;
        flag = Is_OK(diffe);

        dq = Jacob_Ni * diffe;
        thetalist = (dq + thetalist);
        Normalize(thetalist);
    }

    std::cout << "flag: " << flag << std::endl;
    current_theta = thetalist;
}

void Robot::matplot()
{
    //用matplotlib调试
    PyObject *pModule = NULL;
    PyObject *pFunc = NULL;
    pModule = PyImport_ImportModule("plot");
    if (!pModule)
    {
        cout << "    Py get module failed." << endl;
        return;
    }
    //cout << "    Py get module succeed." << endl;
    pFunc = PyObject_GetAttrString(pModule, "plot");

    double CArrays[6][14];
    for (int i = 0; i < 14; i++)
    {
        CArrays[0][i] = T_final(4 * i + 0, 3);
        CArrays[1][i] = T_final(4 * i + 1, 3);
        CArrays[2][i] = T_final(4 * i + 2, 3);
        CArrays[3][i] = T_deflection(4 * i + 0, 3);
        CArrays[4][i] = T_deflection(4 * i + 1, 3);
        CArrays[5][i] = T_deflection(4 * i + 2, 3);
    }
    //cout << "    CArrays finished" << endl;
    npy_intp Dims[2] = {6, 14}; //给定维度信息
    PyObject *PyArray = PyArray_SimpleNewFromData(2, Dims, NPY_DOUBLE, CArrays);
    PyObject *ArgArray = PyTuple_New(1);
    PyTuple_SetItem(ArgArray, 0, PyArray); //同样定义大小与Python函数参数个数一致的PyTuple对象
    PyObject_CallObject(pFunc, ArgArray);  //调用函数，传入Numpy Array 对象。
}
//神经网络修正
Eigen::VectorXd Robot::network_caculate(const Eigen::VectorXd &thetaL)
{
    //用matplotlib调试
    PyObject *pModule = NULL;
    PyObject *pFunc = NULL;
    pModule = PyImport_ImportModule("bp");
    pFunc = PyObject_GetAttrString(pModule, "network_caculate");

    double CArrays[1][6];
    for (int i = 0; i < 7; i++)
    {
        CArrays[0][i] = thetaL(i);
    }
    //cout << "    CArrays finished" << endl;
    npy_intp Dims[2] = {1, 7}; //给定维度信息
    PyObject *PyArray = PyArray_SimpleNewFromData(2, Dims, NPY_DOUBLE, CArrays);
    PyObject *ArgArray = PyTuple_New(1);
    PyObject *pReturnValue;
    PyTuple_SetItem(ArgArray, 0, PyArray);               //同样定义大小与Python函数参数个数一致的PyTuple对象
    pReturnValue = PyObject_CallObject(pFunc, ArgArray); //调用函数，传入Numpy Array 对象。

    Eigen::VectorXd deflection(3);

    deflection << PyFloat_AsDouble(PyList_GetItem(pReturnValue, 0)),
        PyFloat_AsDouble(PyList_GetItem(pReturnValue, 1)),
        PyFloat_AsDouble(PyList_GetItem(pReturnValue, 2)); //输出元素

    return deflection;
}
//神经网络训练
void Robot::network_train(const Eigen::MatrixXd &thetaLs)
{
    //用matplotlib调试
    PyObject *pModule = NULL;
    PyObject *pFunc = NULL;
    pModule = PyImport_ImportModule("bp");
    if (!pModule)
    {
        cout << "    Py get module failed." << endl;
        return;
    }
    cout << "    Py get module succeed." << endl;
    pFunc = PyObject_GetAttrString(pModule, "network_train");
    double CArrays[0][0];
    //cout << "    CArrays finished" << endl;
    npy_intp Dims[2] = {0, 0}; //给定维度信息
    PyObject *PyArray = PyArray_SimpleNewFromData(2, Dims, NPY_DOUBLE, CArrays);
    PyObject *ArgArray = PyTuple_New(1);
    PyTuple_SetItem(ArgArray, 0, PyArray); //同样定义大小与Python函数参数个数一致的PyTuple对象
    PyObject_CallObject(pFunc, ArgArray);  //调用函数，传入Numpy Array 对象。
}

Eigen::MatrixXd Robot::JacobSpace(const Eigen::MatrixXd &T01, const Eigen::MatrixXd &T12, const Eigen::MatrixXd &T23, const Eigen::MatrixXd &T34, const Eigen::MatrixXd &T45, const Eigen::MatrixXd &T56, const Eigen::MatrixXd &T67)
{
    Eigen::MatrixXd Jacob_temp(6, 7);
    Eigen::MatrixXd T(6, 6);
    Eigen::MatrixXd Final_T = T01 * T12 * T23 * T34 * T45 * T56 * T67;
    Eigen::Vector3d pN = Final_T.block<3, 1>(0, 3);

    T = T01;
    Jacob_temp.block<3, 1>(0, 0) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 0) = T.block<3, 1>(0, 2);
    T = T * T12;
    Jacob_temp.block<3, 1>(0, 1) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 1) = T.block<3, 1>(0, 2);
    T = T * T23;
    Jacob_temp.block<3, 1>(0, 2) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 2) = T.block<3, 1>(0, 2);
    T = T * T34;
    Jacob_temp.block<3, 1>(0, 3) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 3) = T.block<3, 1>(0, 2);
    T = T * T45;
    Jacob_temp.block<3, 1>(0, 4) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 4) = T.block<3, 1>(0, 2);
    T = T * T56;
    Jacob_temp.block<3, 1>(0, 5) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 5) = T.block<3, 1>(0, 2);
    T = T * T67;
    Jacob_temp.block<3, 1>(0, 6) = T.block<3, 1>(0, 2).cross(pN - T.block<3, 1>(0, 3));
    Jacob_temp.block<3, 1>(3, 6) = T.block<3, 1>(0, 2);
    return Jacob_temp;
}

Eigen::Vector3d Robot::Rotate2Eular(const Eigen::MatrixXd T_target)
{
    Eigen::Vector3d rotate(3);
    double roll = 0, pitch = 0, yaw = 0;
    pitch = atan2(-T_target(2, 0), sqrt(T_target(2, 1) * T_target(2, 1) + T_target(2, 2) * T_target(2, 2)));
    if (pitch >= -M_PI / 2.0 || pitch <= M_PI / 2.0)
    {
        roll = atan2(T_target(2, 1), T_target(2, 2));
        yaw = atan2(T_target(1, 0), T_target(0, 0));
    }
    else
    {
        roll = atan2(-T_target(2, 1), -T_target(2, 2));
        yaw = atan2(-T_target(1, 0), -T_target(0, 0));
    }
    rotate << roll, pitch, yaw;
    return rotate;
}

Eigen::MatrixXd Robot::pinv(const Eigen::MatrixXd &A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);

    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

    return X;
}

void Robot::Normalize(Eigen::VectorXd &angle)
{
    for (int i = 0; i < angle.size(); i++)
    {
        while (angle[i] > M_PI || angle[i] < -M_PI)
        {
            if (angle[i] > M_PI)
                angle[i] -= M_PI;
            if (angle[i] < -M_PI)
                angle[i] += M_PI;
        }
    }
}

bool Robot::Is_OK(const Eigen::VectorXd &diffe)
{
    for (int j = 0; j < 6; j++)
    {
        if (fabs(diffe(j)) > tolerance)
        {
            return false;
        }
    }
    return true;
}