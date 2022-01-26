#include "deflection.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <fstream>

using namespace std;

int main()
{
    Robot testRobot;

    char order[5];

    while (true)
    {
        cin >> order;
        if (strcmp(order, "train") == 0)
        {
            Eigen::VectorXd thetaLs(0);
            testRobot.network_train(thetaLs);
        }
        if (strcmp(order, "test") == 0)
        {
            Eigen::VectorXd thetaL(7);
            Eigen::VectorXd deflection(3);
            double t1, t2, t3, t4, t5;

            t1 = rand() % 20 - 10;
            t2 = rand() % 20 - 30;
            t3 = rand() % 20 - 10;
            t4 = rand() % 20 + 60;
            t5 = rand() % 20 + 0;
            thetaL << t5, t4, 2.30, t3, t2, t1, 0;
            cout << thetaL.transpose() << "\t\t\t\t\t";
            thetaL = thetaL / 180 * M_PI;
            thetaL(2) = thetaL(2) * 180 / M_PI;
            testRobot.forward_deflection_caculate(thetaL);
            deflection = testRobot.network_caculate(thetaL);
            cout << ((testRobot.T07_deflection.block(0, 3, 3, 1) - testRobot.T07_final.block(0, 3, 3, 1)).transpose() - deflection.transpose()).norm() << endl;
        }
        if (strcmp(order, "run") == 0)
        {
            Eigen::Matrix4d T_target;
            Eigen::Matrix4d T_target1;
            Eigen::Matrix4d T_target2;
            Eigen::VectorXd theta_init(7);
            Eigen::VectorXd theta_network(7);
            Eigen::VectorXd theta_deflection(7);
            Eigen::VectorXd dp_network(3);
            Eigen::VectorXd dp_deflection(3);

            Eigen::VectorXd thetaL(7);
            double t1, t2, t3, t4, t5, td;

            t1 = rand() % 20 * 1.0 - 10;
            t2 = rand() % 20 * 1.0 - 30;
            t3 = rand() % 20 * 1.0 - 10;
            t4 = rand() % 20 * 1.0 + 60;
            t5 = rand() % 20 * 1.0 + 0;
            td = rand() % 100 * 0.01 + 2.30;

            //theta_init<<  t5,     t4,     D,      t3,     t2,     t1,     0;
            theta_init << t5, t4, td, t3, t2, t1, 0;
            cout << "theta_init:       " << theta_init.transpose() << endl;
            theta_init = theta_init / 180 * M_PI;
            theta_init(2) = theta_init(2) * 180 / M_PI;
            testRobot.forward_deflection_caculate(theta_init);
            double x0, y0, z0;
            x0 = testRobot.T07_deflection(0, 3) * 100;
            y0 = testRobot.T07_deflection(1, 3) * 100;
            z0 = testRobot.T07_deflection(2, 3) * 100;
            cout << "target:           " << x0 << ",     " << y0 << ",     " << z0 << endl;

            T_target = testRobot.T07_final;
            T_target1 = T_target;
            T_target2 = T_target;

            //cout << "T_target:    " << endl;
            //cout << T_target.block(0, 3, 3, 1).transpose() << " ";
            //T_target = testRobot.T07_final;
            //T_target1 = testRobot.T07_deflection;

            testRobot.inverse_kinematics_caculate(T_target);
            theta_init = testRobot.current_theta;

            dp_deflection = testRobot.T07_deflection.block(0, 3, 3, 1) - testRobot.T07_final.block(0, 3, 3, 1);
            testRobot.forward_deflection_caculate(theta_init);
            testRobot.inverse_deflection_caculate(T_target1);
            theta_deflection = testRobot.current_theta;

            dp_network = testRobot.network_caculate(theta_init);
            T_target2.block(0, 3, 3, 1) -= dp_network;
            testRobot.inverse_kinematics_caculate(T_target2);
            theta_network = testRobot.current_theta;

            theta_init = theta_init * 180 / M_PI;
            theta_init(2) = theta_init(2) / 180 * M_PI;
            theta_deflection = theta_deflection * 180 / M_PI;
            theta_deflection(2) = theta_deflection(2) / 180 * M_PI;
            theta_network = theta_network * 180 / M_PI;
            theta_network(2) = theta_network(2) / 180 * M_PI;

            cout << "dp_deflection:    " << dp_deflection.transpose() * 100 << endl;
            cout << "dp_network:       " << dp_network.transpose() * 100 << endl;

            //testRobot.forward_deflection_caculate(theta_init);
            cout << "theta_init:       " << theta_init.transpose() << endl;
            //cout << (testRobot.T07_deflection - T_target).norm() << endl;
            //testRobot.forward_deflection_caculate(theta_deflection);
            cout << "theta_deflection: " << theta_deflection.transpose() << endl;
            //cout << (testRobot.T07_deflection - T_target).norm() << endl;
            //testRobot.forward_deflection_caculate(theta_network);
            cout << "theta_network:    " << theta_network.transpose() << endl;
            //cout << (testRobot.T07_deflection - T_target).norm() << endl;
        }
        if (strcmp(order, "data") == 0)
        {
            Eigen::VectorXd theta(7);
            double t1, t2, t3, t4, t5;

            for (t4 = 60; t4 <= 80; t4 += 10)
            {
                for (t5 = 0; t5 <= 20; t5 += 10)
                {
                    for (t3 = -10; t3 <= 10; t3 += 10)
                    {
                        for (t2 = -30; t2 <= -10; t2 += 10)
                        {
                            for (t1 = -10; t1 <= 10; t1 += 10)
                            {
                                theta << t5, t4, 3.44, t3, t2, t1, 0;
                                theta = theta / 180 * 3.14;
                                theta(2) = theta(2) * 180 / 3.14;
                                testRobot.forward_deflection_caculate(theta);
                                //cout << testRobot.T07_deflection.block(0, 3, 3, 1).transpose() << endl;
                                cout << testRobot.T07_final.block(0, 3, 3, 1).transpose() * 100 << endl;
                                //testRobot.matplot();
                            }
                        }
                    }
                }
            }
            testRobot.matplot();
        }
        if (strcmp(order, "end") == 0)
        {
            testRobot.matplot();
            testRobot.inverse_kinematics_caculate(testRobot.T07_final);
            cout << testRobot.T07_final << endl;
            testRobot.matplot();
            break;
        }
    }
}
