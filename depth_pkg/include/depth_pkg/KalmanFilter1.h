#ifndef KALMAN_FILTER_
#define KALMAN_FILTER_
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


class KalmanFilter1
{
private:
    bool is_initialized;
    VectorXd x_;
    MatrixXd F_;
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd H_;
    MatrixXd R_;

public:
    KalmanFilter1()
    {
        is_initialized = false;
    }
    ~KalmanFilter1()
    {};

    VectorXd GetX()
    {
        return x_;
    }

    bool IsInitialized()
    {
        return is_initialized;
    }

    void Initialization(VectorXd x_in)
    {
        x_ = x_in;
        is_initialized = true;
    }

    void SetF(MatrixXd F_in)
    {
        F_ = F_in;
    }

    void SetP(MatrixXd P_in)
    {
        P_ = P_in;
    }

    void SetQ(MatrixXd Q_in)
    {
        Q_ = Q_in;
    }

    void SetH(MatrixXd H_in)
    {
        H_ = H_in;
    }

    void SetR(MatrixXd R_in)
    {
        R_ = R_in;
    }

    void Prediction()
    {
        x_ = F_*x_;
        MatrixXd Ft = F_.transpose();
        P_ = F_*P_*Ft+Q_;
    }

    void MeasurementUpdate(const VectorXd &Z)
    {
        VectorXd Y = Z - H_*x_;
        MatrixXd S = H_*P_*H_.transpose()+R_;
        MatrixXd K = P_*H_.transpose()*S.inverse();
        x_ = x_ +K*Y;
        int size = x_.size();
        MatrixXd I = MatrixXd::Identity(size,size);

        P_ = (I-K*H_)*P_;
    }
};
#endif
