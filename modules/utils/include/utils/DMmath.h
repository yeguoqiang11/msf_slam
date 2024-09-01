/**
 * @file DMmath.h
 * @author Guoqiang Ye (yegq@Yeguoqiang.tech)
 * @brief 
 * @version 0.1
 * @date 2021-08-01
 * 
 * @copyright Copyright (c) 2022 Yeguoqiang Technology Co.Ltd. All rights reserved
 * 
 */
#ifndef SRC_DMATH_H
#define SRC_DMATH_H
#include <iostream>
#include <math.h>
#include <time.h>
#include <stdio.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#ifndef RAD2DEG
#define RAD2DEG 57.29577951
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.01745329252
#endif

namespace utils {
class DMmath
{
  public:
    DMmath() {};
    static Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d &vec);

    static Eigen::Matrix3d RotationVector2Matrix(const Eigen::Vector3d &vec);

    static Eigen::Vector4d Matrix2Quaternion(const Eigen::Matrix3d &mat);

    static Eigen::Matrix3d Quaternion2Matrix(const Eigen::Vector4d &quat);

    static Eigen::Vector3d Matrix2RotationVector(const Eigen::Matrix3d &mat);

    static Eigen::Matrix3d Matrix3dInverse(const Eigen::Matrix3d &mat);

    static Eigen::Matrix3d RotationNormalization(Eigen::Matrix3d &R);

    static Eigen::Matrix3d RightJacobian(const Eigen::Vector3d &v);

    static Eigen::Matrix3d RightJacobianInverse(const Eigen::Vector3d &v);

    static Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d &v);

    static Eigen::Matrix3d LeftJacobianInverse(const Eigen::Vector3d &v);

    static Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

    static Eigen::Matrix2d Yaw2Matrix(double yaw);

    /**
     * @brief calculate euler angle from rotation matrix
     * 
     * @param mat ration matrix mat = Rz * Ry * Rx
     * @return Eigen::Vector3d euler angle = [roll, pitch, yaw], [rx, ry, rz]
     */
    static Eigen::Vector3d RotationMatrix2EulerAngle(const Eigen::Matrix3d &mat);

    /**
     * @brief calculate rotation matrix from euler angle
     * 
     * @param euler euler angle = [roll, pitch, yaw], [rx, ry, rz]
     * @return Eigen::Matrix3d rotation matrix Rz * Ry * Rx
     */
    static Eigen::Matrix3d EulerAngle2Matrix(const Eigen::Vector3d euler);

    /**
     * @brief calculate euler angle from rotation matrix
     * 
     * @param mat rotation matrix mat = Ry * Rx * Rz
     * @return euler angle = [rx, ry, rz]
     */
    static Eigen::Vector3d RotationMatrix2EulerAngleZXY(const Eigen::Matrix3d &mat);

    /**
     * @brief calculate matrix from euler angle
     * 
     * @param euler euler angle [rx, ry, rz]
     * @return matrix Ry * Rx * Rz
     */
    static Eigen::Matrix3d EulerAngleZXY2Matrix(const Eigen::Vector3d &euler);

    /**
     * @brief 3X3 symmetric matrix Diagonalization
     * @param m input 3X3 symmetric matrix
     * @param eig output eigenvalues [lambda1; lambda2; lambda3]
     * @param vt [eigenvector0; eigenvector1; eigenvector2] m = v * diag(eig) * vt
     */
    static void Matrix3dEigenDecomposition(const Eigen::Matrix3d &m, Eigen::Vector3d &eig, Eigen::Matrix3d &vt);

    static Eigen::Vector4d QuaternionProduct(const Eigen::Vector4d &q0, const Eigen::Vector4d &q1);

    static Eigen::Vector4d ConjugateQuaternion(const Eigen::Vector4d &q);

    static Eigen::Vector3d VectorCrossProduct(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1);

    /**
     * @brief transform multiply
     * @result [R, t] = [R0, t0; 0, 1] * [R1, t1; 0, 1]
     */
    static void TransformMultiply(const Eigen::Matrix3d &R0, const Eigen::Vector3d &t0, const Eigen::Matrix3d &R1,
                             const Eigen::Vector3d &t1, Eigen::Matrix3d &R, Eigen::Vector3d &t);
    
    static void FirstTwoDominantEigenCal(const Eigen::Matrix3d &m, Eigen::Vector3d &v0, Eigen::Vector3d &v1, double &lambda0, double &lambda1);
};

template<class T = double>
class Transform {
  public:
    Transform() {}
    Transform(const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &t): R_(R), t_(t) {}
    Transform(const Transform<T> &pose): R_(pose.R_), t_(pose.t_) {}
    Transform(const Eigen::Matrix<T, 6, 1> &v) { 
      SetIdentity();
      DisturbanceUpdate(v.topRows(3), v.bottomRows(3));
    }
    const Transform<T> &operator=(const Transform<T> &pose) {
        this->R_ = pose.R_;
        this->t_ = pose.t_;
        return *this;
    }
    void DisturbanceUpdate(const Eigen::Matrix<T, 3, 1> &drv, const Eigen::Matrix<T, 3, 1> &dt) {
        T angle = drv.norm();
        Eigen::Matrix<T, 3, 3> dR = Eigen::Matrix<T, 3, 3>::Identity();
        if (angle > std::numeric_limits<T>::epsilon() * T(100.)) {
            Eigen::Matrix<T, 3, 3> vec_skew;
            T an_inv = T(1.) / angle;
            vec_skew << T(0.), -drv(2), drv(1),
                        drv(2), T(0.), -drv(0),
                        -drv(1), drv(0), T(0.);
            vec_skew *= an_inv;
            dR += sin(angle) * vec_skew + (T(1.0) - cos(angle)) * vec_skew * vec_skew;
        }

        R_ *= dR;
        t_ += dt;
    }

    void SetIdentity() {
        R_.setIdentity();
        t_.setZero();
    }

    void SetValue(const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &t) {
        R_ = R;
        t_ = t;
    }

    void Normalization() {
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R_ = svd.matrixU() * svd.matrixV().transpose();
    }
    Transform<T> operator*(const Transform<T> &pose) const {
        return Transform<T>(this->R_ * pose.R_, this->R_ * pose.t_ + this->t_);
    }
    Eigen::Matrix<T, 3, 1> transform(const Eigen::Matrix<T, 3, 1> &v) const {
        return R_ * v + t_;
    }
    // v = [drv; dt]
    Transform<T> operator*(const Eigen::Matrix<T, 6, 1> &v) const {
        Transform<T> pose(R_, t_);
        pose.DisturbanceUpdate(v.topRows(3), v.bottomRows(3));
        return std::move(pose);
    }
    // v = [drv, dt]
    const Transform<T> &operator*=(const Eigen::Matrix<T, 6, 1> &v) {
        DisturbanceUpdate(v.topRows(3), v.bottomRows(3));
        return *this;
    }
    const Transform<T> &operator*=(const Transform<T> &pose) {
      this->t_ = this->R_ * pose.t_ + this->t_;
      this->R_ = this->R_ * pose.R_;
      return *this;
    }

    Transform<T> Inverse() const {
        return Transform<T>(R_.transpose(), -R_.transpose() * t_);
    }

    Eigen::Matrix<T, 3, 3> &R() { return R_; }
    Eigen::Matrix<T, 3, 1> &t() { return t_; }
    T &R(const int &i, const int &j) { return R_(i, j); }
    const T &R(const int &i, const int &j) const { return R_(i, j); }
    T &t(const int &i) { return t_(i); }
    const T &t(const int &i) const { return t_(i); }
    const Eigen::Matrix<T, 3, 3> &R() const { return R_; };
    const Eigen::Matrix<T, 3, 1> &t() const { return t_; }

    friend std::ostream &operator<<(std::ostream& out, const Transform<T> &pose) {
        out << "\n[R: \n" << pose.R_ << "\nt: " << pose.t_.transpose() << "]" << std::endl;
        return out;
    }
  private:
    Eigen::Matrix<T, 3, 3> R_;
    Eigen::Matrix<T, 3, 1> t_;
};

class Timer {
  public:
    Timer() { start = clock(); }
    void Start() { start = clock(); }
    double End(std::string label = "") { 
      end = clock();
      double dt = (double)(end - start) / CLOCKS_PER_SEC;
      std::cout << label << " time cost: " << dt << std::endl;
      start = end;
      return dt;
    }
    void PrintTime(std::string label = "") {
      end = clock();
      std::cout << label << " time: " << (double)end / CLOCKS_PER_SEC << std::endl;
    }
  private:
    clock_t start, end;
};

typedef Transform<double> Transform3d;
} // namespace utils
#endif // SRC_DMATH_H
