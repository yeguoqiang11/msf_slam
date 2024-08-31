#include "utils/DMmath.h"

namespace utils{
Eigen::Matrix3d DMmath::SkewMatrix(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d out;
    out << 0., -vec(2), vec(1),
           vec(2), 0., -vec(0),
           -vec(1), vec(0), 0.;
    return out;
}

// R = cos(theta) * I + (1 - cos_theta) * r * r^t + sin_theta * r^; ||r|| = 1; r^ = skew_matrix(r);
Eigen::Matrix3d DMmath::RotationVector2Matrix(const Eigen::Vector3d &vec) {
    double angle = vec.norm();
    if (angle <= std::numeric_limits<double>::epsilon() * 100) return Eigen::Matrix3d::Identity();
    Eigen::Matrix3d vec_skew = SkewMatrix(vec) / angle;
    Eigen::Matrix3d out = Eigen::Matrix3d::Identity();
    out += sin(angle) * vec_skew + (1.0 - cos(angle)) * vec_skew * vec_skew;
    return out;
}

// quat = [x, y, z, w], Hammiton quaternion
Eigen::Vector4d DMmath::Matrix2Quaternion(const Eigen::Matrix3d &mat) {
    Eigen::Vector4d quat;
    double tr = mat.trace();
    if (tr > 0) {
        tr = sqrt(tr + 1.0);
        quat(3) = 0.5 * tr;
        tr = 0.5 / tr;
        quat(0) = (mat(1, 2) - mat(2, 1)) * tr;
        quat(1) = (mat(2, 0) - mat(0, 2)) * tr;
        quat(2) = (mat(0, 1) - mat(1, 0)) * tr;
    } else {
        int i = 0;
        if (mat(1, 1) > mat(0, 0)) i = 1;
        if (mat(2, 2) > mat(i, i)) i = 2;
        int j = (i + 1) % 3;
        int k = (j + 1) % 3;

        tr = sqrt(mat(i, i) - mat(j, j) - mat(k, k) + 1);
        quat(i) = 0.5 * tr;
        tr = 0.5 / tr;
        quat(3) = (mat(j, k) - mat(k, j)) * tr;
        quat(j) = (mat(i, j) + mat(j, i)) * tr;
        quat(k) = (mat(i, k) + mat(k, i)) * tr;
    }
    quat /= quat.norm();
    return quat;
}

// quat = [x, y, z, w], Hammiton quaternion
Eigen::Matrix3d DMmath::Quaternion2Matrix(const Eigen::Vector4d &quat) {
    const Eigen::Vector3d v = quat.topRows(3);
    const double w = quat(3);
    Eigen::Matrix3d v_hat = SkewMatrix(v);
    Eigen::Matrix3d out; out.setIdentity();
    out += -2.0 * w * v_hat + 2.0 * v_hat * v_hat;
    return out;

    // or
    // out = (2.0 * w * w - 1) * Eigen::Matrix3d::Identity() - 2 * w * SkewMatrix(v) + 2 * v * v.transpose();
    // return out;
}

Eigen::Vector3d DMmath::Matrix2RotationVector(const Eigen::Matrix3d &mat) {
    Eigen::Vector4d quat = Matrix2Quaternion(mat);
    double square_n = quat.topRows(3).squaredNorm();
    double n = sqrt(square_n);
    double w = quat(3);

    double angle_by_cos;
    if (n <= std::numeric_limits<double>::epsilon() * 10) {
        // theta * cos(0.5 * theta)^2 = 2 * (cos(0.5 * theta)^2 - sin(0.5 * theta)^2)
        angle_by_cos = 2.0 / w - 2.0 * (square_n) / (w * w * w);
    } else {
        if (fabs(w) <= std::numeric_limits<double>::epsilon() * 10) {
            if (w > 0) angle_by_cos = M_PI / n;
            else angle_by_cos = -M_PI / n;
        } else {
            angle_by_cos = 2.0 * atan(n / w) / n;
        }
    }
    return -angle_by_cos * quat.topRows(3);
}

Eigen::Matrix3d DMmath::Matrix3dInverse(const Eigen::Matrix3d &mat) {
    double det = mat.determinant();
    if (fabs(det) <= std::numeric_limits<double>::epsilon() * 10) return Eigen::Matrix3d::Zero();
    Eigen::Matrix3d out;
    Eigen::Vector3d c0 = mat.col(0);
    Eigen::Vector3d c1 = mat.col(1);
    Eigen::Vector3d c2 = mat.col(2);
    out.row(0) << c1(1) * c2(2) - c1(2) * c2(1), -(c1(0) * c2(2) - c1(2) * c2(0)), c1(0) * c2(1) - c2(0) * c1(1);
    out.row(1) << c2(1) * c0(2) - c2(2) * c0(1), -(c2(0) * c0(2) - c2(2) * c0(0)), c2(0) * c0(1) - c0(0) * c2(1);
    out.row(2) << c0(1) * c1(2) - c0(2) * c1(1), -(c0(0) * c1(2) - c0(2) * c1(0)), c0(0) * c1(1) - c1(0) * c0(1);
    return out * (1.0 / det);
}

Eigen::Matrix3d DMmath::RotationNormalization(Eigen::Matrix3d &R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3d DMmath::RightJacobian(const Eigen::Vector3d &v) {
    Eigen::Matrix3d out; out.setIdentity();
    double angle = v.norm();
    if (angle <= std::numeric_limits<double>::epsilon() * 10) return out;
    double angle_inv = 1.0 / angle;
    Eigen::Matrix3d skew = SkewMatrix(v);
    out += -(1. - cos(angle)) * angle_inv * angle_inv * skew + (angle - sin(angle)) * std::pow(angle_inv, 3) * skew * skew;
    return out;
}

Eigen::Matrix3d DMmath::RightJacobianInverse(const Eigen::Vector3d &v) {
    Eigen::Matrix3d out; out.setIdentity();
    double angle = v.norm();
    if (angle <= std::numeric_limits<double>::epsilon() * 10) return out;
    double angle_inv = 1. / angle;
    Eigen::Matrix3d skew = utils::DMmath::SkewMatrix(v);
    out += 0.5 * skew + (angle_inv * angle_inv + (1 + cos(angle)) * 0.5 * angle_inv / sin(angle)) * skew * skew;
    return out;
}

Eigen::Matrix3d DMmath::LeftJacobian(const Eigen::Vector3d &v) {
    double angle = v.norm();
    if (angle <= std::numeric_limits<double>::epsilon() * 1000) return Eigen::Matrix3d::Identity();
    double anlge_inv = 1.0 / angle;
    Eigen::Vector3d a = v * anlge_inv;
    Eigen::Matrix3d a_hat = SkewMatrix(a);

    Eigen::Matrix3d out; out.setIdentity();
    double sin_a = sin(angle);
    double cos_a = cos(angle);
    out *= sin_a * anlge_inv;
    out += (1. - sin_a * anlge_inv) * a * a.transpose() + (1. - cos_a) * anlge_inv * a_hat;
    return out;
}

Eigen::Matrix3d DMmath::LeftJacobianInverse(const Eigen::Vector3d &v) {
    double theta = 0.5 * v.norm();
    if (theta <= std::numeric_limits<double>::epsilon() * 1000) return Eigen::Matrix3d::Identity();
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    Eigen::Vector3d a = v * 0.5 / theta;
    Eigen::Matrix3d a_hat = SkewMatrix(a);

    double cot_theta = cos_theta / sin_theta;
    Eigen::Matrix3d out; out.setIdentity();
    out *= theta * cot_theta;
    out += (1. - theta * cot_theta) * a * a.transpose() - theta * a_hat;
    return out;
}

// R = cos_theta * I + (1 - cos_theta) * r * r^t + sin_theta * r^; r^ = skew_matrix(r); ||r|| = 1;
// R - R^t = 2 * sin_theta * r^
// trace(R) = cos_theta * 3 + (1 - cos_theta) = 2 * cos_theta + 1 => cos_theta = (trace(R) - 1) * 0.5;
// if theta = Pi => R = -I + 2 * r * r^t; r * r^t = [rx * rx, rx * ry, rx * rz; ry * rx, ry * ry, ry * rz; rz * rx, rz * ry, rz * rz]
// if theta = 0 => R = I 
Eigen::Vector3d DMmath::LogSO3(const Eigen::Matrix3d &R) {
    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    Eigen::Vector3d a;
    a << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    a *= 0.5;
    double cos_theta = (trace - 1.0) * 0.5;
    if (fabs(cos_theta - 1.) <= std::numeric_limits<double>::epsilon() * 1000) return Eigen::Vector3d::Zero();
    if (fabs(-1. - cos_theta) <= std::numeric_limits<double>::epsilon() * 1000) {
        int i = 0;
        if (R(i, i) < R(1, 1)) i = 1;
        if (R(i, i) < R(2, 2)) i = 2;
        a(i) = sqrt(R(i, i) + 1);
        int j = (i + 1) % 3;
        a(j) = R(i, j) / a(i);
        j = (j+1) % 3;
        a(j) = R(i, j) / a(i);
        a /= a.norm();
        return M_PI * a;
    }
    double theta = acos(cos_theta);
    return theta * a / a.norm();
}

// Rbw = [cos_theta, -sin_theta; sin_theta, cos_theta]
Eigen::Matrix2d DMmath::Yaw2Matrix(double yaw) {
    Eigen::Matrix2d out;
    double cos_theta = cos(yaw);
    double sin_theta = sin(yaw);
    out << cos_theta, -sin_theta,
           sin_theta, cos_theta;

    return out;
}

// XYZ rotation; Rbw = Rz * Ry * Rx
Eigen::Vector3d DMmath::RotationMatrix2EulerAngle(const Eigen::Matrix3d &mat) {
    Eigen::Vector3d euler(0., 0., 0.);

    Eigen::Ref<const Eigen::Vector3d> c0 = mat.col(0);
    Eigen::Ref<const Eigen::Vector3d> c1 = mat.col(1);
    Eigen::Ref<const Eigen::Vector3d> c2 = mat.col(2);

    if (fabs(c1(2)) < std::numeric_limits<double>::epsilon() * 1000 &&
        fabs(c2(2)) < std::numeric_limits<double>::epsilon() * 1000) {
        if (-c0(2) >= 0.) {
            euler(1) = 0.5 * M_PI;
            euler(0) = atan2(c1(0), c1(1));
        } else {
            euler(1) = -0.5 * M_PI;
            euler(0) = atan2(-c1(0), c1(1));
        }
    } else {
        euler(0) = atan2(c1(2), c2(2));
        euler(1) = atan2(-c0(2), sqrt(std::pow(c1(2), 2) + std::pow(c2(2), 2)));
        euler(2) = atan2(c0(1), c0(0));
    }
    return euler;
}

Eigen::Matrix3d DMmath::EulerAngle2Matrix(const Eigen::Vector3d euler) {
    double sin_r = sin(euler(0)), cos_r = cos(euler(0)),
           sin_p = sin(euler(1)), cos_p = cos(euler(1)),
           sin_y = sin(euler(2)), cos_y = cos(euler(2));

    Eigen::Matrix3d out;
    out << cos_p * cos_y, sin_r * sin_p * cos_y - cos_r * sin_y, cos_r * sin_p * cos_y + sin_r * sin_y,
           cos_p * sin_y, sin_r * sin_p * sin_y + cos_r * cos_y, cos_r * sin_p * sin_y - sin_r * cos_y,
           -sin_p, sin_r * cos_p, cos_r * cos_p;
    return out;
}

// Rxw = [1, 0, 0; 0, cosx, -sinx; 0, sinx, cosx]
// Ryw = [cosy, 0, siny; 0, 1, 0; -siny, 0, cosy]
// Rzw = [cosz, -sinz, 0; sinz, cosz, 0; 0, 0, 1]
// Rbw = Ryw * Rxw * Rzw = Ryw * [cz, -sz, 0; cxsz, cxcz, -sx; sxsz, sxcz, cx]
// = [cycz+sysxsz, -cysz+sysxcz, sycx; cxsz, cxcz, -sx; -sycz+cysxsz, sysz+cysxcz, cycx]
// euler = [rx, ry, rz]
Eigen::Vector3d DMmath::RotationMatrix2EulerAngleZXY(const Eigen::Matrix3d &mat) {
    Eigen::Vector3d euler(0., 0., 0.);

    Eigen::Ref<const Eigen::Vector3d> c0 = mat.col(0);
    Eigen::Ref<const Eigen::Vector3d> c1 = mat.col(1);
    Eigen::Ref<const Eigen::Vector3d> c2 = mat.col(2);

    if (fabs(c0(1)) < std::numeric_limits<double>::epsilon() * 1000 &&
        fabs(c1(1)) < std::numeric_limits<double>::epsilon() * 1000) {
        if (-c2(1) >= 0.) {
            euler(0) = 0.5 * M_PI;
            euler(2) = atan2(c0(2), c1(2));
        } else {
            euler(0) = -0.5 * M_PI;
            euler(2) = atan2(-c1(0), c0(0));
        }
    } else {
        euler(2) = atan2(c0(1), c1(1));
        euler(0) = atan2(-c2(1), sqrt(std::pow(c0(1), 2) + std::pow(c1(1), 2)));
        euler(1) = atan2(c2(0), c2(2));
    }
    return euler;
}

Eigen::Matrix3d DMmath::EulerAngleZXY2Matrix(const Eigen::Vector3d &euler) {
    double sx = sin(euler(0)), cx = cos(euler(0));
    double sy = sin(euler(1)), cy = cos(euler(1));
    double sz = sin(euler(2)), cz = cos(euler(2));

    Eigen::Matrix3d mat;
    mat << cy * cz + sy * sx * sz, -cy * sz + sy * sx * cz, sy * cx,
           cx * sz, cx * cz, -sx,
           -sy * cz + cy * sx * sz, sy * sz + cy * sx * cz, cy * cx;
    return mat;
}

// reference: Efficient numerical diagonalization of hermitian 3 × 3 matrices
// mat is a real symmetric matrix
// m = v * diag(eig) * vt
void DMmath::Matrix3dEigenDecomposition(const Eigen::Matrix3d &m, Eigen::Vector3d &eig, Eigen::Matrix3d &vt) {
    double m00 = m(0, 0), m11 = m(1, 1), m22 = m(2, 2), m01 = m(0, 1), m02 = m(0, 2), m12 = m(1, 2);
    double cx = 1. / 3., cy = 2. * cx;
    double c2 = -m00 - m11 - m22;
    double c1 = m00 * m11 + m00 * m22 + m11 * m22 - m01 * m01 - m02 * m02 - m12 * m12;
    double c0 = m00 * m12 * m12 + m11 * m02 * m02 + m22 * m01 * m01 - m00 * m11 * m22
                - 2. * m02 * m01 * m12;
    double p = c2 * c2 - 3. * c1;
    double q = -13.5 * c0 - std::pow(c2, 3) + 4.5 * c2 * c1;
    double a = 27. * (0.25 * c1 * c1 * (p - c1) + c0 * (q + 6.75 * c0));
    if (a < 0.) a = fabs(a);
    double phi = cx * atan2(sqrt(a), q);
    double x1 = 2. * cos(phi);
    double x2 = 2. * cos(phi + cy * M_PI);
    double x3 = 2. * cos(phi - cy * M_PI);

    double sqrt_p = sqrt(p);

    eig(0) = cx * (sqrt_p * x1 - c2);
    eig(1) = cx * (sqrt_p * x2 - c2);
    eig(2) = cx * (sqrt_p * x3 - c2);

    double e, e1, v0_sqnorm, v1_sqnorm;
    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d v0 = m.col(0) - eig(i) * Eigen::Vector3d(1., 0., 0.);
        Eigen::Vector3d v1 = m.col(1) - eig(i) * Eigen::Vector3d(0., 1., 0.);

        v0_sqnorm = v0.squaredNorm();
        v1_sqnorm = v1.squaredNorm();
        if (v0_sqnorm <= 0.) v0 = m.col(2) - eig(i) * Eigen::Vector3d(0., 0., 1.);
        if (v1_sqnorm <= 0.) v1 = m.col(2) - eig(i) * Eigen::Vector3d(0., 0., 1.);
        e = (v0.cross(v1)).squaredNorm();
        e1 = v0.dot(v1);
        if (e <= 0. && e1 > 0.) {
            vt.row(i) = Eigen::Vector3d(1., -e, 0.).normalized().transpose();
            continue;
        }

        vt.row(i) = (v0.cross(v1)).transpose();
        vt.row(i).normalize();
    }
    
    // sort eigen value and eigen vector
    if (fabs(eig(0)) < fabs(eig(1))) {
        std::swap(eig(0), eig(1));
        vt.row(1).swap(vt.row(0));
    }

    if (fabs(eig(0)) < fabs(eig(2))) {
        std::swap(eig(0), eig(2));
        vt.row(2).swap(vt.row(0));
    }

    if (fabs(eig(1)) < fabs(eig(2))) {
        std::swap(eig(1), eig(2));
        vt.row(2).swap(vt.row(1));
    }
}

// q0 = [x, y, z, w], q1 = [x, y, z, w]
// qab = qaw * qwb
Eigen::Vector4d DMmath::QuaternionProduct(const Eigen::Vector4d &q0, const Eigen::Vector4d &q1) {
    Eigen::Vector4d out;
    out.topRows(3) = q0(3) * q1.topRows(3) + q1(3) * q0.topRows(3) + VectorCrossProduct(q1.topRows(3), q0.topRows(3));
    out(3) = q0(3) * q1(3) - q0.topRows(3).dot(q1.topRows(3));
    return out;
}

Eigen::Vector4d DMmath::ConjugateQuaternion(const Eigen::Vector4d &q) {
    Eigen::Vector4d out = q;
    out.topRows(3) *= -1.;
    return out;
}

Eigen::Vector3d DMmath::VectorCrossProduct(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1) {
    Eigen::Vector3d out;
    out << v0(1) * v1(2) - v0(2) * v1(1), v0(2) * v1(0) - v0(0) * v1(2), v0(0) * v1(1) - v0(1) * v1(0);
    return out;
}

void DMmath::TransformMultiply(const Eigen::Matrix3d &R0, const Eigen::Vector3d &t0, const Eigen::Matrix3d &R1,
                             const Eigen::Vector3d &t1, Eigen::Matrix3d &R, Eigen::Vector3d &t) {
    // [R0, t0] * [R1, t1] = [R0 * R1, R0 * t1 + t0]
    t = R0 * t1 + t0;
    R = R0 * R1;
}

// Power Method for two dominant eigenvectors and eigenvalues from symmetric matrix of A
void DMmath::FirstTwoDominantEigenCal(const Eigen::Matrix3d &m, Eigen::Vector3d &v0, Eigen::Vector3d &v1,
                                      double &lambda0, double &lambda1) {
    v0 << 1., 0., 0.;
    for (int i = 0; i < 20; i++) {
        v0.normalize();
        v0 = m * v0;
        lambda0 = v0.norm();
    }
    v0.normalize();

    v1 << 0., 1., 0.;
    double e = fabs(v0.dot(v1));
    if ((1. - e) >= std::numeric_limits<double>::epsilon() * 1000.) {
        v1 << 1., 0., 0.;
    }
    for (int i = 0; i < 50; i++) {
        v1 = v1 - v1.dot(v0) * v0;
        v1.normalize();
        v1 = m * v1;
        lambda1 = v1.norm();
    }
    v1.normalize();
}


} // namespace slam

