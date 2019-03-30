//
// Created by Shimin Zhang on 4/9/18.
//

#ifndef KRANG_BALANCING_DDP_OBJECTS_H_
#define KRANG_BALANCING_DDP_OBJECTS_H_

#include <ddp/costs.hpp>
#include <ddp/dynamics.hpp>
#include <ddp/eigenmvn.hpp>
#include <ddp/plant.hpp>
#include <fstream>
#include <iostream>

template <class T>
struct TwipDynamics : public Dynamics<T, 8, 2> {
  struct Parameters {
    double R, mw, L, g;
    double m_1;
    double MX_1, MY_1, MZ_1;
    double XX_1, YY_1, ZZ_1;
    double XY_1, YZ_1, XZ_1;
    double fric_1;
    double XXw, YYw, ZZw;
  };

  struct Forces {
    Eigen::Matrix<double, 3, 3> A;
    Eigen::Matrix<double, 3, 3> C;
    Eigen::Matrix<double, 3, 1> Q;
    Eigen::Matrix<double, 3, 1> Gamma_fric;
  };

  using State = typename Dynamics<T, 8, 2>::State;
  using Control = typename Dynamics<T, 8, 2>::Control;

  void SetParameters(Parameters p) {
    R = p.R, mw = p.mw, L = p.L, g = p.g;
    m_1 = p.m_1;
    MX_1 = p.MX_1, MY_1 = p.MY_1, MZ_1 = p.MZ_1;
    XX_1 = p.XX_1, YY_1 = p.YY_1, ZZ_1 = p.ZZ_1;
    XY_1 = p.XY_1, YZ_1 = p.YZ_1, XZ_1 = p.YZ_1;
    fric_1 = p.fric_1;
    XXw = p.XXw, YYw = p.YYw, ZZw = p.ZZw;

    A(0, 0) = (2 * YYw + R * R * m_1 + 2 * R * R * mw) / (R * R);
    A(0, 1) = MX_1;
    A(1, 0) = MX_1;
    A(2, 2) = XX_1;
    C(0, 0) = 0;
    C(2, 0) = 0;
    C(2, 2) = 0;
    Q(0) = 0;
    Q(1) = 0;
    B(0, 0) = -1 / R;
    B(0, 1) = 0;
    B(1, 0) = 0;
    B(1, 1) = L / (2 * R);
    B(2, 0) = 1;
    B(2, 1) = 0;
    newBu(0) = 0;
  }

  Parameters GetParameters() {
    Parameters p;
    p.R = R, p.mw = mw, p.L = L, p.g = g;
    p.m_1 = m_1;
    p.MX_1 = MX_1, p.MY_1 = MY_1, p.MZ_1 = MZ_1;
    p.XX_1 = XX_1, p.YY_1 = YY_1, p.ZZ_1 = ZZ_1;
    p.XY_1 = XY_1, p.YZ_1 = YZ_1, p.XZ_1 = YZ_1;
    p.fric_1 = fric_1;
    p.XXw = XXw, p.YYw = YYw, p.ZZw = ZZw;
    return p;
  }

  Forces ComputeForces(const Eigen::Ref<const State> &x,
                       const Eigen::Ref<const Control> &u) {
    using std::cos;
    using std::sin;

    xx = x(0);
    psii = x(1);
    q_imu = x(2);
    dx = x(3);
    dpsi = x(4);
    dq_imu = x(5);
    X0 = x(6);
    Y0 = x(7);

    Forces dy_forces;

    dy_forces.A(0, 0) = (2 * YYw + R * R * m_1 + 2 * R * R * mw) / (R * R);
    dy_forces.A(0, 1) = MX_1;
    dy_forces.A(1, 0) = MX_1;
    dy_forces.A(2, 2) = XX_1;
    dy_forces.C(0, 0) = 0;
    dy_forces.C(2, 0) = 0;
    dy_forces.C(2, 2) = 0;
    dy_forces.Q(0) = 0;
    dy_forces.Q(1) = 0;

    dy_forces.A(0, 2) = MY_1 * cos(q_imu) + MZ_1 * sin(q_imu);
    dy_forces.A(1, 1) =
        (L * L * YYw + 4 * R * R * XXw -
         2 * R * R * XXw * pow(cos((2 * xx + L * psii) / (2 * R)), 2) -
         2 * R * R * XXw * pow(cos((2 * xx - L * psii) / (2 * R)), 2) +
         2 * R * R * ZZw * pow(cos((2 * xx + L * psii) / (2 * R)), 2) +
         2 * R * R * ZZw * pow(cos((2 * xx - L * psii) / (2 * R)), 2) +
         L * L * R * R * mw + 2 * R * R * YY_1 * pow(cos(q_imu), 2) +
         2 * R * R * YZ_1 * sin(2 * q_imu) +
         2 * R * R * ZZ_1 * pow(sin(q_imu), 2)) /
        (2 * R * R);
    dy_forces.A(1, 2) = -XY_1 * cos(q_imu) - XZ_1 * sin(q_imu);
    dy_forces.A(2, 0) = MY_1 * cos(q_imu) + MZ_1 * sin(q_imu);
    dy_forces.A(2, 1) = -XY_1 * cos(q_imu) - XZ_1 * sin(q_imu);
    dy_forces.C(0, 1) =
        -(dpsi * (XXw * sin((2 * xx + L * psii) / R) +
                  XXw * sin((2 * xx - L * psii) / R) -
                  ZZw * sin((2 * xx + L * psii) / R) -
                  ZZw * sin((2 * xx - L * psii) / R) -
                  2 * R * MZ_1 * cos(q_imu) + 2 * R * MY_1 * sin(q_imu))) /
        (2 * R);
    dy_forces.C(0, 2) = dq_imu * (MZ_1 * cos(q_imu) - MY_1 * sin(q_imu));
    dy_forces.C(1, 0) =
        (dpsi *
         (sin((2 * xx + L * psii) / R) / 2 + sin((2 * xx - L * psii) / R) / 2) *
         (XXw - ZZw)) /
        R;
    dy_forces.C(1, 1) = (4 * R * MY_1 * sin(q_imu) - 4 * R * MZ_1 * cos(q_imu) +
                         2 * dx * XXw * sin((2 * xx + L * psii) / R) +
                         2 * dx * XXw * sin((2 * xx - L * psii) / R) -
                         2 * dx * ZZw * sin((2 * xx + L * psii) / R) -
                         2 * dx * ZZw * sin((2 * xx - L * psii) / R) +
                         L * XXw * dpsi * sin((2 * xx + L * psii) / R) -
                         L * XXw * dpsi * sin((2 * xx - L * psii) / R) -
                         L * ZZw * dpsi * sin((2 * xx + L * psii) / R) +
                         L * ZZw * dpsi * sin((2 * xx - L * psii) / R) +
                         4 * R * YZ_1 * dq_imu * cos(2 * q_imu) -
                         2 * R * YY_1 * dq_imu * sin(2 * q_imu) +
                         2 * R * ZZ_1 * dq_imu * sin(2 * q_imu)) /
                        (4 * R);
    dy_forces.C(1, 2) = XY_1 * dq_imu * sin(q_imu) -
                        XZ_1 * dq_imu * cos(q_imu) +
                        YZ_1 * dpsi * (2 * pow(cos(q_imu), 2) - 1) -
                        YY_1 * dpsi * cos(q_imu) * sin(q_imu) +
                        ZZ_1 * dpsi * cos(q_imu) * sin(q_imu);
    dy_forces.C(2, 1) =
        -dpsi * (YZ_1 * cos(2 * q_imu) - (YY_1 * sin(2 * q_imu)) / 2 +
                 (ZZ_1 * sin(2 * q_imu)) / 2);
    dy_forces.Q(2) = g * MZ_1 * cos(q_imu) - g * MY_1 * sin(q_imu);
    dy_forces.Gamma_fric(0) = (2 * fric_1 * (dq_imu - dx / R)) / R;
    dy_forces.Gamma_fric(1) = -(L * L * dpsi * fric_1) / (2 * R * R);
    dy_forces.Gamma_fric(2) = -2 * fric_1 * (dq_imu - dx / R);

    return dy_forces;
  }

  inline State f(const Eigen::Ref<const State> &x,
                 const Eigen::Ref<const Control> &u) {
    using std::cos;
    using std::sin;
    // state transition = [thetadot, force, xdot, xddot]
    State xdot;

    Forces dy_forces = ComputeForces(x, u);

    xx = x(0);
    psii = x(1);
    q_imu = x(2);
    dx = x(3);
    dpsi = x(4);
    dq_imu = x(5);
    X0 = x(6);
    Y0 = x(7);

    A = dy_forces.A;
    C = dy_forces.C;
    Q = dy_forces.Q;
    Gamma_fric = dy_forces.Gamma_fric;

    dq << dx, dpsi, dq_imu;
    ddth = u(0);
    tau_0 = u(1);
    // A*ddqVec + C*dqVec + Q - Gamma_fric= [ -tau_1/R; (L*tau_0)/(2*R); tau_1]
    // A*ddqVec + h = [ -tau_1/R; (L*tau_0)/(2*R); tau_1], where
    h = C * dq + Q - Gamma_fric;
    // R times first equation added to third equation results in
    // [R*A(1,1)+A(3,1), R*A(1,2)+A(3,2); A(2,1), A(2,2)]*[ddxx; ddpsi]
    //   + [R*A(1,3)+A(3,3); A(2,3)]*ddth + [R*h(1)+h(3); h(2)] = [0;
    //   (L*tau_0)/(2*R)]
    // newA*[ddxx; ddpsi] + newh = [0; (L*tau_0)/(2*R)], where:
    newA(0, 0) = R * A(0, 0) + A(2, 0);
    newA(0, 1) = R * A(0, 1) + A(2, 1);
    newA(1, 0) = A(1, 0);
    newA(1, 1) = A(1, 1);
    newh(0) = (R * A(0, 2) + A(2, 2)) * ddth + R * h(0) + h(2);
    newh(1) = A(1, 2) * ddth + h(1);
    newBu(1) = (L * tau_0) / (2 * R);
    xdot << dq, newA.colPivHouseholderQr().solve(-newh + newBu), ddth,
        dx * cos(psii), dx * sin(psii);

    return xdot;
  }

  T R, mw, L, g;
  T m_1;
  T MX_1, MY_1, MZ_1;
  T XX_1, YY_1, ZZ_1;
  T XY_1, YZ_1, XZ_1;
  T fric_1;
  T XXw, YYw, ZZw;

  T xx, psii, q_imu;
  T dx, dpsi, dq_imu;
  T X0, Y0;

  T ddth, tau_0;

  Eigen::Matrix<T, 3, 3> A;
  Eigen::Matrix<T, 3, 3> C;
  Eigen::Matrix<T, 3, 1> Q;
  Eigen::Matrix<T, 3, 1> Gamma_fric;
  Eigen::Matrix<T, 3, 2> B;
  Eigen::Matrix<T, 3, 1> h;
  Eigen::Matrix<T, 3, 1> dq;
  Eigen::Matrix<T, 2, 2> newA;
  Eigen::Matrix<T, 2, 1> newh;
  Eigen::Matrix<T, 2, 1> newBu;
};

template <class T>
struct TwipDynamicsCost : public CostFunction<TwipDynamics<T>> {
  using Scalar = T;
  using Dynamics = TwipDynamics<T>;
  using State = typename CostFunction<TwipDynamics<T>>::State;
  using Control = typename CostFunction<TwipDynamics<T>>::Control;
  using Gradient = typename CostFunction<TwipDynamics<T>>::Gradient;
  using Hessian = typename CostFunction<TwipDynamics<T>>::Hessian;
  using StateHessian = Eigen::Matrix<Scalar, TwipDynamics<T>::StateSize,
                                     TwipDynamics<T>::StateSize>;
  using ControlHessian = Eigen::Matrix<Scalar, TwipDynamics<T>::ControlSize,
                                       TwipDynamics<T>::ControlSize>;

  TwipDynamicsCost(const Eigen::Ref<const State> &xf,
                      const Eigen::Ref<const StateHessian> &Q,
                      const Eigen::Ref<const ControlHessian> &R,
                      double negative_theta_penalty_factor = 1.0)
      : CostFunction<TwipDynamics<T>>(xf),
        Q_(Q),
        R_(R),
        negative_theta_penalty_factor_(negative_theta_penalty_factor) {
    QR_.setZero();
    QR_.topLeftCorner(Dynamics::StateSize, Dynamics::StateSize) = Q;
    QR_.bottomRightCorner(Dynamics::ControlSize, Dynamics::ControlSize) = R;
  }

  Scalar c(const Eigen::Ref<const State> &x,
           const Eigen::Ref<const Control> &u) {
    StateHessian Q_local = Q_;
    State error = x - this->target();
    if (error(2) < 0) Q_local(2, 2) = negative_theta_penalty_factor_ * Q_(2, 2);
    return (error.transpose() * Q_local * error).value() +
           (u.transpose() * R_ * u).value();
  }

  Gradient dc(const Eigen::Ref<const State> &x,
              const Eigen::Ref<const Control> &u) {
    Gradient g;
    StateHessian Q_local = Q_;
    State error = x - this->target();
    if (error(2) < 0) Q_local(2, 2) = negative_theta_penalty_factor_ * Q_(2, 2);
    g.head(Dynamics::StateSize) = Q_local * (x - this->target());
    g.tail(Dynamics::ControlSize) = R_ * u;
    return g;
  }

  Hessian d2c(const Eigen::Ref<const State> &x,
              const Eigen::Ref<const Control> &u) {
    Hessian QR_local = QR_;
    State error = x - this->target();
    if (error(2) < 0)
      QR_local(2, 2) = negative_theta_penalty_factor_ * QR_(2, 2);
    return QR_local;
  }

  // Cost Functions which takes reference instead of target
  Scalar c_ref(const Eigen::Ref<const State> &x,
               const Eigen::Ref<const Control> &u,
               const Eigen::Ref<const State> &xf) {
    State error = x - xf;
    return (error.transpose() * Q_ * error).value() +
           (u.transpose() * R_ * u).value();
  }

  Gradient dc_ref(const Eigen::Ref<const State> &x,
                  const Eigen::Ref<const Control> &u,
                  const Eigen::Ref<const State> &xf) {
    Gradient g;
    g.head(Dynamics::StateSize) = Q_ * (x - xf);
    g.tail(Dynamics::ControlSize) = R_ * u;
    return g;
  }

  Hessian d2c_ref(const Eigen::Ref<const State> &x,
                  const Eigen::Ref<const Control> &u,
                  const Eigen::Ref<const State> &xf) {
    return QR_;
  }

 private:
  StateHessian Q_;
  ControlHessian R_;
  Hessian QR_;
  double negative_theta_penalty_factor_;
};

template <class T>
struct TwipDynamicsTerminalCost : public TerminalCostFunction<TwipDynamics<T>> {
  using Scalar = T;
  using Dynamics = TwipDynamics<T>;
  using State = typename TerminalCostFunction<TwipDynamics<T>>::State;
  using Gradient = typename TerminalCostFunction<TwipDynamics<T>>::Gradient;
  using Hessian = typename TerminalCostFunction<TwipDynamics<T>>::Hessian;

  TwipDynamicsTerminalCost(const Eigen::Ref<const State> &xf,
                           const Eigen::Ref<const Hessian> &Q)
      : TerminalCostFunction<TwipDynamics<T>>(xf), Q_(Q) {}

  Scalar c(const Eigen::Ref<const State> &x) {
    return (x - TerminalCostFunction<TwipDynamics<T>>::xf).transpose() * Q_ *
           (x - TerminalCostFunction<TwipDynamics<T>>::xf);
  }

  Gradient dc(const Eigen::Ref<const State> &x) {
    return Q_ * (x - TerminalCostFunction<TwipDynamics<T>>::xf);
  }

  Hessian d2c(const Eigen::Ref<const State> &x) { return Q_; }

 private:
  Hessian Q_;
};

template <class T>
struct TwipDynamicsExt : public Dynamics<T, 10, 2> {
  using State = typename Dynamics<T, 10, 2>::State;
  using Control = typename Dynamics<T, 10, 2>::Control;
  inline State f(const Eigen::Ref<const State> &x,
                 const Eigen::Ref<const Control> &u) {
    State xdot;
    xdot.head(8) = twip_dynamics_.f(x.head(8), x.tail(2));
    xdot.tail(2) = u;
    return xdot;
  }
  TwipDynamics<T> twip_dynamics_;
};

template <class T>
struct TwipDynamicsExtCost : public CostFunction<TwipDynamicsExt<T>> {
  using Scalar = T;
  using Dynamics = TwipDynamicsExt<T>;
  using State = typename CostFunction<TwipDynamicsExt<T>>::State;
  using Control = typename CostFunction<TwipDynamicsExt<T>>::Control;
  using Gradient = typename CostFunction<TwipDynamicsExt<T>>::Gradient;
  using Hessian = typename CostFunction<TwipDynamicsExt<T>>::Hessian;
  using StateHessian = Eigen::Matrix<Scalar, TwipDynamicsExt<T>::StateSize,
                                     TwipDynamicsExt<T>::StateSize>;
  using ControlHessian = Eigen::Matrix<Scalar, TwipDynamicsExt<T>::ControlSize,
                                       TwipDynamicsExt<T>::ControlSize>;

  TwipDynamicsExtCost(const Eigen::Ref<const State> &xf,
                      const Eigen::Ref<const StateHessian> &Q,
                      const Eigen::Ref<const ControlHessian> &R,
                      double negative_theta_penalty_factor = 1.0)
      : CostFunction<TwipDynamicsExt<T>>(xf),
        Q_(Q),
        R_(R),
        negative_theta_penalty_factor_(negative_theta_penalty_factor) {
    QR_.setZero();
    QR_.topLeftCorner(Dynamics::StateSize, Dynamics::StateSize) = Q;
    QR_.bottomRightCorner(Dynamics::ControlSize, Dynamics::ControlSize) = R;
  }

  Scalar c(const Eigen::Ref<const State> &x,
           const Eigen::Ref<const Control> &u) {
    StateHessian Q_local = Q_;
    State error = x - this->target();
    if (error(2) < 0) Q_local(2, 2) = negative_theta_penalty_factor_ * Q_(2, 2);
    return (error.transpose() * Q_local * error).value() +
           (u.transpose() * R_ * u).value();
  }

  Gradient dc(const Eigen::Ref<const State> &x,
              const Eigen::Ref<const Control> &u) {
    Gradient g;
    StateHessian Q_local = Q_;
    State error = x - this->target();
    if (error(2) < 0) Q_local(2, 2) = negative_theta_penalty_factor_ * Q_(2, 2);
    g.head(Dynamics::StateSize) = Q_local * (x - this->target());
    g.tail(Dynamics::ControlSize) = R_ * u;
    return g;
  }

  Hessian d2c(const Eigen::Ref<const State> &x,
              const Eigen::Ref<const Control> &u) {
    Hessian QR_local = QR_;
    State error = x - this->target();
    if (error(2) < 0)
      QR_local(2, 2) = negative_theta_penalty_factor_ * QR_(2, 2);
    return QR_local;
  }

  // Cost Functions which takes reference instead of target
  Scalar c_ref(const Eigen::Ref<const State> &x,
               const Eigen::Ref<const Control> &u,
               const Eigen::Ref<const State> &xf) {
    State error = x - xf;
    return (error.transpose() * Q_ * error).value() +
           (u.transpose() * R_ * u).value();
  }

  Gradient dc_ref(const Eigen::Ref<const State> &x,
                  const Eigen::Ref<const Control> &u,
                  const Eigen::Ref<const State> &xf) {
    Gradient g;
    g.head(Dynamics::StateSize) = Q_ * (x - xf);
    g.tail(Dynamics::ControlSize) = R_ * u;
    return g;
  }

  Hessian d2c_ref(const Eigen::Ref<const State> &x,
                  const Eigen::Ref<const Control> &u,
                  const Eigen::Ref<const State> &xf) {
    return QR_;
  }

 private:
  StateHessian Q_;
  ControlHessian R_;
  Hessian QR_;
  double negative_theta_penalty_factor_;
};

template <class T>
struct TwipDynamicsExtTerminalCost
    : public TerminalCostFunction<TwipDynamicsExt<T>> {
  using Scalar = T;
  using Dynamics = TwipDynamicsExt<T>;
  using State = typename TerminalCostFunction<TwipDynamicsExt<T>>::State;
  using Gradient = typename TerminalCostFunction<TwipDynamicsExt<T>>::Gradient;
  using Hessian = typename TerminalCostFunction<TwipDynamicsExt<T>>::Hessian;

  TwipDynamicsExtTerminalCost(const Eigen::Ref<const State> &xf,
                              const Eigen::Ref<const Hessian> &Q)
      : TerminalCostFunction<TwipDynamicsExt<T>>(xf), Q_(Q) {}

  Scalar c(const Eigen::Ref<const State> &x) {
    return (x - TerminalCostFunction<TwipDynamicsExt<T>>::xf).transpose() * Q_ *
           (x - TerminalCostFunction<TwipDynamicsExt<T>>::xf);
  }

  Gradient dc(const Eigen::Ref<const State> &x) {
    return Q_ * (x - TerminalCostFunction<TwipDynamicsExt<T>>::xf);
  }

  Hessian d2c(const Eigen::Ref<const State> &x) { return Q_; }

 private:
  Hessian Q_;
};

template <class T>
struct CsvWriter {
  using StateTrajectory = Eigen::Matrix<T, 8, Eigen::Dynamic>;
  using ControlTrajectory = Eigen::Matrix<T, 2, Eigen::Dynamic>;
  using State = Eigen::Matrix<T, 8, 1>;
  using Control = Eigen::Matrix<T, 2, 1>;

  CsvWriter() { is_open_ = false; }
  ~CsvWriter() {
    if (is_open_) out_file_.close();
  }

  void OpenFile(char *file_name) {
    out_file_.open(file_name);
    is_open_ = true;
  }

  void SaveStep(State x, Control u, double time, int updating_iteration) {
    for (int j = 0; j < 8; j++) {
      out_file_ << x(j) << ", ";
    }
    out_file_ << u(0) << "," << u(1) << ", " << time << ", "
              << updating_iteration << std::endl;
  }

  void SaveTrajectory(StateTrajectory &xs, ControlTrajectory &us,
                      std::string file_name) {
    std::ofstream out_file;
    out_file.open(file_name);

    int traj_len = xs.cols();
    int u_len = us.cols();

    for (int m = 0; m < traj_len; ++m) {
      for (int j = 0; j < 8; j++) out_file << xs(j, m) << ", ";
      if (m < u_len) {
        out_file << us(0, m) << ", " << us(1, m);
      } else {
        out_file << "0.0, 0.0";
      }
      out_file << std::endl;
    }
    out_file.close();
  }

  std::ofstream out_file_;
  bool is_open_;
};

#endif  // KRANG_BALANCING_DDP_OBJECTS_H_
