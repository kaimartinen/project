#include "modulation.h"

namespace modulation {


  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

  Modulation::Modulation(Eigen::Vector3d& curr_position, Eigen::VectorXf& curr_speed) :
  modulation_(2,2),
  speed_(curr_speed),
  position_(curr_position)
  {
    modulation_ << 1,0,0,1;
    gripper_position_ << 0.0,0.0,0.0;
  }

  Modulation::Modulation() :
  modulation_(2,2),
  speed_(3),
  position_(3)
  {
    speed_ << 1, 1, 1;
    position_ << 1, 1, 1;
    modulation_ << 1,0,0,1;
    gripper_position_ << 0.0,0.0,0.0;
  }

  Modulation::~Modulation()
  {

  }

  std::vector<ellipse_extraction::Ellipse>& Modulation::getEllipses() {
    return ellipses_;
  }

  void Modulation::setEllipses(std::vector<ellipse_extraction::Ellipse> ellipses) {


    bool irm_modulation = true;
    if(irm_modulation)
    {
      ellipses.push_back(ellipse_extraction::Ellipse(gripper_position_[0],gripper_position_[1],"inner"));
      ellipses.push_back(ellipse_extraction::Ellipse(gripper_position_[0],gripper_position_[1],"outter"));
    }
    ellipses_ = ellipses;
  }


  void Modulation::updateSpeedAndPosition(Eigen::Vector3d& curr_pose, Eigen::VectorXf& curr_speed,Eigen::Vector3d& curr_gripper_pose) {
    position_ = curr_pose;
    speed_ = curr_speed;
    gripper_position_ = curr_gripper_pose;
    for (int k = 0 ; k < ellipses_.size(); k++) {
      if(ellipses_[k].getType() == "outter" || ellipses_[k].getType() == "inner")
      {
        ellipses_[k].setPPoint(gripper_position_[0], gripper_position_[1]);
      }
    }
  }

  void Modulation::computeXiWave() {
    xi_wave_.clear();
    for (int i = 0; i < ellipses_.size(); i++) {
      Eigen::Vector2f pos_ell_frame;
      pos_ell_frame << position_[0] -ellipses_[i].getPPoint()[0] , position_[1] -ellipses_[i].getPPoint()[1];
      pos_ell_frame = ellipses_[i].getR()*pos_ell_frame;
      std::vector<double> xi_wave_i = {pos_ell_frame[0] , pos_ell_frame[1]};
      // std::vector<double> xi_wave_i = {(double)position_[0] - (double)ellipses_[i].getPPoint()[0], (double)position_[1] - (double)ellipsses_[i].getPPoint()[1]};
      xi_wave_.push_back(xi_wave_i);
    }
  }

  void Modulation::computeGamma() {
    gamma_.clear();
    computeXiWave();
    int i = 0;
    bool in_collision = false;
    std::stringstream mes;
    mes << "Gamma: ";
    for (ellipse_extraction::Ellipse ellipse : ellipses_) {
      double gamma_i = pow((xi_wave_[i][0]/ellipse.getHeight()), 2*ellipse.getP1()) + pow((xi_wave_[i][1]/ellipse.getWidth()), 2*ellipse.getP2());

      if(ellipse.getType() == "outter")
      {
        gamma_i = 1.0/gamma_i;
      }
      // double gamma_i = pow((xi_wave_[i][0]/ellipse.getWidth()), 2.0*p_) + pow((xi_wave_[i][1]/ellipse.getHeight()), 2.0*p_);
      if(gamma_i < 1.005) {

        if(ellipse.getType() != "outter")
        {
          gamma_.push_back(1.0001);
          ROS_INFO("Something wrong. Inside obstacle: %d of %d (%s), gamma_i: (%lf), with weight: %g ", i + 1, int(ellipses_.size()),ellipse.getType().c_str(), gamma_i, computeWeight(i));
        }
        else
          gamma_.push_back(1.005);

        in_collision = true;
        gamma_i = 1.0001;
      }
      else
        gamma_.push_back(gamma_i);
      // if(gamma_[i] < 1.01)
      //   ROS_INFO("test");
      // else
        mes  << i << " " << ellipse.getType().c_str() << ": " << gamma_i << " Xi:(" << xi_wave_[i][0] << " " << xi_wave_[i][1] << ") " << std::endl;

      i++;
    }
    // if(!in_collision)
    //   ROS_INFO("all obstacles clear");
    // else
    //   ROS_INFO_STREAM("Something wrong. Inside obstacle");

    // ROS_INFO_STREAM(mes.str());

  }

  double Modulation::computeWeight(int k)
  {
    double w = 1;
    for (int i = 0; i < ellipses_.size(); i++) {
      if (i != k) {
        w = w * ((gamma_[i] - 1)/((gamma_[k] - 1) + (gamma_[i] - 1)));
      }
    }
    return w;
  }

  std::vector<double> Modulation::computeEigenvalue(int k) {
    std::vector<double> lambda;
    double w = computeWeight(k);

    if(ellipses_[k].getType() == "outter")
    {
      lambda = {1.0 - (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho())), 1.0};
      std::vector<double> n = computeHyperplane(k);
      if(n[0] * speed_(7) + n[1] * speed_(8) < 0.0)
      {
        lambda[0] = 2 - lambda[0];
      }
      if(gamma_[k] > 2.7)
      {
        lambda[0] = 1.0;
      }
    }
    else if (ellipses_[k].getType() == "inner")
    {
      lambda = {1.0 - (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho())), 1.0 };
      std::vector<double> n = computeHyperplane(k);
      if(n[0] * speed_(7) + n[1] * speed_(8) > 0.0)
        lambda[0] = 2 - lambda[0];
      // else if ()
      //   lambda[0] = 1.0;
    }
    else
    {
      lambda = {1.0 - (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho())), 1.0 + (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho()))};
    }


    // if (ellipses_.size() > 1) {
    //   lambda = {1.0 - (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho())), 1.0 + (w / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho()))};
    // } else{
    //   lambda = {1.0 - (1.0 / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho())), 1.0 + (1.0 / pow(fabs(gamma_[k]),1.0/ellipses_[k].getRho()))};
    // }

    return lambda;
  }

  Eigen::MatrixXf Modulation::assembleD_k(int k) {
    Eigen::MatrixXf d_k(2,2);
    d_k.setIdentity();
    std::vector<double> lambda = computeEigenvalue(k);

    for(int i = 0; i < 2; i++) {
      // ROS_INFO("Lambda_%d: %g",i,lambda[i]);
      d_k(i, i) = lambda[i];
    }
    return d_k;
  }

  std::vector<double>  Modulation::computeHyperplane(int k) {
    //Derivation of Gamma in ~Xi_i direction
    std::vector<double> n = {(pow(xi_wave_[k][0] / ellipses_[k].getHeight(), 2.0*ellipses_[k].getP1() -1))*2*ellipses_[k].getP1() /ellipses_[k].getHeight(),
      (pow(xi_wave_[k][1] / ellipses_[k].getWidth(), 2.0*ellipses_[k].getP2() -1)) *2*ellipses_[k].getP2() /ellipses_[k].getWidth()};
    return n;
  };

  std::vector<std::vector<double> > Modulation::computeEBase(int k, std::vector<double> normal) {
    int d = 2;
    std::vector<std::vector<double> > base = {{0, 0}};
    for (int i = 1; i <= d - 1; i++) {
      for (int j = 1; j <= d; j++) {
        if (j == 1) {
          base[i-1][j-1] = -normal[i - 1];
        } else if (j == i && i != 1) {
          base[i-1][j-1] = normal[0];
        } else {
          base[i-1][j-1] = 0;
        }
      }
    }
    return base;
  };

  Eigen::MatrixXf Modulation::assembleE_k(int k) {
    Eigen::MatrixXf e_k(2,2);
    std::vector<double> norm = computeHyperplane(k);
    std::vector<std::vector<double> > base = computeEBase(k, norm);
    for (int i = 0; i < 2; i++) {
      e_k(i, 0) = norm[i];
      // e_k(i, 1) = base[0][i];
      if(i==0)
        e_k(i, 1) = norm[1];
      else
        e_k(i, 1) = -norm[0];
    }
    return e_k;
  }

  void Modulation::computeModulationMatrix(Eigen::Matrix2f R_world) {
    modulation_ << 1, 0,
                   0, 1;
    std::stringstream mes;
    computeGamma();
    bool out_mod = false;
    for (int k = 0 ; k < ellipses_.size(); k++) {
      Eigen::MatrixXf d_k = assembleD_k(k);
      Eigen::MatrixXf e_k = assembleE_k(k);
      // Eigen::MatrixXf res = (R_world*ellipses_[k].getR()*e_k * d_k * e_k.inverse()*ellipses_[k].getR().transpose()*R_world.transpose());
      Eigen::MatrixXf res = (ellipses_[k].getR()*e_k * d_k * e_k.inverse()*ellipses_[k].getR().transpose());
      modulation_ = (res * modulation_);
      if(gamma_[k] < 2.0)
        out_mod = false;
        mes << k << ": d_k" << d_k.format(CommaInitFmt) << " e_k" << e_k.format(CommaInitFmt) << " res" << res.format(CommaInitFmt) << std::endl;
    }
    if(out_mod)
    {
      mes << "modulation matrix: " << modulation_.format(CommaInitFmt);
      ROS_INFO_STREAM(mes.str());
    }

  }

  Eigen::VectorXf Modulation::compModulation(Eigen::Matrix2f R_world) {
    computeModulationMatrix(R_world);
    Eigen::VectorXf d2(2);
    d2 << speed_[7], speed_[8];
    d2 = modulation_ * d2;
    speed_(7) = d2[0];
    speed_(8) = d2[1];
    return speed_;
  }

  double computeL2Norm(std::vector<double> v) {
    double res = 0;
    for(double entry : v) {
      res += entry * entry;
    }
    return sqrt(res);
  }

}
