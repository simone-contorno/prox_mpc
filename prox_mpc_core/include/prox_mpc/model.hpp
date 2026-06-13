// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODEL_HPP_
#define PROX_MPC__MODEL_HPP_

#include <map>
#include <string>
#include <vector>

#include <prox_mpc/utils.hpp>
#include <prox_mpc/structs.hpp>

namespace prox_mpc
{

/* Robot model class. */
class Model : public ModelInfo, Constraints
{
public:
  /* Constructor. */
  Model()
  {
    this->name = "";
    this->obs_flag = false;
    this->map_idx_x = 0;
    this->map_idx_u = 0;
    this->map_idx_du = 0;
    this->map_idx_w = 0;
  }

  /* Set */

  void setName(std::string name);
  void setN(size_t n);
  void setM(size_t m);
  void setParams(const VectorXd & params);
  void setc(const VectorXd & c);
  void setA(const MatrixXd & A);
  void setB(const MatrixXd & B);
  void setIneq(std::string var, size_t idx_vec, double low, double upp);
  void updateIneq(std::string var, size_t idx_vec, double low, double upp);
  void setObsAvoid(bool obs_flag, double obs_dist = 2.);
  void setX(const VectorXd & x);
  void setU(const VectorXd & u);

  /* Set model box dimensions */

  void setBoxWidth(double width);
  void setBoxLength(double length);
  void setPoseWidth(double pose_width);
  void setPoseLength(double pose_length);
  void setBoxPoints(size_t points);

  /* Virtual set functions */

  /*!
   * Set the local function approximation of the objective function for the QP sub-problem.
   * @param dt step size.
   * @param x_next next state.
   */
  virtual void updatec([[maybe_unused]] double dt, [[maybe_unused]] VectorXd x_next) {}

  /*!
   * Set the state matrix in the kinematics equality constraint for the QP sub-problem.
   */
  virtual void updateA([[maybe_unused]] double dt) {}

  /*!
   * Set the control matrix in the kinematics equality constraint for the QP sub-problem.
   */
  virtual void updateB() {}

  /* Get */

  std::string getName();
  size_t getN();
  size_t getM();
  VectorXd getParams();
  VectorXd getX();
  VectorXd getU();
  VectorXd getc();
  MatrixXd getA();
  MatrixXd getB();
  const std::map<int, std::vector<double>> & getIneq(std::string var);
  bool getObsFlag();
  double getObsDist();

  /* Get model box dimensions */

  double getBoxWidth();
  double getBoxLength();
  double getPoseWidth();
  double getPoseLength();
  double getBoxPoints();

private:
  /* Model box */
  double width;        // Model box width.
  double length;       // Model box length.
  double pose_width;   // Pose distance from the model box right side.
  double pose_length;  // Pose distance from the model box back side.
  size_t points;       // Box points for each side.
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODEL_HPP_
