// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODEL_HPP_
#define PROX_MPC__MODEL_HPP_

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include <prox_mpc/utils.hpp>
#include <prox_mpc/structs.hpp>

namespace prox_mpc
{

/* Robot model class. */
/* Constraints is inherited privately on purpose: only Model's own methods touch
 * the inequality maps, and derived model plugins configure them through Model's
 * public interface. */
class Model : public ModelInfo, private Constraints
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

  /*!
   * Virtual destructor. Model is owned polymorphically through std::shared_ptr<Model>
   * and the pluginlib class loader, so destruction must occur through the base class.
   */
  virtual ~Model() = default;

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
  void setObsAvoid(bool obs_flag);
  void setX(const VectorXd & x);
  void setU(const VectorXd & u);

  /* Virtual set functions */

  /*!
   * Set the local function approximation of the objective function for the QP sub-problem.
   * @param dt step size.
   * @param x_next next state.
   */
  virtual void updatec(double dt, VectorXd x_next) = 0;

  /*!
   * Set the state matrix in the kinematics equality constraint for the QP sub-problem.
   */
  virtual void updateA(double dt) = 0;

  /*!
   * Set the control matrix in the kinematics equality constraint for the QP sub-problem.
   */
  virtual void updateB() = 0;

  /*!
   * Configure the model constants by name after construction. This complements
   * the default constructor required for runtime loading, where parameters cannot
   * be passed in. The default implementation reads no keys; an overriding model
   * applies any present key and keeps its constructor value for any absent key.
   * @param params model constants by name (e.g. "L"); absent keys keep the literal.
   */
  virtual void configure([[maybe_unused]] const std::map<std::string, double> & params) {}

  /*!
   * Map a control vector to a body Twist message.
   * The default is the identity mapping (first control -> linear.x, second
   * control -> angular.z). Models whose control is not a body twist (for example
   * a steering rate) override this.
   * @param u control vector (length m).
   */
  virtual geometry_msgs::msg::Twist toTwist(const VectorXd & u) const
  {
    geometry_msgs::msg::Twist twist;
    if (u.size() > 0) {twist.linear.x = u(0);}
    if (u.size() > 1) {twist.angular.z = u(1);}
    return twist;
  }

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

protected:
  /*!
   * Override one inequality bound from a configure() params map, keeping the
   * current (constructor) value for any side whose key is absent. The bound for
   * idx_vec must already exist (declared in the constructor via setIneq).
   * @param params configure() params map.
   * @param var "x", "u", "du" or "w".
   * @param idx_vec vector index whose bound is overridden.
   * @param key_low params key for the lower bound (kept current if absent).
   * @param key_upp params key for the upper bound (kept current if absent).
   */
  void overrideBound(
    const std::map<std::string, double> & params, std::string var, size_t idx_vec,
    std::string key_low, std::string key_upp);
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODEL_HPP_
