// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0

#ifndef PROX_MPC__MODEL_HPP_
#define PROX_MPC__MODEL_HPP_

#include <limits>
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

  void setName(std::string new_name);
  void setN(size_t new_n);
  void setM(size_t new_m);
  void setParams(const VectorXd & new_params);
  void setc(const VectorXd & new_c);
  void setA(const MatrixXd & new_A);
  void setB(const MatrixXd & new_B);
  void setIneq(std::string var, size_t idx_vec, double low, double upp);
  void updateIneq(std::string var, size_t idx_vec, double low, double upp);
  void setObsAvoid(bool new_obs_flag);
  void setX(const VectorXd & new_x);
  void setU(const VectorXd & new_u);

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
   * @param config_params model constants by name (e.g. "L"); absent keys keep the literal.
   */
  virtual void configure(
    [[maybe_unused]] const std::map<std::string, double> & config_params) {}

  /*!
   * Map a control vector to a body Twist message.
   * The default is the identity mapping (first control -> linear.x, second
   * control -> angular.z). Models whose control is not a body twist (for example
   * a steering rate) override this.
   * @param u_in control vector (length m).
   */
  virtual geometry_msgs::msg::Twist toTwist(const VectorXd & u_in) const
  {
    geometry_msgs::msg::Twist twist;
    if (u_in.size() > 0) {twist.linear.x = u_in(0);}
    if (u_in.size() > 1) {twist.angular.z = u_in(1);}
    return twist;
  }

  /*!
   * Declare how this model's state and control vectors map onto the planar
   * quantities a Nav2 consumer drives: which state entries carry x, y and yaw,
   * which control entry carries the signed longitudinal speed, whether a
   * steering angle is carried and where, where the point the state refers to
   * sits in base_link, and the wheelbase the steering geometry is defined on.
   *
   * The default reproduces the convention the bundled models and the bundled
   * controller followed before this hook existed: state [x, y, yaw, (delta)],
   * control [v, delta_dot], the state referenced to base_link, and, for any
   * model carrying more than three states, a steering angle at state index 3
   * whose rate is control index 1. The wheelbase is left at 0, which a consumer
   * reads as undeclared; a model that carries a steering angle must override
   * this and declare one.
   *
   * Read once at the consumer's configuration time, so it is not a control-loop
   * query and may build its result.
   */
  virtual PlanarMapping getPlanarMapping() const
  {
    PlanarMapping mapping;
    if (this->n > 3) {
      mapping.idx_steering = 3;
      if (this->m > 1) {mapping.idx_steer_rate = 1;}
    }
    return mapping;
  }

  /*!
   * Map a body Twist back to a control vector: the inverse of toTwist() for the
   * channels a twist can determine, and a non-finite entry for every channel it
   * cannot. A planar twist carries two usable degrees of freedom, so a model
   * with more controls than that - or one whose control is a rate the twist
   * does not observe, a steering rate for instance - leaves those channels
   * undetermined and the caller decides what to do with them.
   *
   * The default is the inverse of the default toTwist(): linear.x into control
   * 0 and angular.z into control 1, with every further channel undetermined. A
   * model that overrides toTwist() and whose controls are still recoverable
   * from a twist overrides this as well, or the two mappings disagree.
   *
   * Appended last on purpose: it takes the final vtable slot, so an existing
   * plugin keeps the layout it was compiled against for every earlier virtual.
   * @param twist body twist.
   */
  virtual VectorXd fromTwist(const geometry_msgs::msg::Twist & twist) const
  {
    VectorXd u_out = VectorXd::Constant(
      static_cast<Eigen::Index>(this->m), std::numeric_limits<double>::quiet_NaN());
    if (u_out.size() > 0) {u_out(0) = twist.linear.x;}
    if (u_out.size() > 1) {u_out(1) = twist.angular.z;}
    return u_out;
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
   * @param config_params configure() params map.
   * @param var "x", "u", "du" or "w".
   * @param idx_vec vector index whose bound is overridden.
   * @param key_low params key for the lower bound (kept current if absent).
   * @param key_upp params key for the upper bound (kept current if absent).
   */
  void overrideBound(
    const std::map<std::string, double> & config_params, std::string var, size_t idx_vec,
    std::string key_low, std::string key_upp);
};

}  // namespace prox_mpc

#endif  // PROX_MPC__MODEL_HPP_
