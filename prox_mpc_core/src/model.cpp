// Copyright 2026 Simone Contorno
// SPDX-License-Identifier: Apache-2.0


#include <prox_mpc/model.hpp>

#include <stdexcept>

namespace prox_mpc
{

/********************************************************************************/
/************************************* Get **************************************/
/********************************************************************************/

/* Get the model name. */
std::string Model::getName() {return name;}

/* Get the state dimension. */
size_t Model::getN() {return n;}

/* Get the control input dimension. */
size_t Model::getM() {return m;}

/* Get the model parameters. */
VectorXd Model::getParams() {return params;}

/* Get the state vector. */
VectorXd Model::getX() {return x;}

/* Get the control input vector. */
VectorXd Model::getU() {return u;}

/* Get the local function approximation.. */
VectorXd Model::getc() {return c;}

/* Get the state matrix. */
MatrixXd Model::getA() {return A;}

/* Get the control matrix. */
MatrixXd Model::getB() {return B;}

/*!
 * Get inequality constraints.
 * @param var "x", "u", "du" or "w" respectively for state, control, control derivative and slack variable.
 */
const std::map<int, std::vector<double>> & Model::getIneq(std::string var)
{
  if (var == "x") {return ineq_x;}
  if (var == "u") {return ineq_u;}
  if (var == "du") {return ineq_du;}
  if (var == "w") {return ineq_w;}
  throw std::invalid_argument("Model::getIneq: var must be 'x', 'u', 'du' or 'w'");
}

/* Get obstacle avoidance flag. */
bool Model::getObsFlag() {return obs_flag;}

/* Get obstacle avoidance minimum distance. */
double Model::getObsDist() {return obs_dist;}

/* Get the model box width. */
double Model::getBoxWidth() {return width;}

/* Get the model box length. */
double Model::getBoxLength() {return length;}

/* Get the pose distance from the model box right side. */
double Model::getPoseWidth() {return pose_width;}

/* Get the pose distance from the model box back side. */
double Model::getPoseLength() {return pose_length;}

/* Get the number of box points for each side. */
double Model::getBoxPoints() {return points;}

/********************************************************************************/
/************************************* Set **************************************/
/********************************************************************************/

/*!
 * Set the model name.
 * @param name name.
 */
void Model::setName(std::string name) {this->name = name;}

/*!
 * Set the state vector dimension.
 * @param n dimension (> 0).
 */
void Model::setN(size_t n)
{
  if (n == 0) {throw std::invalid_argument("Model::setN: n must be > 0");}
  this->n = n;
  this->x = VectorXd::Zero(n);
}

/*!
 * Set the control vector dimension.
 * @param m dimension (> 0).
 */
void Model::setM(size_t m)
{
  if (m == 0) {throw std::invalid_argument("Model::setM: m must be > 0");}
  this->m = m;
  this->u = VectorXd::Zero(m);
}

/*!
 * Set the model parameters.
 * @param params vector of parameters.
 */
void Model::setParams(const VectorXd & params) {this->params = params;}

/*!
 * Set local function approximation.
 * @param c
 */
void Model::setc(const VectorXd & c) {this->c = c;}

/*!
 * Set the state matrix.
 * @param A state matrix n x n.
 */
void Model::setA(const MatrixXd & A) {this->A = A;}

/*!
 * Set the control matrix.
 * @param B control matrix n x m.
 */
void Model::setB(const MatrixXd & B) {this->B = B;}

/*!
 * Set the state vector.
 * @param x
 */
void Model::setX(const VectorXd & x) {this->x = x;}

/*!
 * Set the control vector.
 * @param u
 */
void Model::setU(const VectorXd & u) {this->u = u;}

/*!
 * Set inequality constraint.
 * @param var 'x', 'u', 'du' or 'w' respectively for the state, the control, the control derivative and the slack variable.
 * @param idx_vec vector index (> 0).
 * @param low lower bound value.
 * @param upp upper bound value.
 */
void Model::setIneq(std::string var, size_t idx_vec, double low, double upp)
{
  if (low > upp) {throw std::invalid_argument("Model: low must be <= upp");}
  if (var != "x" && var != "u" && var != "du" && var != "w") {
    throw std::invalid_argument("Model: var must be 'x', 'u', 'du' or 'w'");
  }

  std::vector<double> bounds;
  bounds.push_back(idx_vec);
  bounds.push_back(low);
  bounds.push_back(upp);

  if (var == "x") {
    ineq_x[map_idx_x] = bounds;
    map_idx_x++;
  } else if (var == "u") {
    ineq_u[map_idx_u] = bounds;
    map_idx_u++;
  } else if (var == "du") {
    ineq_du[map_idx_du] = bounds;
    map_idx_du++;
  } else if (var == "w") {
    ineq_w[map_idx_w] = bounds;
    map_idx_w++;
  }
}

/*!
 * Update inequality constraint.
 * @param var 'x', 'u', 'du' or 'w' respectively for the state, the control, the control derivative and the slack variable.
 * @param idx_vec vector index (> 0).
 * @param low lower bound value.
 * @param upp upper bound value.
 */
void Model::updateIneq(std::string var, size_t idx_vec, double low, double upp)
{
  if (low > upp) {throw std::invalid_argument("Model: low must be <= upp");}
  if (var != "x" && var != "u" && var != "du" && var != "w") {
    throw std::invalid_argument("Model: var must be 'x', 'u', 'du' or 'w'");
  }

  std::vector<double> bounds;
  bounds.push_back(idx_vec);
  bounds.push_back(low);
  bounds.push_back(upp);

  size_t idx = 0;
  bool found = false;
  if (var == "x") {
    for (size_t i = 0; i < ineq_x.size(); i++) {
      if (ineq_x[i][0] == idx_vec) {idx = i; found = true;}}
    if (found) {ineq_x[idx] = bounds;}
  } else if (var == "u") {
    for (size_t i = 0; i < ineq_u.size(); i++) {
      if (ineq_u[i][0] == idx_vec) {idx = i; found = true;}}
    if (found) {ineq_u[idx] = bounds;}
  } else if (var == "du") {
    for (size_t i = 0; i < ineq_du.size(); i++) {
      if (ineq_du[i][0] == idx_vec) {idx = i; found = true;}}
    if (found) {ineq_du[idx] = bounds;}
  } else if (var == "w") {
    for (size_t i = 0; i < ineq_w.size(); i++) {
      if (ineq_w[i][0] == idx_vec) {idx = i; found = true;}}
    if (found) {ineq_w[idx] = bounds;}
  }
}

/*!
 * Set the obstacle avoidance constraint.
 * @param obs_flag false (inactive) or true (active) (default: false).
 * @param obs_dist minimum distance by the closest obstacle.
 */
void Model::setObsAvoid(bool obs_flag, double obs_dist)
{
  this->obs_flag = obs_flag;
  this->obs_dist = obs_dist;
}

/*!
 * Set the model box width.
 * @param width width (>= 0).
 */
void Model::setBoxWidth(double width)
{
  if (width < 0.) {throw std::invalid_argument("Model::setBoxWidth: width must be >= 0");}
  this->width = width;
}

/*!
 * Set the model box length.
 * @param length length (>= 0).
 */
void Model::setBoxLength(double length)
{
  if (length < 0.) {throw std::invalid_argument("Model::setBoxLength: length must be >= 0");}
  this->length = length;
}

/*!
 * Set the pose distance from the model box right side.
 * @param pose_width distance (>= 0).
 */
void Model::setPoseWidth(double pose_width)
{
  if (pose_width < 0.) {
    throw std::invalid_argument("Model::setPoseWidth: pose_width must be >= 0");
  }
  this->pose_width = pose_width;
}

/*!
 * Set the pose distance from the model box back side.
 * @param pose_length distance (>= 0).
 */
void Model::setPoseLength(double pose_length)
{
  if (pose_length < 0.) {
    throw std::invalid_argument("Model::setPoseLength: pose_length must be >= 0");
  }
  this->pose_length = pose_length;
}

/*!
 * Set the number of model box points for each side.
 * @param points number (>= 0).
 */
void Model::setBoxPoints(size_t points)
{
  this->points = points;
}

}  // namespace prox_mpc
