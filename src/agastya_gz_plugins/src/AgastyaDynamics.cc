// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Vaibhav (VaibhavAero108)
// AGASTYA amphibious quadrotor dynamics + manual flight/swim assist.
//
// Runs inside Gazebo Sim on simulation time. Every physics step it:
//   1. computes environment forces (cell-integrated buoyancy of the sealed
//      enclosure + structure volume, air/water drag blended by submersion),
//   2. runs a cascaded position-hold / velocity / geometric attitude
//      controller from the latest manual command,
//   3. allocates per-rotor thrust for the four A2212 + 1045 rotors with
//      first-order motor lag and medium-dependent thrust limits,
//   4. applies a single summed wrench to the base link (ExternalWorldWrenchCmd
//      is cleared by the Physics system after each step, so nothing queues).
//
// Rotor, drag and water-thrust figures are configuration parameters taken
// from nominal datasheet values or explicit assumptions; they are not
// measurements of a built AGASTYA prototype.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace agastya
{
using gz::math::Matrix3d;
using gz::math::Matrix4d;
using gz::math::Quaterniond;
using gz::math::Vector3d;

namespace
{
double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double Lerp(double a, double b, double t) { return a + (b - a) * t; }
double SmoothStep(double t)
{
  t = Clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}
double WrapAngle(double a)
{
  while (a > GZ_PI) a -= 2 * GZ_PI;
  while (a < -GZ_PI) a += 2 * GZ_PI;
  return a;
}
Vector3d Vee(const Matrix3d &s) { return Vector3d(s(2, 1), s(0, 2), s(1, 0)); }
Vector3d Mul(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(a.X() * b.X(), a.Y() * b.Y(), a.Z() * b.Z());
}
Vector3d Vec(const sdf::ElementPtr &sdf, const std::string &name, const Vector3d &def)
{
  return sdf->HasElement(name) ? sdf->Get<Vector3d>(name) : def;
}
double Num(const sdf::ElementPtr &sdf, const std::string &name, double def)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : def;
}
}  // namespace

struct Rotor
{
  Vector3d position;  // propeller hub, link frame
  double spin{1.0};   // +1 CCW seen from above, -1 CW
  double thrust{0.0}; // current (lagged) thrust [N]
  double cmd{0.0};
  double waterFraction{0.0};
};

struct Cell
{
  Vector3d position;  // link frame
  Vector3d halfSize;  // link frame
  double volume{0.0};
};

class AgastyaDynamics : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    auto sdf = _sdf->Clone();
    this->model = gz::sim::Model(_entity);
    const std::string linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->linkEntity = this->model.LinkByName(_ecm, linkName);
    if (this->linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[AgastyaDynamics] link [" << linkName << "] not found\n";
      return;
    }
    this->link = gz::sim::Link(this->linkEntity);
    this->link.EnableVelocityChecks(_ecm, true);

    // Water region (pool) ---------------------------------------------------
    this->surfaceZ = Num(sdf, "water_surface_z", 0.0);
    this->waterMin = Vec(sdf, "water_min", Vector3d(-4, -3, -1.2));
    this->waterMax = Vec(sdf, "water_max", Vector3d(4, 3, 0));
    this->rhoWater = Num(sdf, "water_density", 1000.0);
    this->gravity = Num(sdf, "gravity", 9.81);

    // Rotors -----------------------------------------------------------------
    this->airMaxThrust = Num(sdf, "air_max_thrust", 8.0);
    this->waterMaxThrust = Num(sdf, "water_max_thrust", 3.0);
    this->airTorqueCoeff = Num(sdf, "air_torque_coeff", 0.016);
    this->waterTorqueCoeff = Num(sdf, "water_torque_coeff", 0.03);
    this->motorTau = Num(sdf, "motor_time_constant", 0.04);
    this->rotorBand = Num(sdf, "rotor_transition_band", 0.04);
    if (sdf->HasElement("rotor"))
    {
      for (auto r = sdf->GetElement("rotor"); r; r = r->GetNextElement("rotor"))
      {
        Rotor rotor;
        rotor.position = r->Get<Vector3d>("position");
        rotor.spin = r->Get<double>("spin", 1.0).first;
        this->rotors.push_back(rotor);
      }
    }
    if (this->rotors.size() != 4)
    {
      gzerr << "[AgastyaDynamics] exactly 4 <rotor> elements are required\n";
      this->rotors.clear();
      return;
    }

    // Buoyancy volumes -------------------------------------------------------
    if (sdf->HasElement("buoyancy_box"))
    {
      for (auto b = sdf->GetElement("buoyancy_box"); b; b = b->GetNextElement("buoyancy_box"))
      {
        const Vector3d size = b->Get<Vector3d>("size");
        const Vector3d center = b->Get<Vector3d>("center");
        const int nx = 3, ny = 3, nz = 4;
        const Vector3d cell(size.X() / nx, size.Y() / ny, size.Z() / nz);
        for (int i = 0; i < nx; ++i)
          for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k)
            {
              Cell c;
              c.position = center - size * 0.5 +
                           Vector3d((i + 0.5) * cell.X(), (j + 0.5) * cell.Y(), (k + 0.5) * cell.Z());
              c.halfSize = cell * 0.5;
              c.volume = cell.X() * cell.Y() * cell.Z();
              this->cells.push_back(c);
            }
      }
    }
    if (sdf->HasElement("buoyancy_lumped"))
    {
      for (auto b = sdf->GetElement("buoyancy_lumped"); b; b = b->GetNextElement("buoyancy_lumped"))
      {
        Cell c;
        c.position = b->Get<Vector3d>("center");
        const double h = b->Get<double>("height", 0.05).first;
        c.halfSize = Vector3d(0.0, 0.0, h * 0.5);
        c.volume = b->Get<double>("volume");
        this->cells.push_back(c);
      }
    }
    for (const auto &c : this->cells) this->totalVolume += c.volume;

    // Drag (body frame) --------------------------------------------------------
    this->airLin = Vec(sdf, "air_linear_drag", Vector3d(0.10, 0.10, 0.15));
    this->airQuad = Vec(sdf, "air_quadratic_drag", Vector3d(0.03, 0.03, 0.06));
    this->waterLin = Vec(sdf, "water_linear_drag", Vector3d(1.0, 1.0, 2.0));
    this->waterQuad = Vec(sdf, "water_quadratic_drag", Vector3d(6.0, 7.0, 24.0));
    this->airAng = Vec(sdf, "air_angular_drag", Vector3d(0.002, 0.002, 0.002));
    this->waterAng = Vec(sdf, "water_angular_drag", Vector3d(0.02, 0.02, 0.02));
    this->waterAngQuad = Vec(sdf, "water_angular_quadratic_drag", Vector3d(0.01, 0.01, 0.01));

    // Pilot envelope -------------------------------------------------------
    this->airMaxSpeed = Num(sdf, "air_max_speed", 3.0);
    this->airMaxClimb = Num(sdf, "air_max_climb", 1.5);
    this->airMaxDescent = Num(sdf, "air_max_descent", 1.0);
    this->waterMaxSpeed = Num(sdf, "water_max_speed", 0.4);
    this->waterMaxVertical = Num(sdf, "water_max_vertical", 0.3);
    this->airMaxTilt = Num(sdf, "air_max_tilt", 0.6);
    this->waterMaxTilt = Num(sdf, "water_max_tilt", 0.6);
    this->surfaceMaxTilt = Num(sdf, "surface_max_tilt", 0.25);
    this->maxYawRate = Num(sdf, "max_yaw_rate", 1.5);
    this->cmdTimeout = Num(sdf, "command_timeout", 0.6);

    // Topics -----------------------------------------------------------------
    const std::string ns = sdf->Get<std::string>("namespace", "/agastya").first;
    this->node.Subscribe(ns + "/cmd_vel", &AgastyaDynamics::OnTwist, this);
    this->node.Subscribe(ns + "/mode", &AgastyaDynamics::OnMode, this);
    this->node.Subscribe(ns + "/depth/setpoint", &AgastyaDynamics::OnDepth, this);
    this->statusPub = this->node.Advertise<gz::msgs::StringMsg>(ns + "/status");

    this->configured = true;
    gzmsg << "[AgastyaDynamics] configured: " << this->rotors.size() << " rotors, "
          << this->cells.size() << " buoyancy cells, displaced volume "
          << this->totalVolume * 1000.0 << " L\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (dt <= 0.0) return;
    if (now < this->lastSimTime) this->ResetState();  // world reset
    this->lastSimTime = now;

    // --- state -------------------------------------------------------------
    const auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(this->linkEntity);
    const auto poseOpt = this->link.WorldPose(_ecm);
    if (!inertialComp || !poseOpt) return;
    const gz::math::Inertiald &inertial = inertialComp->Data();
    const double mass = inertial.MassMatrix().Mass();
    const Vector3d com = inertial.Pose().Pos();
    const Matrix3d Ri(inertial.Pose().Rot());
    const Matrix3d J = Ri * inertial.MassMatrix().Moi() * Ri.Transposed();  // link frame
    const auto velOpt = this->link.WorldLinearVelocity(_ecm, com);
    const auto angOpt = this->link.WorldAngularVelocity(_ecm);
    if (!velOpt || !angOpt) return;

    const gz::math::Pose3d &pose = *poseOpt;
    const Quaterniond q = pose.Rot();
    const Matrix3d R(q);
    const Matrix3d Rt = R.Transposed();
    const Vector3d p = pose.Pos() + q.RotateVector(com);
    const Vector3d v = *velOpt;
    const Vector3d wWorld = *angOpt;
    const Vector3d wBody = Rt * wWorld;
    const double yaw = q.Euler().Z();
    const double weight = mass * this->gravity;

    if (!this->initialized)
    {
      this->mass0 = mass;
      double fullBuoy = this->rhoWater * this->gravity * this->totalVolume;
      gzmsg << "[AgastyaDynamics] lumped mass " << mass << " kg, CoM " << com
            << ", weight " << weight << " N, fully submerged buoyancy " << fullBuoy
            << " N (net " << fullBuoy - weight << " N)\n";
      this->ResetTargets(p, yaw);
      this->initialized = true;
    }

    // --- environment forces --------------------------------------------------
    Vector3d forceEnv, torqueEnv;  // world frame, about CoM
    double submerged = 0.0;
    for (const auto &c : this->cells)
    {
      const Vector3d cw = pose.Pos() + q.RotateVector(c.position);
      if (!this->InPool(cw)) continue;
      // vertical half-extent of the (rotated) cell in the world
      const double hz = std::abs(R(2, 0)) * c.halfSize.X() + std::abs(R(2, 1)) * c.halfSize.Y() +
                        std::abs(R(2, 2)) * c.halfSize.Z();
      double frac;
      if (hz < 1e-6)
        frac = cw.Z() < this->surfaceZ ? 1.0 : 0.0;
      else
        frac = Clamp((this->surfaceZ - (cw.Z() - hz)) / (2.0 * hz), 0.0, 1.0);
      if (frac <= 0.0) continue;
      const double vol = c.volume * frac;
      submerged += vol;
      // centre of the submerged part of the cell
      const double zc = cw.Z() - hz + frac * hz;
      const Vector3d at(cw.X(), cw.Y(), zc);
      const Vector3d fb(0, 0, this->rhoWater * this->gravity * vol);
      forceEnv += fb;
      torqueEnv += (at - p).Cross(fb);
    }
    const double sub = this->totalVolume > 0 ? submerged / this->totalVolume : 0.0;
    const Vector3d buoyancyWorld = forceEnv;

    {
      const Vector3d vb = Rt * v;
      const Vector3d lin(Lerp(airLin.X(), waterLin.X(), sub), Lerp(airLin.Y(), waterLin.Y(), sub),
                         Lerp(airLin.Z(), waterLin.Z(), sub));
      const Vector3d quad(Lerp(airQuad.X(), waterQuad.X(), sub), Lerp(airQuad.Y(), waterQuad.Y(), sub),
                          Lerp(airQuad.Z(), waterQuad.Z(), sub));
      Vector3d fb;
      for (int i = 0; i < 3; ++i)
      {
        double f = -(lin[i] * vb[i] + quad[i] * vb[i] * std::abs(vb[i]));
        const double lim = 0.8 * mass * std::abs(vb[i]) / dt;  // never reverse velocity in one step
        fb[i] = Clamp(f, -lim, lim);
      }
      forceEnv += R * fb;

      Vector3d tb;
      for (int i = 0; i < 3; ++i)
      {
        const double lin_a = Lerp(airAng[i], waterAng[i], sub);
        const double quad_a = waterAngQuad[i] * sub;
        double t = -(lin_a * wBody[i] + quad_a * wBody[i] * std::abs(wBody[i]));
        const double lim = 0.8 * J(i, i) * std::abs(wBody[i]) / dt;
        tb[i] = Clamp(t, -lim, lim);
      }
      torqueEnv += R * tb;
    }

    // --- rotor medium ------------------------------------------------------
    double airAuthority = 0.0;
    for (auto &r : this->rotors)
    {
      const Vector3d rw = pose.Pos() + q.RotateVector(r.position);
      const double depth = this->InPool(rw) ? this->surfaceZ - rw.Z() : -1.0;
      r.waterFraction = SmoothStep((depth + 0.5 * this->rotorBand) / this->rotorBand);
      airAuthority += 1.0 - r.waterFraction;
    }
    (void)airAuthority;

    // --- read pilot inputs -------------------------------------------------
    gz::msgs::Twist cmdMsg;
    std::string mode;
    bool depthRequest = false, depthCancel = false;
    double depthValue = 0.0;
    double cmdStamp;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      cmdMsg = this->twist;
      mode = this->mode;
      cmdStamp = this->twistWallStamp;
      if (this->depthPending)
      {
        depthRequest = this->depthValue > 0.0;
        depthCancel = !depthRequest;
        depthValue = this->depthValue;
        this->depthPending = false;
      }
    }
    const double wallNow = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // Commands are refreshed by the operator at a wall-clock rate, so the
    // staleness check is wall-clock; all dynamics use simulation time.
    const bool fresh = (wallNow - cmdStamp) < this->cmdTimeout;
    Vector3d cmdLin = fresh ? Vector3d(cmdMsg.linear().x(), cmdMsg.linear().y(), cmdMsg.linear().z())
                            : Vector3d::Zero;
    double cmdYaw = fresh ? cmdMsg.angular().z() : 0.0;
    if (!std::isfinite(cmdLin.X()) || !std::isfinite(cmdLin.Y()) || !std::isfinite(cmdLin.Z()) ||
        !std::isfinite(cmdYaw))
    {
      cmdLin = Vector3d::Zero;
      cmdYaw = 0.0;
    }

    // controller domain: 0 = air profile, 1 = water profile
    double sigma = sub;
    if (mode == "air") sigma = 0.0;
    else if (mode == "underwater") sigma = 1.0;

    const double maxH = Lerp(this->airMaxSpeed, this->waterMaxSpeed, sigma);
    const double maxUp = Lerp(this->airMaxClimb, this->waterMaxVertical, sigma);
    const double maxDown = Lerp(this->airMaxDescent, this->waterMaxVertical, sigma);
    {
      const double h = std::hypot(cmdLin.X(), cmdLin.Y());
      if (h > maxH) { cmdLin.X(cmdLin.X() * maxH / h); cmdLin.Y(cmdLin.Y() * maxH / h); }
      cmdLin.Z(Clamp(cmdLin.Z(), -maxDown, maxUp));
      cmdYaw = Clamp(cmdYaw, -this->maxYawRate, this->maxYawRate);
    }
    const bool sticksActive = cmdLin.Length() > 0.02 || std::abs(cmdYaw) > 0.02;

    // --- arming / landing state machine -----------------------------------------
    if (mode == "off")
    {
      this->flying = false;
      this->depthHold = false;
    }
    else if (!this->flying)
    {
      const bool wantsUp = cmdLin.Z() > 0.05;
      const bool inWater = sub > 0.2;
      if (wantsUp || depthRequest || (inWater && sticksActive))
      {
        this->flying = true;
        this->ResetTargets(p, yaw);
        if (wantsUp && !inWater) this->target.Z(p.Z() + 0.15);
        this->landedTimer = 0.0;
      }
    }
    if (depthCancel) this->depthHold = false;
    if (depthRequest && this->flying)
    {
      this->depthHold = true;
      this->target.Z(this->surfaceZ - depthValue);
    }

    Vector3d thrustForce, thrustTorque;  // world, about CoM
    double collective = 0.0;
    if (this->flying)
    {
      // body-yaw command -> world
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      Vector3d vCmd(cy * cmdLin.X() - sy * cmdLin.Y(), sy * cmdLin.X() + cy * cmdLin.Y(), cmdLin.Z());
      if (this->depthHold && std::abs(cmdLin.Z()) > 0.02) this->depthHold = false;
      if (this->depthHold) vCmd.Z(0.0);

      // On stick release, re-anchor the hold point at the natural stopping
      // point (v * 0.35 s) instead of the leashed target: no overshoot/bounce.
      const bool hActive = std::hypot(vCmd.X(), vCmd.Y()) > 0.02;
      const bool vActive = std::abs(vCmd.Z()) > 0.02;
      if (this->hWasActive && !hActive)
      {
        this->target.X(p.X() + v.X() * 0.35);
        this->target.Y(p.Y() + v.Y() * 0.35);
      }
      if (this->vWasActive && !vActive && !this->depthHold)
        this->target.Z(p.Z() + v.Z() * 0.25);
      this->hWasActive = hActive;
      this->vWasActive = vActive;

      // position target integrates the command, leashed to the vehicle
      this->target += vCmd * dt;
      const Vector3d leash(0.6, 0.6, this->depthHold ? 2.0 : 0.4);
      for (int i = 0; i < 3; ++i)
        this->target[i] = Clamp(this->target[i], p[i] - leash[i], p[i] + leash[i]);
      this->yawTarget = WrapAngle(this->yawTarget + cmdYaw * dt);
      const double yawErr = WrapAngle(this->yawTarget - yaw);
      if (std::abs(yawErr) > 0.6) this->yawTarget = WrapAngle(yaw + Clamp(yawErr, -0.6, 0.6));

      const Vector3d kp(1.0, 1.0, 1.2);
      Vector3d vDes = vCmd + Mul(kp, this->target - p);
      {
        const double h = std::hypot(vDes.X(), vDes.Y());
        if (h > maxH) { vDes.X(vDes.X() * maxH / h); vDes.Y(vDes.Y() * maxH / h); }
        vDes.Z(Clamp(vDes.Z(), -maxDown, maxUp));
      }
      const Vector3d ev = vDes - v;
      this->integral += ev * dt;
      this->integral.X(Clamp(this->integral.X(), -1.0, 1.0));
      this->integral.Y(Clamp(this->integral.Y(), -1.0, 1.0));
      this->integral.Z(Clamp(this->integral.Z(), -1.5, 1.5));
      const Vector3d kv(Lerp(2.2, 1.6, sigma), Lerp(2.2, 1.6, sigma), Lerp(3.5, 2.5, sigma));
      const Vector3d ki(0.4, 0.4, Lerp(1.0, 0.6, sigma));
      Vector3d aDes = Mul(kv, ev) + Mul(ki, this->integral);
      const double aMaxH = Lerp(4.0, 1.0, sigma);
      {
        const double h = std::hypot(aDes.X(), aDes.Y());
        if (h > aMaxH) { aDes.X(aDes.X() * aMaxH / h); aDes.Y(aDes.Y() * aMaxH / h); }
        aDes.Z(Clamp(aDes.Z(), -4.0, 4.0));
      }

      Vector3d F = (aDes + Vector3d(0, 0, this->gravity)) * mass - buoyancyWorld;
      const double fzMin = Lerp(0.35, 0.02, sigma) * weight;
      F.Z(std::max(F.Z(), fzMin));
      double tilt = Lerp(this->airMaxTilt, this->waterMaxTilt, sigma);
      // Crossing the waterline: keep the vehicle nearly level so all four
      // propellers change medium together (avoids a breach/entry flip).
      if (sub > 0.02 && sub < 0.98) tilt = std::min(tilt, this->surfaceMaxTilt);
      {
        const double h = std::hypot(F.X(), F.Y());
        const double hMax = F.Z() * std::tan(tilt);
        if (h > hMax) { F.X(F.X() * hMax / h); F.Y(F.Y() * hMax / h); }
      }

      // desired attitude (geometric controller on SO(3))
      const Vector3d b3d = F.Normalized();
      const Vector3d b1c(std::cos(this->yawTarget), std::sin(this->yawTarget), 0.0);
      const Vector3d b2d = b3d.Cross(b1c).Normalized();
      const Vector3d b1d = b2d.Cross(b3d);
      const Matrix3d Rd(b1d.X(), b2d.X(), b3d.X(), b1d.Y(), b2d.Y(), b3d.Y(), b1d.Z(), b2d.Z(), b3d.Z());
      const Matrix3d Rdt = Rd.Transposed();
      const Vector3d eR = Vee(Rdt * R - Rt * Rd) * 0.5;
      const Vector3d wDes = Rt * Rd * Vector3d(0, 0, cmdYaw);
      const Vector3d eW = wBody - wDes;
      const Vector3d kR(40.0, 40.0, 8.0), kW(10.0, 10.0, 4.0);
      const Vector3d alpha = -Mul(kR, eR) - Mul(kW, eW);
      const Vector3d tauBody = J * alpha + wBody.Cross(J * wBody);

      const Vector3d b3 = R * Vector3d::UnitZ;
      collective = std::max(0.0, F.Dot(b3));
      this->Allocate(collective, tauBody, com);

      // Touchdown detection (air only). Requires physical evidence of ground
      // support: the controller wants to descend, commands well under hover
      // thrust, yet the vehicle does not move. In free air a thrust deficit
      // of 25 % of weight would accelerate it downward within milliseconds.
      const bool pushingDown = (this->target.Z() - p.Z()) < -0.12;
      if (sub < 0.05 && cmdLin.Z() <= 0.0 && v.Length() < 0.08 && pushingDown &&
          collective < 0.75 * weight && !this->depthHold)
        this->landedTimer += dt;
      else
        this->landedTimer = 0.0;
      if (this->landedTimer > 1.0)
      {
        this->flying = false;
        this->landedTimer = 0.0;
        gzmsg << "[AgastyaDynamics] touchdown detected, motors idle\n";
      }
    }
    if (!this->flying)
    {
      for (auto &r : this->rotors) r.cmd = 0.0;
      this->integral = Vector3d::Zero;
      this->ResetTargets(p, yaw);
    }

    // --- motor lag + rotor wrench -------------------------------------------------
    const double alphaLag = Clamp(dt / std::max(this->motorTau, dt), 0.0, 1.0);
    const Vector3d b3 = R * Vector3d::UnitZ;
    for (auto &r : this->rotors)
    {
      const double fmax = Lerp(this->airMaxThrust, this->waterMaxThrust, r.waterFraction);
      r.thrust += (r.cmd - r.thrust) * alphaLag;
      r.thrust = Clamp(r.thrust, 0.0, fmax);
      const double k = Lerp(this->airTorqueCoeff, this->waterTorqueCoeff, r.waterFraction);
      const Vector3d f = b3 * r.thrust;
      thrustForce += f;
      thrustTorque += (R * (r.position - com)).Cross(f);
      thrustTorque += b3 * (-r.spin * k * r.thrust);
    }

    const Vector3d totalForce = thrustForce + forceEnv;
    const Vector3d totalTorque = thrustTorque + torqueEnv;
    if (totalForce.IsFinite() && totalTorque.IsFinite())
      this->link.AddWorldWrench(_ecm, totalForce, totalTorque, com);

    // --- status (10 Hz sim time) -----------------------------------------------
    if (now - this->lastStatus >= 0.1)
    {
      this->lastStatus = now;
      const char *domain = sub > 0.95 ? "UNDERWATER" : (sub > 0.02 ? "SURFACE" : "AIR");
      const double depth = this->InPool(p) ? std::max(0.0, this->surfaceZ - p.Z()) : 0.0;
      char buf[512];
      std::snprintf(buf, sizeof(buf),
                    "{\"mode\":\"%s\",\"state\":\"%s\",\"domain\":\"%s\",\"submerged\":%.2f,"
                    "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"depth\":%.2f,\"speed\":%.2f,"
                    "\"depth_hold\":%s,\"depth_target\":%.2f,\"thrust\":[%.2f,%.2f,%.2f,%.2f],"
                    "\"collective\":%.2f,\"weight\":%.2f,\"buoyancy\":%.2f,\"sim_time\":%.2f}",
                    mode.c_str(), this->flying ? "FLYING" : "LANDED", domain, sub, p.X(), p.Y(), p.Z(),
                    depth, v.Length(), this->depthHold ? "true" : "false",
                    this->depthHold ? this->surfaceZ - this->target.Z() : 0.0, this->rotors[0].thrust,
                    this->rotors[1].thrust, this->rotors[2].thrust, this->rotors[3].thrust, collective,
                    weight, buoyancyWorld.Z(), now);
      gz::msgs::StringMsg msg;
      msg.set_data(buf);
      this->statusPub.Publish(msg);
    }
  }

private:
  bool InPool(const Vector3d &w) const
  {
    return w.X() >= this->waterMin.X() && w.X() <= this->waterMax.X() && w.Y() >= this->waterMin.Y() &&
           w.Y() <= this->waterMax.Y() && w.Z() >= this->waterMin.Z() - 0.5;
  }

  void ResetTargets(const Vector3d &p, double yaw)
  {
    this->target = p;
    this->yawTarget = yaw;
  }

  void ResetState()
  {
    this->flying = false;
    this->depthHold = false;
    this->integral = Vector3d::Zero;
    this->initialized = false;
    for (auto &r : this->rotors) { r.thrust = 0.0; r.cmd = 0.0; }
  }

  // Thrust allocation with attitude priority over collective, yaw last.
  void Allocate(double collective, const Vector3d &tau, const Vector3d &com)
  {
    Matrix4d A;
    std::array<double, 4> fmax{};
    for (int i = 0; i < 4; ++i)
    {
      const Rotor &r = this->rotors[i];
      const Vector3d d = r.position - com;
      const double k = Lerp(this->airTorqueCoeff, this->waterTorqueCoeff, r.waterFraction);
      A(0, i) = 1.0;
      A(1, i) = d.Y();
      A(2, i) = -d.X();
      A(3, i) = -r.spin * k;
      fmax[i] = Lerp(this->airMaxThrust, this->waterMaxThrust, r.waterFraction);
    }
    // Command all rotors against the weakest rotor's limit: when propellers
    // are in different media the stronger ones must not out-push the rest.
    const double fcap = *std::min_element(fmax.begin(), fmax.end());
    fmax.fill(fcap);
    const Matrix4d Ainv = A.Inverse();
    auto solve = [&](double a, double b, double c, double d) {
      std::array<double, 4> out{};
      for (int i = 0; i < 4; ++i) out[i] = Ainv(i, 0) * a + Ainv(i, 1) * b + Ainv(i, 2) * c + Ainv(i, 3) * d;
      return out;
    };
    const double fmaxMin = *std::min_element(fmax.begin(), fmax.end());
    const auto base = solve(std::min(collective, 4.0 * fmaxMin), 0, 0, 0);
    auto att = solve(0, tau.X(), tau.Y(), 0);
    const auto yawv = solve(0, 0, 0, tau.Z());

    const double spread = *std::max_element(att.begin(), att.end()) - *std::min_element(att.begin(), att.end());
    if (spread > 0.95 * fmaxMin)
      for (auto &a : att) a *= 0.95 * fmaxMin / spread;

    std::array<double, 4> f{};
    double over = 0.0, under = 0.0;
    for (int i = 0; i < 4; ++i)
    {
      f[i] = base[i] + att[i];
      over = std::max(over, f[i] - fmax[i]);
      under = std::max(under, -f[i]);
    }
    const double shift = over > 0.0 ? -over : under;
    for (auto &x : f) x += shift;

    double lambda = 1.0;
    for (int i = 0; i < 4; ++i)
    {
      if (yawv[i] > 1e-9) lambda = std::min(lambda, (fmax[i] - f[i]) / yawv[i]);
      else if (yawv[i] < -1e-9) lambda = std::min(lambda, (0.0 - f[i]) / yawv[i]);
    }
    lambda = Clamp(lambda, 0.0, 1.0);
    for (int i = 0; i < 4; ++i)
      this->rotors[i].cmd = Clamp(f[i] + lambda * yawv[i], 0.0, fmax[i]);
  }

  void OnTwist(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->twist = _msg;
    this->twistWallStamp = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void OnMode(const gz::msgs::StringMsg &_msg)
  {
    std::string m = _msg.data();
    std::transform(m.begin(), m.end(), m.begin(), ::tolower);
    m.erase(0, m.find_first_not_of(" \t\n"));
    m.erase(m.find_last_not_of(" \t\n") + 1);
    if (m == "auto" || m == "air" || m == "underwater" || m == "off")
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->mode = m;
      gzmsg << "[AgastyaDynamics] mode -> " << m << "\n";
    }
    else
    {
      gzwarn << "[AgastyaDynamics] unknown mode [" << m << "] (auto|air|underwater|off)\n";
    }
  }

  void OnDepth(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->depthValue = std::isfinite(_msg.data()) ? Clamp(_msg.data(), 0.0, 1.0) : 0.0;
    this->depthPending = true;
  }

  // configuration
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  gz::sim::Link link{gz::sim::kNullEntity};
  bool configured{false};
  double surfaceZ{0.0}, rhoWater{1000.0}, gravity{9.81};
  Vector3d waterMin, waterMax;
  double airMaxThrust{8.0}, waterMaxThrust{3.0}, airTorqueCoeff{0.016}, waterTorqueCoeff{0.03};
  double motorTau{0.04}, rotorBand{0.04};
  std::vector<Rotor> rotors;
  std::vector<Cell> cells;
  double totalVolume{0.0};
  Vector3d airLin, airQuad, waterLin, waterQuad, airAng, waterAng, waterAngQuad;
  double airMaxSpeed{3.0}, airMaxClimb{1.5}, airMaxDescent{1.0}, waterMaxSpeed{0.4}, waterMaxVertical{0.3};
  double airMaxTilt{0.6}, waterMaxTilt{0.6}, surfaceMaxTilt{0.25}, maxYawRate{1.5}, cmdTimeout{0.6};

  // runtime state
  bool initialized{false}, flying{false}, depthHold{false}, hWasActive{false}, vWasActive{false};
  double mass0{0.0}, lastSimTime{0.0}, lastStatus{-1.0}, landedTimer{0.0}, yawTarget{0.0};
  Vector3d target, integral;

  // transport
  gz::transport::Node node;
  gz::transport::Node::Publisher statusPub;
  std::mutex mutex;
  gz::msgs::Twist twist;
  double twistWallStamp{-1e9};
  std::string mode{"auto"};
  bool depthPending{false};
  double depthValue{0.0};
};
}  // namespace agastya

GZ_ADD_PLUGIN(agastya::AgastyaDynamics, gz::sim::System,
              agastya::AgastyaDynamics::ISystemConfigure,
              agastya::AgastyaDynamics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(agastya::AgastyaDynamics, "agastya::AgastyaDynamics")
