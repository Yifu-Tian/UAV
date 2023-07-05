# Introduction
The position controller mainly consists of two part **positionController** and **velocityController**
# Code
```C++
void PositionControl::_positionController()
{
  // P-position controller
  const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
           _param_mpc_z_p.get()));
  _vel_sp = vel_sp_position + _vel_sp;

  // Constrain horizontal velocity by prioritizing the velocity component along the
  // the desired position setpoint over the feed-forward term.
  const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
           Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
  _vel_sp(0) = vel_sp_xy(0);
  _vel_sp(1) = vel_sp_xy(1);
  // Constrain velocity in z-direction.
  _vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}
```

```C++
void PositionControl::_velocityController(const float &dt)
{
  // Generate desired thrust setpoint.
  // PID
  // u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
  // Umin <= u_des <= Umax
  //
  // Anti-Windup:
  // u_des = _thr_sp; r = _vel_sp; y = _vel
  // u_des >= Umax and r - y >= 0 => Saturation = true
  // u_des >= Umax and r - y <= 0 => Saturation = false
  // u_des <= Umin and r - y <= 0 => Saturation = true
  // u_des <= Umin and r - y >= 0 => Saturation = false
  //
  //   Notes:
  // - PID implementation is in NED-frame
  // - control output in D-direction has priority over NE-direction
  // - the equilibrium point for the PID is at hover-thrust
  // - the maximum tilt cannot exceed 90 degrees. This means that it is
  //    not possible to have a desired thrust direction pointing in the positive
  //    D-direction (= downward)
  // - the desired thrust in D-direction is limited by the thrust limits
  // - the desired thrust in NE-direction is limited by the thrust excess after
  //    consideration of the desired thrust in D-direction. In addition, the thrust in
  //    NE-direction is also limited by the maximum tilt.

  const Vector3f vel_err = _vel_sp - _vel;

  // Consider thrust in D-direction.
  float thrust_desired_D = _param_mpc_z_vel_p.get() * vel_err(2) +  _param_mpc_z_vel_d.get() * _vel_dot(2) + _thr_int(
           2) - _param_mpc_thr_hover.get();

  // The Thrust limits are negated and swapped due to NED-frame.
  float uMax = -_param_mpc_thr_min.get();
  float uMin = -_param_mpc_thr_max.get();

  // make sure there's always enough thrust vector length to infer the attitude
  uMax = math::min(uMax, -10e-4f);

  // Apply Anti-Windup in D-direction.
  bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
             (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

  if (!stop_integral_D) {
    _thr_int(2) += vel_err(2) * _param_mpc_z_vel_i.get() * dt;

    // limit thrust integral
    _thr_int(2) = math::min(fabsf(_thr_int(2)), _param_mpc_thr_max.get()) * math::sign(_thr_int(2));
  }

  // Saturate thrust setpoint in D-direction.
  _thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

  if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
    // Thrust set-point in NE-direction is already provided. Only
    // scaling by the maximum tilt is required.
    float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
    _thr_sp(0) *= thr_xy_max;
    _thr_sp(1) *= thr_xy_max;

  } else {
    // PID-velocity controller for NE-direction.
    Vector2f thrust_desired_NE;
    thrust_desired_NE(0) = _param_mpc_xy_vel_p.get() * vel_err(0) + _param_mpc_xy_vel_d.get() * _vel_dot(0) + _thr_int(0);
    thrust_desired_NE(1) = _param_mpc_xy_vel_p.get() * vel_err(1) + _param_mpc_xy_vel_d.get() * _vel_dot(1) + _thr_int(1);

    // Get maximum allowed thrust in NE based on tilt and excess thrust.
    float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
    float thrust_max_NE = sqrtf(_param_mpc_thr_max.get() * _param_mpc_thr_max.get() - _thr_sp(2) * _thr_sp(2));
    thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

    // Saturate thrust in NE-direction.
    _thr_sp(0) = thrust_desired_NE(0);
    _thr_sp(1) = thrust_desired_NE(1);

    if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
      float mag = thrust_desired_NE.length();
      _thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
      _thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
    }

    // Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
    // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
    float arw_gain = 2.f / _param_mpc_xy_vel_p.get();

    Vector2f vel_err_lim;
    vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
    vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

    // Update integral
    _thr_int(0) += _param_mpc_xy_vel_i.get() * vel_err_lim(0) * dt;
    _thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
  }
}
```
