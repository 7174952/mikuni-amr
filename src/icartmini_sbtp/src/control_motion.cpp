// Copyright (c) 2010-2016 The YP-Spur Authors, except where otherwise indicated.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cmath>
#include <cstdio>
#include <cstring>

#include <cstdint>
#include <cstdlib>
#include <ctime>

#include "trajectory_tracking.h"

/* get time stamp */
double Trajectory_tracking::get_time(void)
{
  struct timeval current;

  gettimeofday(&current, NULL);

  return current.tv_sec + current.tv_usec / 1000000.0;
}

/*-PI < theta < PIに調整する*/
double Trajectory_tracking::trans_q(double theta)
{
  while (theta > M_PI)
    theta -= 2.0 * M_PI;
  while (theta < -M_PI)
    theta += 2.0 * M_PI;

  return theta;
}

/* 円弧追従 */
double Trajectory_tracking::circle_follow(Space2D* p_odm, Amr_info* p_amr_info)
{
  double d, q, r, ang, rad;

  r = sqrt((p_amr_info->path_circle.xc - p_odm->x) * (p_amr_info->path_circle.xc - p_odm->x) + (p_amr_info->path_circle.yc - p_odm->y) * (p_amr_info->path_circle.yc - p_odm->y));

  ang = atan2((p_odm->y - p_amr_info->path_circle.yc), (p_odm->x - p_amr_info->path_circle.xc));
  ang = trans_q(ang);

  // レギュレータ問題に変換
  d = fabs(p_amr_info->path_circle.radius) - r;

  q = trans_q(p_odm->theta - (ang + SIGN(p_amr_info->path_circle.radius) * (M_PI / 2.0)));

  if (r < fabs(p_amr_info->path_circle.radius))
  {
    rad = p_amr_info->path_circle.radius;
  }
  else
  {
    rad = SIGN(p_amr_info->path_circle.radius) * r;
  }

  return regurator(d, q, rad, p_amr_info->v_max, p_amr_info->w_max, p_amr_info);
}

/* 直線追従 */
double Trajectory_tracking::line_follow(Space2D* p_odm, Amr_info* p_amr_info)
{
  double d, q;

  d = (p_amr_info->actual_pose.x - p_odm->x) * sin(p_amr_info->actual_pose.theta) - (p_amr_info->actual_pose.y - p_odm->y) * cos(p_amr_info->actual_pose.theta);
  q = p_odm->theta - p_amr_info->actual_pose.theta;
  q = trans_q(q);

  return regurator(d, q, 10000, p_amr_info->v_max, p_amr_info->w_max, p_amr_info);
  // 1000は無限大のつもり(1km)
}

/* 軌跡追従レギュレータ */
double Trajectory_tracking::regurator(double d, double q, double r, double v_max, double w_max, Amr_info* p_amr_info)
{
  double v, w;
  double cd;
  double wref;

  v = v_max - SIGN(v_max) * amr_param.PARAM_L_C1 * fabs(p_amr_info->wref);
  if (v * v_max < 0)
    v = 0;

  wref = v / r;
  if (wref > fabs(w_max))
    wref = fabs(w_max);
  else if (wref < -fabs(w_max))
    wref = -fabs(w_max);

  cd = d;

  if (cd > 0)
    cd = 0;

  if (cd < -amr_param.PARAM_L_DIST)
    cd = -amr_param.PARAM_L_DIST;

  w = p_amr_info->wref_smooth -
      p_amr_info->control_dt * (SIGN(r) * SIGN(v_max) * amr_param.PARAM_L_K1 * cd +
                          amr_param.PARAM_L_K2 * q + amr_param.PARAM_L_K3 * (p_amr_info->wref_smooth - wref));

  if(w > fabs(w_max))
  {
      w = w_max;
  }
  else if(w < -fabs(w_max))
  {
      w = -fabs(w_max);
  }

  p_amr_info->vref = v;
  p_amr_info->wref = w;

  return d;
}

/* 回転 */
double Trajectory_tracking::spin(Space2D* p_odm, Amr_info* p_amr_info)
{
  double w, theta;
  double dt;

  dt = amr_param.PARAM_CONTROL_CYCLE * 1.5;
  theta = p_odm->theta + p_amr_info->wref_smooth * dt;
  w = timeoptimal_servo(
      trans_q(theta - p_amr_info->actual_pose.theta),
      p_amr_info->w_max,
      0,
      p_amr_info->dw);

  p_amr_info->wref = w;
  p_amr_info->vref = 0;
  return fabs(p_odm->theta - p_amr_info->actual_pose.theta);
}

/* 方角 */
double Trajectory_tracking::orient(Space2D* p_odm, Amr_info* p_amr_info)
{
  double w, theta;
  double dt;

  dt = amr_param.PARAM_CONTROL_CYCLE * 1.5;
  theta = p_odm->theta + p_amr_info->wref_smooth * dt;
  w = timeoptimal_servo(
      trans_q(theta - p_amr_info->actual_pose.theta),
      p_amr_info->w_max,
      0,
      p_amr_info->dw);

  p_amr_info->wref = w;
  p_amr_info->vref = p_amr_info->v_max;
  return fabs(p_odm->theta - p_amr_info->actual_pose.theta);
}

/* 点までの距離 */
double Trajectory_tracking::dist_pos(Space2D* p_odm, Amr_info* p_amr_info)
{
  double r;
  r = sqrt((p_amr_info->actual_pose.x - p_odm->x) * (p_amr_info->actual_pose.x - p_odm->x) + (p_amr_info->actual_pose.y - p_odm->y) * (p_amr_info->actual_pose.y - p_odm->y));

  return r;
}

/* 直線まで移動し止まる */
int Trajectory_tracking::stop_line(Space2D* p_odm, Amr_info* p_amr_info)
{
  double a;
  double q;
  double vel;
  int over;
  double x, y;
  double dt;

  dt = amr_param.PARAM_CONTROL_CYCLE * 1.5;
  x = p_odm->x + p_amr_info->vref_smooth * cos(p_odm->theta) * dt;
  y = p_odm->y + p_amr_info->vref_smooth * sin(p_odm->theta) * dt;

  a = (x - p_amr_info->actual_pose.x) * cos(p_amr_info->actual_pose.theta) + (y - p_amr_info->actual_pose.y) * sin(p_amr_info->actual_pose.theta);
  vel = timeoptimal_servo(
      a,
      p_amr_info->v_max,
      0,
      p_amr_info->dv);
  over = 0;

  q = p_odm->theta - p_amr_info->actual_pose.theta;
  q = trans_q(q);
  regurator(0, q, 10000, vel, p_amr_info->w_max, p_amr_info);

  if (a > 0.05)
  {
    // 越えている
    over = 3;
  }
  else if (a < -0.05)
  {
    // まだ
    over = 1;
  }
  else
  {
    // 大体乗った
    over = 2;
  }
  return over;
}

double Trajectory_tracking::timeoptimal_servo(double err, double vel_max, double vel, double acc)
{
  double vel_ref_next;
  double v;
  double _err;

  _err = err + vel * amr_param.PARAM_CONTROL_CYCLE * 1.5;
  if (_err * err < 0)
    _err = 0;

  // 次の目標位置で停止するために必要な現在の速度を計算
  v = sqrt(2 * acc * fabs(_err));
  if (vel_max < v)
  {
    vel_ref_next = -SIGN(_err) * fabs(vel_max);
  }
  else
  {
    vel_ref_next = -SIGN(_err) * v;
  }

  // 次の制御周期で目標値をこえてしまう場合をクリップ
  if ((err + vel_ref_next * amr_param.PARAM_CONTROL_CYCLE * 1.5) * (err) < 0)
  {
    vel_ref_next = -err / amr_param.PARAM_CONTROL_CYCLE;
  }
  return vel_ref_next;
}

double Trajectory_tracking::timeoptimal_servo2(double err, double vel_max, double vel, double acc, double vel_end)
{
  double v;
  double _err;
  double _vel_max;

  _err = err + vel * amr_param.PARAM_CONTROL_CYCLE * 1.5;

  v = sqrt(vel_end * vel_end + 2 * acc * fabs(_err));

  if (fabs(vel_max) < fabs(vel_end))
  {
    if (fabs(err) < (vel_end * vel_end - vel_max * vel_max) / (2.0 * acc))
      _vel_max = fabs(vel_end);
    else
      _vel_max = vel_max;
  }
  else
    _vel_max = vel_max;

  if (_vel_max < v)
  {
    v = _vel_max;
  }
  if (_err > 0)
  {
    v = -v;
  }

  return v;
}
