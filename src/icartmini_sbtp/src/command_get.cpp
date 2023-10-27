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

#include <ctime>

#include "trajectory_tracking.h"

void Trajectory_tracking::get_pos_com(double *data, double *resdata, Amr_info* p_amr_info)
{

  resdata[0] = p_amr_info->odom_pose.x;
  resdata[1] = p_amr_info->odom_pose.y;
  resdata[2] = p_amr_info->odom_pose.theta;
  resdata[3] = p_amr_info->odom_pose.time;
  // printf( "get %f %f %f\n", x, y, theta );
}

void Trajectory_tracking::get_vref_com(double *data, double *resdata, Amr_info* p_amr_info)
{
  resdata[0] = p_amr_info->vref;
  resdata[1] = p_amr_info->wref;
  resdata[2] = p_amr_info->odom_pose.time;
}

void Trajectory_tracking::get_vel_com(double *data, double *resdata, Amr_info* p_amr_info)
{
  resdata[0] = p_amr_info->vref_smooth;
  resdata[1] = p_amr_info->wref_smooth;
  resdata[2] = p_amr_info->odom_pose.time;

  // printf("getvel %f %f %f\n",);
}

int Trajectory_tracking::near_pos_com(double *data, double *resdata, Amr_info* p_amr_info)
{
  double x, y, theta, cx, cy;
  double dist;

  x = p_amr_info->odom_pose.x;
  y = p_amr_info->odom_pose.y;
  theta = p_amr_info->odom_pose.theta;

  cx = data[0];
  cy = data[1];
  dist = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));
  resdata[0] = dist;

  if (dist < data[2])
    return 1;
  else
    return 0;
}

int Trajectory_tracking::near_ang_com(double *data, double *resdata, Amr_info* p_amr_info)
{
  double x, y, theta;
  double dist;

  x = p_amr_info->odom_pose.x;
  y = p_amr_info->odom_pose.y;
  theta = p_amr_info->odom_pose.theta;

  dist = trans_q(trans_q(data[0]) - trans_q(theta));
  resdata[0] = dist;

  if (fabs(dist) < data[1])
    return 1;
  else
    return 0;
}

int Trajectory_tracking::over_line_com(double *data, double *resdata, Amr_info* p_amr_info)
{
  double x, y, theta;
  double dist;

  x = p_amr_info->odom_pose.x;
  y = p_amr_info->odom_pose.y;
  theta = p_amr_info->odom_pose.theta;

  dist = (x - data[0]) * cos(data[2]) + (y - data[1]) * sin(data[2]);

  resdata[0] = dist;
  if (dist > 0)
    return 1;
  else
    return 0;
}
