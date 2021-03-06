/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Collect all boxes and place them on the plate
 */

// Webots Libraries
#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>

// Youbot Libraries
#include <arm.h>
#include <base.h>
#include <gripper.h>
#include <tiny_math.h>

// Standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Devices
static WbDeviceTag kinect_color;
static WbDeviceTag kinect_range;
static WbDeviceTag lidar;
static WbDeviceTag compass;
static WbDeviceTag gps;

// Constants
#define TIME_STEP 32
#define RED_COLOR 0
#define BLUE_COLOR 1
double distance_arm0_platform = 0.2;
double distance_arm0_robot_center = 0.189;
int n_boxes=11;
double searching_points[8][2] = { {0.0,0.0},
                                  {1.75,0.0},
                                  {-1.6,1.6},
                                  {0.0,1.6},
                                  {1.6,1.6},
                                  {-1.6,-1.6},
                                  {0.0,-1.6},
                                  {1.6,-1.6}};

// Global variables
int stored_boxes[2]={0,0};
int picked_boxes_count=0;
int picked_boxes_color[3]={RED_COLOR,RED_COLOR,RED_COLOR};

double get_box_pos_y(int color){
  if(color==RED_COLOR)
    return 0.7-stored_boxes[color]*0.075;
  return -0.7+stored_boxes[color]*0.075;
}

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int what_color(){
  const unsigned char *image = wb_camera_get_image(kinect_color);
  int width = wb_camera_get_width(kinect_color);
  int height = wb_camera_get_height(kinect_color);
  int red = 0;
  int blue = 0;

  for (int i = width / 3; i < 2 * width / 3; i++)
    for (int j = 7 * height / 8; j < height; j++) {
      red += wb_camera_image_get_red(image, width, i, j);
      blue += wb_camera_image_get_blue(image, width, i, j);
    }

  if (red > 3 * blue)
    return RED_COLOR;
  return BLUE_COLOR;
}

double get_compass_angle(){
  const double *compass_raw_values = wb_compass_get_values(compass);
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_north = {1.0, 0.0};
  double theta = vector2_angle(&v_front, &v_north);
  return theta;
}

double get_distance_of_box(){
  const float *range_image = wb_lidar_get_range_image(lidar);
  return range_image[0]+0.286+0.025;
}

Vector2 get_box_pos(){
  double distance = get_distance_of_box();
  double theta = get_compass_angle();
  const double *gps_raw_values = wb_gps_get_values(gps);
  Vector2 box_pos;
  box_pos.u = gps_raw_values[0] - distance*sin(theta);
  box_pos.v = gps_raw_values[1] + distance*cos(theta);
  return box_pos;
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void high_level_go_to(double x, double y, double a) {
  base_goto_set_target(x, y, a);
  while (!base_goto_reached()) {
    base_goto_run();
    step();
  }
  base_reset();
}

static void turn_around(double x, double y,double direction) {
  high_level_go_to(x, y, direction);
}

double box_orientation(Vector2 box_pos){
  if(box_pos.u<0)
    if(box_pos.v<0)
      return M_PI;
    else
      return 0;
  else
    return -M_PI_2;
}

Vector2 get_target_pos(Vector2 box_pos, double alpha){
  double delta = distance_arm0_platform + distance_arm0_robot_center;

  Vector2 target_pos;

  if (alpha == -M_PI_2)
    target_pos.u = box_pos.u - delta;
  else
    target_pos.u = box_pos.u;

  if (alpha == M_PI)
    target_pos.v = box_pos.v + delta;
  else if (alpha == 0)
    target_pos.v = box_pos.v - delta;
  else
    target_pos.v = box_pos.v;

  return target_pos;
}

static void high_level_grip_box(double y, int level, int column, bool grip) {
  static double h_per_step = 0.002;
  static double box_length = 0.05;
  static double box_gap = 0.002;
  static double platform_height = 0.04;
  static double offset = 0.02;  // security margin

  double x = 0.5 * column * (box_gap + box_length);
  double z = offset + platform_height + (level + 1) * box_length;
  x *= 0.9;  // This fix a small offset that I cannot explain

  if (!grip)
    z += offset;

  // prepare
  arm_set_sub_arm_rotation(ARM5, M_PI_2);
  arm_ik(x, y, 0.20);
  if (grip)
    gripper_release();
  passive_wait(0.5);

  // move the arm down
  double h;
  for (h = 0.2; h > z; h -= h_per_step) {
    arm_ik(x, y, h);
    step();
  }

  passive_wait(0.2);

  // grip or ungrip
  if (grip)
    gripper_set_gap(0.04);
  else
    gripper_release();
  passive_wait(1.0);

  // move the arm up
  for (h = z; h < 0.2; h += h_per_step) {
    arm_ik(x, y, h);
    step();
  }
  arm_set_orientation(ARM_FRONT);
}

static void high_level_stock(int o, bool stock) {
  arm_set_height(ARM_BACK_PLATE_HIGH);
  arm_set_orientation(o);
  passive_wait(4.5);
  if (stock)
    gripper_release();
  else
    gripper_set_gap(0.04);
  passive_wait(1.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(3.0);
}

static void pick_box(){
  picked_boxes_color[picked_boxes_count]=what_color();
  high_level_grip_box(distance_arm0_platform, -1, 0, true);
  picked_boxes_count+=1;
  if(picked_boxes_count==1)
    high_level_stock(ARM_FRONT_LEFT, true);
  else if(picked_boxes_count==2)
    high_level_stock(ARM_FRONT_RIGHT, true);
}

static void place_box(int color){
  high_level_go_to(-1.611,get_box_pos_y(color),M_PI_2);
  high_level_grip_box(distance_arm0_platform, -1, 0, false);
  stored_boxes[color]+=1;
  picked_boxes_count-=1;
}

static void place_all_boxes(double current_pos[3]){
  turn_around(current_pos[0],current_pos[1],M_PI_2);
  if(picked_boxes_count==3)
    place_box(picked_boxes_color[2]);
  if(picked_boxes_count==2){
    high_level_stock(ARM_FRONT_RIGHT, false);
    place_box(picked_boxes_color[1]);
  }
  if(picked_boxes_count==1){
    high_level_stock(ARM_FRONT_LEFT, false);
    place_box(picked_boxes_color[0]);
  }
}

bool find_object(){
  const float *range_image;
  double sec = 13;
  double start_time = wb_robot_get_time();
  do {
    base_turn_left();
    range_image = wb_lidar_get_range_image(lidar);
    if (range_image[0]<1){
      base_reset();
      return true;
    }
    step();
  } while (start_time + sec > wb_robot_get_time());
  base_reset();
  return false;
}

bool find_a_box(){
  bool box_found = false;
  for(int i=0; i< 8; i++){
    if(find_object()){
      box_found = true;
      break;
    }else{
      high_level_go_to(searching_points[i][0], searching_points[i][1], -M_PI_2);
    }
  }
  return box_found;
}

static void move_toward_the_box(){
  double target_pos[3];
  Vector2 box_pos = get_box_pos();
  double alpha = box_orientation(box_pos);
  Vector2 target = get_target_pos(box_pos,alpha);
  target_pos[0] = target.u;
  target_pos[1] = target.v;
  target_pos[2] = alpha;
  high_level_go_to(target_pos[0], target_pos[1], target_pos[2]);
}

static void automatic_behavior() {
  arm_set_height(ARM_HANOI_PREPARE);
  double target_pos[3]={0.0,0.0,0.0};

  while(find_a_box()){
    move_toward_the_box();
    move_toward_the_box();
    pick_box();
    if(picked_boxes_count==3){
      place_all_boxes(target_pos);
      high_level_go_to(0.0, 0.0, -M_PI_2);
    }
  }

  
  if(picked_boxes_count > 0)
    place_all_boxes(target_pos);

  arm_reset();
  high_level_go_to(0.0, 0.0, -M_PI_2);
}

int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();

  kinect_color = wb_robot_get_device("kinect color");
  kinect_range = wb_robot_get_device("kinect range");
  lidar = wb_robot_get_device("lidar");
  compass = wb_robot_get_device("compass");
  gps = wb_robot_get_device("gps");

  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  wb_compass_enable(compass, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);

  passive_wait(1.0);

  automatic_behavior();

  wb_robot_cleanup();

  return 0;
}
