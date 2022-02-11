
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

#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <tiny_math.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define TIME_STEP 32

#define RED_COLOR 0
#define BLUE_COLOR 1

double distance_arm0_platform = 0.2;
double distance_arm0_robot_center = 0.189;

int n_boxes=11;
int stored_boxes[2]={0,0};
int picked_boxes_count=0;
int picked_boxes_color[3]={RED_COLOR,RED_COLOR,RED_COLOR};

// static WbDeviceTag compass;

// bool flag_obj=false;

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

int what_color(WbDeviceTag kinect_color){
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

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

// static void rotation() {
//   base_turn_left();
//   //base_set_speeds(0, 0, 3.14);
// }

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

static void pick_box(WbDeviceTag kinect_color){
  picked_boxes_color[picked_boxes_count]=what_color(kinect_color);
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

static void place_all_boxes(double current_pos[2]){
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

static void set_thresh_pos(double goto_info[11][2], int i, double goto_info_alpha){
  double delta = distance_arm0_platform + distance_arm0_robot_center;

  if (goto_info_alpha == -M_PI_2)  goto_info[i][0] -= delta;
  else if (goto_info_alpha == M_PI)  goto_info[i][1] -= delta;
  else if (goto_info_alpha == 0)  goto_info[i][1] -= delta;
  else  goto_info[i][0] -= delta;
}


static void find_obj(){

  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
    const float *range_image = wb_lidar_get_range_image(lidar);
      while (range_image[0]<1){
        rotation();
      }
        flag_obj=true;
}

static void lidar_goto(){

    const double *compass_raw_values = wb_compass_get_values(compass);
    Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};

    // Vector2 v_right = {-v_front.v, v_front.u};
    Vector2 v_north = {1.0, 0.0};

    // compute distance
    // Vector2 v_dir;
    // double distance = vector2_norm(&v_dir);

    // compute absolute angle & delta with the delta with the target angle
    double theta = vector2_angle(&v_front, &v_north);

  WbDeviceTag ds = wb_robot_get_device("ds");
    wb_distance_sensor_enable(ds,TIME_STEP);
    double ds_val = wb_distance_sensor_get_value(ds);

    high_level_go_to(ds_val*cos(theta), ds_val*sin(theta), theta);
}


static void automatic_behavior(WbDeviceTag kinect_color) {
  
  double goto_info_alpha = -M_PI_2;

  double goto_info[11][2] = { {0.75, 0},
                              {1,-0.349},
                              {-0.943, 1.792},
                              {-1.978, 2},
                              {0.465, 2},
                              {1.648, 1.19},
                              {1.349, -0.981},
                              {2, -1.6},
                              {2.125, 0.75},
                              {-0.75, -1.745},
                              {-1.648, -2.19}};

  arm_set_height(ARM_HANOI_PREPARE);

  for (int i = 0; i < n_boxes; i++)
  {
    set_thresh_pos(goto_info, i, goto_info_alpha);

    high_level_go_to(goto_info[i][0], goto_info[i][1], goto_info_alpha);

    // find_obj();
    // lidar_goto();

    if (abs(goto_info[i+1][0]-goto_info[i][0])>0.3 || abs(goto_info[i+1][1]-goto_info[i][1])>0.3){
      if (goto_info[i+1][1]-goto_info[i][1] >= abs(goto_info[i+1][0]-goto_info[i][0]))
          goto_info_alpha = -M_PI_2;  /////
      else if (abs(goto_info[i+1][1]-goto_info[i][1]) <= (goto_info[i+1][0]-goto_info[i][0]))
          goto_info_alpha = M_PI;
      else if (abs(goto_info[i+1][1]-goto_info[i][1]) <= -(goto_info[i+1][0]-goto_info[i][0]))
          goto_info_alpha = 0;
      else
          goto_info_alpha = -M_PI_2;
    }


    pick_box(kinect_color);
    if(picked_boxes_count==3 || i==n_boxes-1){
      place_all_boxes(goto_info[i]);
      if(i<n_boxes-1){
        turn_around(-1.611,get_box_pos_y(picked_boxes_color[0]),goto_info_alpha);
        goto_info[i][0] = -1.611;
        goto_info[i][1] = get_box_pos_y(picked_boxes_color[0]);
        //turn_around(goto_info[i][0], goto_info[i][1], goto_info_alpha);
      }
    }
  }

  arm_reset();
  high_level_go_to(0.0, 0.0, -M_PI_2);
}


int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();
  passive_wait(1.0);

  WbDeviceTag kinect_color = wb_robot_get_device("kinect color");
  WbDeviceTag kinect_range = wb_robot_get_device("kinect range");
  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);

  automatic_behavior(kinect_color);

  wb_robot_cleanup();

  return 0;
}

