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

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32

#define RED_COLOR 0
#define BLUE_COLOR 1

double distance_arm0_platform = 0.2;
double distance_arm0_robot_center = 0.189;

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

int what_color(const unsigned char *image){
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

static void high_level_go_to(double x, double y, double a) {
  base_goto_set_target(x, y, a);
  while (!base_goto_reached()) {
    base_goto_run();
    step();
  }
  base_reset();
}

static void high_level_grip_box(double y, int level, int column, bool grip) {
  static double h_per_step = 0.002;
  static double box_length = 0.05;
  static double box_gap = 0.005;
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

static void place_box(int color){
  high_level_go_to(-1.611,get_box_pos_y(color),M_PI_2);
  high_level_grip_box(distance_arm0_platform, -1, 0, false);
  stored_boxes[color]+=1;
}

static void place_all_boxes(){
  while(picked_boxes_count>1){
    place_box(RED_COLOR);
    high_level_stock(ARM_FRONT_RIGHT, false);
    picked_boxes_count-=1;
  }
  place_box(BLUE_COLOR);
  picked_boxes_count=0;
}

static void automatic_behavior(WbDeviceTag kinect_color) {
  double delta = distance_arm0_platform + distance_arm0_robot_center;
  
  double goto_info[3][3] = {{0.75-delta, 0, -M_PI_2},
                            {1-delta,-0.349, -M_PI_2},
                            {-0.943, 1.792-delta, 0}};

  arm_set_height(ARM_HANOI_PREPARE);

  high_level_go_to(goto_info[0][0], goto_info[0][1], goto_info[0][2]);
  high_level_grip_box(distance_arm0_platform, -1, 0, true);
  const unsigned char *image = wb_camera_get_image(kinect_color);
  picked_boxes_color[picked_boxes_count]=what_color(image);
  picked_boxes_count+=1;
  high_level_stock(ARM_FRONT_LEFT, true);
  
  high_level_go_to(goto_info[1][0], goto_info[1][1], goto_info[1][2]);
  high_level_grip_box(distance_arm0_platform, -1, 0, true);
  picked_boxes_count+=1;
  high_level_stock(ARM_FRONT_RIGHT, true);
  
  high_level_go_to(goto_info[2][0], goto_info[2][1], goto_info[2][2]);
  high_level_grip_box(distance_arm0_platform, -1, 0, true);
  picked_boxes_count+=1;

  place_all_boxes();

  // high_level_go_to(goto_info[GOTO_DST][0], goto_info[GOTO_DST][1], goto_info[GOTO_DST][2]);
  // high_level_grip_box(distance_arm0_platform, 0, 0, false);
  
  // end behavior
  arm_reset();
  high_level_go_to(0.0, 0.0, -M_PI_2);
}

int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  base_goto_init(TIME_STEP);
  arm_init();
  gripper_init();

  WbDeviceTag kinect_color = wb_robot_get_device("kinect color");
  WbDeviceTag kinect_range = wb_robot_get_device("kinect range");
  wb_camera_enable(kinect_color, TIME_STEP);
  wb_range_finder_enable(kinect_range, TIME_STEP);

  passive_wait(1.0);

  automatic_behavior();

  wb_robot_cleanup();

  return 0;
}
