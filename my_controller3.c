
#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32


#include "arm.h"

#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/keyboard.h>
#include <webots/supervisor.h>


#include <math.h>
#include <stdio.h>

static WbDeviceTag arm_elements[5];

static enum Height current_height = ARM_RESET;
static enum Orientation current_orientation = ARM_FRONT;

enum Height new_height;
enum Orientation new_orientation;

void arm_init() {
  arm_elements[ARM1] = wb_robot_get_device("arm1");
  arm_elements[ARM2] = wb_robot_get_device("arm2");
  arm_elements[ARM3] = wb_robot_get_device("arm3");
  arm_elements[ARM4] = wb_robot_get_device("arm4");
  arm_elements[ARM5] = wb_robot_get_device("arm5");

  //wb_motor_set_velocity(arm_elements[ARM2], 0.5);

  //arm_set_height(ARM_RESET);
  //arm_set_orientation(ARM_FRONT);
}

void arm_reset() {
  arm_set_height(ARM_RESET);
  arm_set_orientation(ARM_FRONT);
}

void arm_set_height(enum Height height) {
  switch (height) {
    case ARM_FRONT_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], -1.53);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_FRONT_CARDBOARD_BOX:
      wb_motor_set_position(arm_elements[ARM2], 0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_RESET:
      wb_motor_set_position(arm_elements[ARM2], 1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.635);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], -1.0);
      break;
    case ARM_BACK_PLATE_HIGH:
      wb_motor_set_position(arm_elements[ARM2], 0.678);
      wb_motor_set_position(arm_elements[ARM3], 0.682);
      wb_motor_set_position(arm_elements[ARM4], 1.74);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_BACK_PLATE_LOW:
      wb_motor_set_position(arm_elements[ARM2], 0.92);
      wb_motor_set_position(arm_elements[ARM3], 0.42);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_HANOI_PREPARE:
      wb_motor_set_position(arm_elements[ARM2], -0.4);
      wb_motor_set_position(arm_elements[ARM3], -1.2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI_2);
      wb_motor_set_position(arm_elements[ARM5], M_PI_2);
      break;
    default:
      fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = height;
}

void arm_set_orientation(enum Orientation orientation) {
  switch (orientation) {
    case ARM_BACK_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -2.949);
      break;
    case ARM_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -M_PI_2);
      break;
    case ARM_FRONT_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -0.2);
      break;
    case ARM_FRONT:
      wb_motor_set_position(arm_elements[ARM1], 0.0);
      break;
    case ARM_FRONT_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 0.2);
      break;
    case ARM_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], M_PI_2);
      break;
    case ARM_BACK_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 2.949);
      break;
    default:
      fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

void arm_increase_height() {
  new_height = current_height + 1;

  // Prevents from going beyond index.
  if (new_height >= ARM_MAX_HEIGHT)
    new_height = ARM_MAX_HEIGHT - 1;

  // Prevents self-colliding poses.
  if (new_height == ARM_FRONT_FLOOR) {
    if (current_orientation == ARM_BACK_LEFT || current_orientation == ARM_BACK_RIGHT)
      new_height = current_height;
  }

  arm_set_height(new_height);
}

void arm_decrease_height() {
  new_height = current_height - 1;
  if ((int)new_height < 0)
    new_height = 0;
  arm_set_height(new_height);
}

void arm_increase_orientation() {
  new_orientation = current_orientation + 1;

  // Prevents from going beyond index.
  if (new_orientation >= ARM_MAX_SIDE)
    new_orientation = ARM_MAX_SIDE - 1;

  // Prevents self-colliding poses.
  if (new_orientation == ARM_BACK_LEFT) {
    if (current_height == ARM_FRONT_FLOOR)
      new_orientation = current_orientation;
  }

  arm_set_orientation(new_orientation);
}

void arm_decrease_orientation() {
  new_orientation = current_orientation - 1;

  // Prevents from going beyond index.
  if ((int)new_orientation < 0)
    new_orientation = 0;

  // Prevents self-colliding poses.
  if (new_orientation == ARM_BACK_RIGHT) {
    if (current_height == ARM_FRONT_FLOOR)
      new_orientation = current_orientation;
  }

  arm_set_orientation(new_orientation);
}

void arm_set_sub_arm_rotation(enum Arm arm, double radian) {
  wb_motor_set_position(arm_elements[arm], radian);
}

double arm_get_sub_arm_length(enum Arm arm) {
  switch (arm) {
    case ARM1:
      return 0.253;
    case ARM2:
      return 0.155;
    case ARM3:
      return 0.135;
    case ARM4:
      return 0.081;
    case ARM5:
      return 0.105;
  }
  return 0.0;
}

void arm_ik(double x, double y, double z) {
  double y1 = sqrt(x * x + y * y);
  double z1 = z + arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5) - arm_get_sub_arm_length(ARM1);

  double a = arm_get_sub_arm_length(ARM2);
  double b = arm_get_sub_arm_length(ARM3);
  double c = sqrt(y1 * y1 + z1 * z1);

  double alpha = -asin(x / y1);
  double beta = -(M_PI_2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(z1 / y1));
  double gamma = -(M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b)));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;

  wb_motor_set_position(arm_elements[ARM1], alpha);
  wb_motor_set_position(arm_elements[ARM2], beta);
  wb_motor_set_position(arm_elements[ARM3], gamma);
  wb_motor_set_position(arm_elements[ARM4], delta);
  wb_motor_set_position(arm_elements[ARM5], epsilon);
}

 
//////////////////////////////////////////
#include "base.h"

#include "tiny_math.h"

#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define SPEED 4.0
#define MAX_SPEED 0.3
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0

typedef struct {
  Vector2 v_target;
  double alpha;
  bool reached;
} goto_struct;

static WbDeviceTag wheels[4];
static WbDeviceTag gps;
static WbDeviceTag compass;
static goto_struct goto_data;

static double robot_vx = 0.0;
static double robot_vy = 0.0;
static double robot_omega = 0.0;

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_init() {
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
  robot_vx = 0.0;
  robot_vy = 0.0;
  robot_omega = 0.0;
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_move(double vx, double vy, double omega) {
  double speeds[4];
  speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega);
  speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega);
  speeds[2] = 1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega);
  speeds[3] = 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega);
  base_set_wheel_speeds_helper(speeds);
  printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
}

void base_forwards_increment() {
  robot_vx += SPEED_INCREMENT;
  robot_vx = robot_vx > MAX_SPEED ? MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_backwards_increment() {
  robot_vx -= SPEED_INCREMENT;
  robot_vx = robot_vx < -MAX_SPEED ? -MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_left_increment() {
  robot_omega += SPEED_INCREMENT;
  robot_omega = robot_omega > MAX_SPEED ? MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_right_increment() {
  robot_omega -= SPEED_INCREMENT;
  robot_omega = robot_omega < -MAX_SPEED ? -MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_left_increment() {
  robot_vy += SPEED_INCREMENT;
  robot_vy = robot_vy > MAX_SPEED ? MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_right_increment() {
  robot_vy -= SPEED_INCREMENT;
  robot_vy = robot_vy < -MAX_SPEED ? -MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_goto_init(double time_step) {
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps)
    wb_gps_enable(gps, time_step);
  if (compass)
    wb_compass_enable(compass, time_step);
  if (!gps || !compass)
    fprintf(stderr, "cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double y, double alpha) {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = x;
  goto_data.v_target.v = y;
  goto_data.alpha = alpha;
  goto_data.reached = false;
}

void base_goto_run() {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);

  // compute 2d vectors
  Vector2 v_gps = {gps_raw_values[0], gps_raw_values[1]};
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle(&v_front, &v_north);
  double delta_angle = theta - goto_data.alpha;

  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = -v_right.u;
  transform.a.v = v_front.u;
  transform.b.u = v_right.v;
  transform.b.v = -v_front.v;
  transform.c.u = v_right.u * v_gps.u - v_right.v * v_gps.v;
  transform.c.v = -v_front.u * v_gps.u + v_front.v * v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

  // compute the speeds
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  // -> first stimulus: delta_angle
  speeds[0] = -delta_angle / M_PI * K1;
  speeds[1] = delta_angle / M_PI * K1;
  speeds[2] = -delta_angle / M_PI * K1;
  speeds[3] = delta_angle / M_PI * K1;

  // -> second stimulus: u coord of the relative target vector
  speeds[0] += v_target_rel.u * K2;
  speeds[1] += v_target_rel.u * K2;
  speeds[2] += v_target_rel.u * K2;
  speeds[3] += v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] += -v_target_rel.v * K3;
  speeds[1] += v_target_rel.v * K3;
  speeds[2] += v_target_rel.v * K3;
  speeds[3] += -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i = 0; i < 4; i++) {
    speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
    speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)

    // added an arbitrary factor increasing the convergence speed
    speeds[i] *= 30.0;
    speeds[i] = bound(speeds[i], -SPEED, SPEED);
  }
  base_set_wheel_speeds_helper(speeds);

  // check if the taget is reached
  if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
    goto_data.reached = true;
}

bool base_goto_reached() {
  return goto_data.reached;
}
/////////////////////////////////////////////////////////////

void vector3_set_values(Vector3 *vect, double u, double v, double w) {
  vect->u = u;
  vect->v = v;
  vect->w = w;
}

void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw) {
  vector3_set_values(&(m->a), au, av, aw);
  vector3_set_values(&(m->b), bu, bv, bw);
  vector3_set_values(&(m->c), cu, cv, cw);
}

void matrix33_set_identity(Matrix33 *m) {
  matrix33_set_values(m, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v) {
  res->u = m->a.u * v->u + m->b.u * v->v + m->c.u * v->w;
  res->v = m->a.v * v->u + m->b.v * v->v + m->c.v * v->w;
  res->w = m->a.w * v->u + m->b.w * v->v + m->c.w * v->w;
}

double vector2_norm(const Vector2 *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double vector2_angle(const Vector2 *v1, const Vector2 *v2) {
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}


////////////////////////////////
#include <webots/motor.h>
#include <webots/robot.h>

#include "tiny_math.h"

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.025
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers;

void gripper_init() {
  fingers = wb_robot_get_device("finger::left");

  wb_motor_set_velocity(fingers, 0.03);
}

void gripper_grip() {
  wb_motor_set_position(fingers, MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers, MAX_POS);
}

void gripper_set_gap(double gap) {
  double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers, v);
}


///////////////////////////////////////////////////////

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

#include <webots/motor.h>
#include <webots/distance_sensor.h>
#define MAX_FORCE 10.0  // Maximum force to apply
#define DISTANCE_THRESHOLD 0.3  // Distance threshold for suction
#include <webots/vacuum_gripper.h>


static void grab() {
  //double suction_force = 10.0;
  printf("grip");
  WbDeviceTag gripper = wb_robot_get_device("vacuum_gripper");
  wb_vacuum_gripper_enable_presence(gripper, TIME_STEP);
  wb_vacuum_gripper_turn_on(gripper);
}

static void drop() {
  //double suction_force = 10.0;
  WbDeviceTag gripper = wb_robot_get_device("vacuum_gripper");
  wb_vacuum_gripper_turn_off(gripper);
}

/*

static void automatic_behavior() {
  
  passive_wait(2.0);
  printf("??????\n");
 // gripper_release();
  printf("yo?\n");
 // grab();
 // passive_wait(4.0);
 // grab();
 // passive_wait(4.0);
  printf("F\n");
 // arm_set_height(ARM_FRONT_CARDBOARD_BOX);
  printf("here1\n");
  //grab();
  passive_wait(4.0);
  gripper_grip();
  passive_wait(1.0);
  arm_set_height(ARM_BACK_PLATE_LOW);
  passive_wait(3.0);
  gripper_release();
  passive_wait(1.0);
  arm_reset();
  printf("strafe\n\n");
  base_strafe_left();
  passive_wait(5.0);
  gripper_grip();
  base_reset();
  passive_wait(1.0);
  base_turn_left();
  passive_wait(1.0);
  base_reset();
  gripper_release();
  printf("???\n\n");
  arm_set_height(ARM_BACK_PLATE_LOW);
  printf("here2\n");
  passive_wait(3.0);
  gripper_grip();
  passive_wait(1.0);
  printf("---\n\n");
  arm_set_height(ARM_RESET);
  passive_wait(2.0);
  printf("xxx\n\n");
  arm_set_height(ARM_FRONT_PLATE);
  printf("Final?\n\n");
  arm_set_orientation(ARM_RIGHT);
  passive_wait(4.0);
   wb_motor_set_position(arm_elements[ARM4], 0.40);
   passive_wait(2.0);
   drop();
    printf("What?\n\n");
 // arm_set_height(ARM_FRONT_FLOOR);
 // passive_wait(2.0);
  //wb_motor_set_position(arm_elements[ARM3], 0.05);
 // wb_motor_set_position(arm_elements[ARM4], -2.85);
   passive_wait(2.0);
 
  printf("here3\n");
  gripper_release();
  passive_wait(1.0);
   drop();
  arm_set_height(ARM_FRONT_PLATE);
  passive_wait(2.0);
  arm_set_height(ARM_RESET);
  passive_wait(2.0);
  arm_reset();
  gripper_grip();
  passive_wait(2.0);
}*/

static void display_helper_message() {
  printf("\n \nControl commands:\n");
  printf(" Arrows:         Move the robot\n");
  printf(" Page Up/Down:   Rotate the robot\n");
  printf(" +/-:            (Un)grip\n");
  printf(" Shift + arrows: Handle the arm\n");
  printf(" Space:          Reset\n");
  
}




#define ROTATION_INCREMENT 0.1  // Adjust this value as needed
static double angle = 0.0;
static double angle2 = 0.0;
//static double translator = 0.0;
#define TRANSLATION_INCREMENT 0.1

static void rotational_actuator1() {
  WbDeviceTag motor = wb_robot_get_device("rotational_actuator");
  printf("rotational actuator");
  
  angle += ROTATION_INCREMENT;
  wb_motor_set_position(motor, angle);

}

static void rotational_actuator2() {
  WbDeviceTag motor2 = wb_robot_get_device("rotational_actuator2");
  printf("rotational actuator - 2");
  
  angle2 += ROTATION_INCREMENT;
  wb_motor_set_position(motor2, angle2);

}

static void nail_gunF() {
  WbNodeRef node = wb_supervisor_node_get_from_def("nail_gun");
  if (node == NULL) {
    printf("Node not found. Please check the DEF name.\n");
    //wb_robot_cleanup();
    //return 1;
  }
  WbFieldRef translationField = wb_supervisor_node_get_field(node, "translation");
  printf("nail_gun\n");
  
  const double *current_translation = wb_supervisor_field_get_sf_vec3f(translationField);
        
      // Create a new translation vector
  double new_translation[3]; 
  new_translation[0] = current_translation[0]; //  Move forward in the x-direction
  new_translation[1] = current_translation[1];   
  new_translation[2] = current_translation[2] + 0.05;
      
      // Set the new translation
  wb_supervisor_field_set_sf_vec3f(translationField, new_translation);

}

static void nail_gunB() {
  WbNodeRef node = wb_supervisor_node_get_from_def("nail_gun");
  if (node == NULL) {
    printf("Node not found. Please check the DEF name.\n");
    //wb_robot_cleanup();
    //return 1;
  }
  WbFieldRef translationField = wb_supervisor_node_get_field(node, "translation");
  printf("nail_gun\n");
  
  const double *current_translation = wb_supervisor_field_get_sf_vec3f(translationField);
        
      // Create a new translation vector
  double new_translation[3]; 
  new_translation[0] = current_translation[0]; //  Move forward in the x-direction
  new_translation[1] = current_translation[1];   
  new_translation[2] = current_translation[2] - 0.005;
      
      // Set the new translation
  wb_supervisor_field_set_sf_vec3f(translationField, new_translation);

}

/*
static void perform_construction_task() {
  // Sequence of operations to construct the timber structure
  // Example: Picking up a panel and placing it on a target location

  // Move to the panel location
   base_move(0.1, 0.0, 0.0);
   passive_wait(3.0);
   base_reset();
  // base_move(0.0, 0.6, 0.0);
  // passive_wait(3.0);
  // base_reset();

  // Lower the pa
  
  // Lower the arm to the panel and grab
  arm_set_height(ARM_FRONT_FLOOR);
  grab();
  passive_wait(5.0);
  arm_set_height(ARM_RESET);
  passive_wait(2.0);
  grab();

  // Move to the target location
  base_move(0.1, 0.0, 0.0);
  passive_wait(3.0);
  base_reset();

  // Move right at 0.5 m/s for 3 seconds
  base_move(0.0, 0.5, 0.0);
  passive_wait(3.0);
  base_reset();

  // Lower the panel
  arm_set_height(ARM_FRONT_FLOOR);
//  
  passive_wait(5.0);
  //base_reset();
  grab();
   passive_wait(2.0);
   
   arm_set_height(ARM_FRONT_CARDBOARD_BOX);
  printf("here1\n");
   
   
   //drop();
   //passive_wait(2.0);
 
  // base_reset();
  // printf("made it");
  
 

}*/

// static void perform_construction_task2() {
 // Move to the target location
  // base_move(0.1, 0.0, 0.0);
  // passive_wait(3.0);
  // base_reset();

// }

void set_position(WbNodeRef node, double x, double y, double z) {
  // Get the field that contains the position of the node
  WbFieldRef translationField = wb_supervisor_node_get_field(node, "translation");

  // Create a new position array
  const double newPosition[3] = {x, y, z};

  // Set the new position
  wb_supervisor_field_set_sf_vec3f(translationField, newPosition);
}

void set_rotation(WbNodeRef node, double x, double y, double z, double angle) {
  WbFieldRef rotationField = wb_supervisor_node_get_field(node, "rotation");
  const double newRotation[4] = {x, y, z, angle};
  wb_supervisor_field_set_sf_rotation(rotationField, newRotation);
}

void disable_physics(WbNodeRef node) {
  WbFieldRef physicsField = wb_supervisor_node_get_field(node, "physics");
  if (physicsField) {
    WbNodeRef physicsNode = wb_supervisor_field_get_sf_node(physicsField);
    if (physicsNode) {
      wb_supervisor_node_remove(physicsNode);
    }
  }
}

void enable_physics(WbNodeRef node) {
  WbFieldRef physicsField = wb_supervisor_node_get_field(node, "physics");
  // Create a string that defines the Physics node with desired properties
    wb_supervisor_field_import_sf_node_from_string(physicsField, "Physics {}");
    WbNodeRef physicsNode = wb_supervisor_field_get_sf_node(physicsField);
     WbFieldRef massField = wb_supervisor_node_get_field(physicsNode, "mass");
  wb_supervisor_field_set_sf_float(massField, -1.0);

  // Set the density of the physics node
  WbFieldRef densityField = wb_supervisor_node_get_field(physicsNode, "density");
  wb_supervisor_field_set_sf_float(densityField, 1000.0);
  
}



int main(int argc, char **argv) {


 
  wb_robot_init();
  passive_wait(2.0);
   base_init();
  arm_init();
  printf("arm_innit\n");
  gripper_init();
  printf("gripper_innit\n");
  
  printf("xaxa2\n");
  WbNodeRef youbot = wb_supervisor_node_get_from_def("youbot");
  
  for (int i = 1; i<6; i++) {
  
  char stair_name[10];
    sprintf(stair_name, "stair%d", i);
  WbNodeRef stair = wb_supervisor_node_get_from_def(stair_name);
 
  printf("xaxa1\n");
  wb_supervisor_node_save_state(youbot, "start_state");
  char newStateName[20];
  grab();
  passive_wait(2.0);
  wb_motor_set_position(arm_elements[ARM3], -2.004);
  passive_wait(1.0);
  
  disable_physics(stair);
  passive_wait(1.0);
  set_position(stair, -1.68+((i-1)*0.15), 7.93, -0.759+((i-1)*0.095));
  set_rotation(stair, 0.0267, 1, -0.000913, -3.01);
  passive_wait(2.0);

  set_position(youbot, -1.79341 + ((i)*0.208), 8.40292, -0.839567 + ((i)*0.118901));  
  // if (i < 2) {
  // passive_wait(1.0);
  // base_forwards_increment();
   // passive_wait(1.0);
   // base_forwards_increment();
    // passive_wait(1.0);
    // base_forwards_increment();
   // passive_wait(1.0);
    // base_reset();   
    // passive_wait(1.0);
    // } else {
      // passive_wait(1.0);
  // base_forwards_increment();
   // passive_wait(1.0);
    // base_reset();   
    // passive_wait(1.0);
    
    // }
    
    
    drop();
    wb_motor_set_position(arm_elements[ARM3], -2.72);
    passive_wait(2.0);
    
    }
     passive_wait(4.0);
    // grab();
    // passive_wait(1.0);
      // wb_motor_set_position(arm_elements[ARM3], -2.004);
  // passive_wait(1.0);
  
  // disable_physics(stair2);
  // passive_wait(1.0);
  // set_position(stair2, -1.81, 7.94, -0.801);
  // set_rotation(stair2, 0.0547, 0.998, -0.0442, -3.02);
  // passive_wait(1.0);
  // base_forwards_increment();
   // passive_wait(1.0);
   // base_forwards_increment();
    // passive_wait(1.0);
    // base_reset();   
    // passive_wait(1.0);
    
  
  display_helper_message();
  
  int pc = 0;
  wb_keyboard_enable(TIME_STEP);

  while (true) {
    step();
    
    
    int c = wb_keyboard_get_key();
    //fprintf(stderr, "hello\n");
    if ((c >= 0) && c != pc) {
      printf("%d\n", c);
      switch (c) {
        case WB_KEYBOARD_UP:
          base_forwards_increment();
          break;
        case WB_KEYBOARD_DOWN:
          base_backwards_increment();
          break;
        case WB_KEYBOARD_LEFT:
          base_strafe_left_increment();
          break;
        case WB_KEYBOARD_RIGHT:
          base_strafe_right_increment();
          break;
        case WB_KEYBOARD_PAGEUP:
          base_turn_left_increment();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          base_turn_right_increment();
          break;
        case 65:
          grab();
          break;
        case 68:
          drop();
          break;
        case 81:
          rotational_actuator1();
          break;
        case 69:
          rotational_actuator2();
          break;
        case 87:
          nail_gunF();
          break;
        case 83:
          nail_gunB();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
  