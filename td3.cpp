#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
  double* arr = new double[new_size];
  if (length > new_size){
      for(int i=0; i<new_size;i++){
          arr[i] = array[i];
      }
  }else{
      for(int i=0; i<length; i++){
          arr[i] = array[i];
      }
      for(int i=length; i<new_size; i++){
          arr[i] = 0;
      }
  }
  delete array;
  return arr; // YOU CAN CHANGE THIS
}

double* shrink_array(double* array, int length, int new_size) {
  double* arr = new double[new_size];
  for(int i=0;i<new_size;i++){
      arr[i] = array[i];
  }
  delete array;

  return arr; // YOU CAN CHANGE THIS
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
  if(current_size==max_size){
      max_size += 5;
      array = extend_array(array, current_size, max_size);
  }
  array[current_size++] = element;

  return array; // YOU CAN CHANGE THIS
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
  current_size--;
  if(max_size-current_size>4){
      max_size -= 5;
      array = shrink_array(array, current_size, max_size);
  }

  return array; // YOU CAN CHANGE THIS
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
      telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
      telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
    }
  }

  return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
  int done = 0;
  int* current_indexs = new int[tot_telemetries];
  for(int i=0; i<tot_telemetries; i++){
      current_indexs[i] = 0;
  }
  while(done < tot_telemetries){
      int t_index;
      double t_t=-1;
      double t_x;
      double t_y;
      for(int chk_index=0;chk_index<tot_telemetries;chk_index++){
          if(current_indexs[chk_index] == telemetries_sizes[chk_index]){
              continue;
          }
          double t = telemetries[chk_index][current_indexs[chk_index]];
          if(t<t_t||t_t<0){
              t_index = chk_index;
              t_t = t;
              t_x = telemetries[chk_index][current_indexs[chk_index]+1];
              t_y = telemetries[chk_index][current_indexs[chk_index]+2];
          }
      }
      if(t_t==-1){
          break;
      }
      global_telemetry = append_to_array(t_t, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      global_telemetry = append_to_array(t_x, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      global_telemetry = append_to_array(t_y, global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
      current_indexs[t_index] += 3;
  }
}
