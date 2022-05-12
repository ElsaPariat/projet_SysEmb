#ifndef CTRL_DIRECTION_H
#define CTRL_DIRECTION_H
#include <imu.h>

/**
 * @brief 		 sets the speed direction according to the rotation of the arena
 * 							and eventually make a turn to avoid collision with the walls.
 * @param gz     gyroscope value for the Z axis
 */
void ctrl_direction(float gz);

/**
 * @brief 		 activates panik mode if a parasitic rotation along X or Y is detected
 * 							panik mode = body and front leds activate + play A5 note
 * @param gz     gyroscope values for the X and Y axis
 */
void panik_check(float gx, float gy);

//Fonction du TP3 qui allume la led la plus basse
void show_gravity(imu_msg_t *imu_values);

#endif
