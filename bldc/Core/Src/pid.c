#include "pid.h"
#include <stdint.h>
int pid_controller(float target_vel, float actual_vel, float Kp, float Ti, float Td)
{
	float ek0;
	static float u1, ek1, ek2;
  	float q0 = Kp * (1 + T/Ti + Td/T);
  	float q1 = -Kp * (1 + 2 * Td/T);
  	float q2 = Kp * Td/T;
	ek0 = target_vel - actual_vel;
	u1 = u1 + q0 * ek0 + q1 * ek1 + q2 * ek2;
	if (u1 > 8399)
 	{
		u1 = 8399;
	}
 	if (u1 < 0)
	{
		u1 = 0;
	}
	ek2 = ek1;
	ek1 = ek0;
 	return (int)u1;
}