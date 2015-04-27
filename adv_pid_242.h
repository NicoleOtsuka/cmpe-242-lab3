/*
 * CMPE-242 Embedded Hardware, Lab3: PID Controller
 *
 * The header file of Autonomous Driving Vehicle controller
 *
 * Author: Guangyu Chen <chgy_1@hotmail.com>
 * Mentor: Dr. Harry Li <hua.li@sjsu.edu>
 * Date: Apr 25, 2015
 * Group: Group 1
 * Version: 0.1
 */

#ifndef _ADV_PID_242_
#define _ADV_PID_242_

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define pr_debug(fmt, ...) \
	do { \
		if (DEBUG) \
			printf(fmt, ##__VA_ARGS__); \
	} while (0)

#define KERNEL_FLAG_GAUSSIAN	0
#define KERNEL_FLAG_LOG		1
#define KERNEL_SIZE		33

#define ADV_SIGNAL_LENGTH	4096
#define ADV_VEHICLE_LENGTH	12000
#define ADV_VEHICLE_WIDTH	500

float cal_lateral_error(float, float);
int cal_1d_kernel(float *, int, int, int);
int cal_1d_convolution(float *, int, float *, int, float *);
int cal_propotional(float, float *, float *, unsigned int);
int cal_integral(float *, float *, unsigned int, float, float);
int cal_derivative(float *, float *, unsigned int);

#endif /* ADV_PID_242 */
