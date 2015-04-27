/*
 * CMPE-242 Embedded Hardware, Lab3: PID Controller
 *
 * The source code of Autonomous Driving Vehicle controller
 *
 * Author: Guangyu Chen <chgy_1@hotmail.com>
 * Mentor: Dr. Harry Li <hua.li@sjsu.edu>
 * Date: Apr 25, 2015
 * Group: Group 1
 * Version: 0.1
 */

#define DEBUG 0

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "adv_pid_242.h"

#define DO_PID				1
#define DO_CONVOLUTION			1

/* Set a test target for PID control */
#define PID_TARGET_TEST			2048
#define PID_DELTA_T			1

#define PWM_IOCTL_STOP			0
#define PWM_IOCTL_SET_FREQ		1

/*
 * Coefficients for PID control:
 * PID = a1 * kp * P + a2 * ki * I + a3 * kd * D
 *
 * Note: a1 + a2 + a3 = 1
 */
#define PID_A1				0.9
#define PID_A2				0.09
#define PID_A3				0.01

/* Make KP = PID_A1, KI = PID_A2, KP = PID_A3 to simplify things */
#define PID_KP				PID_A1
#define PID_KI				PID_A2
#define PID_KD				PID_A3

static int fd_adc, fd_pwm;

/**
 * Function cal_lateral_error() calculates the lateral based on
 * the current speed and angular_error:
 * Lateral Error = sin(angle) * speed;
 *
 * @speed: the speed of the vehicle
 * @angular_error: the current angular error of the vehicle
 */
float cal_lateral_error(float speed, float angular_error)
{
	return sin(M_PI * angular_error / 180) * speed;
}

/**
 * Function cal_1d_kernel() generates a kernel (Gaussian or LoG)
 *
 * @kernel: the pointer to the memory space to store the kernel
 * @size: the desired size of the kernel (keep it an odd number)
 * @sigma: the sigma factor for the calculation
 * @flag: the flag to indicate which kernel, Gaussian or LoG
 */
int cal_1d_kernel(float *kernel, int size, int sigma, int flag)
{
	float sigma2_x_2 = sigma * sigma * 2;
	int i;

	/* Validate the mask size and make sure it is an odd number */
	if (!(size & 1)) {
		printf("%s: Mask size should be an odd number\n", __func__);
		return -EINVAL;
	}

	/* Symmetric kernel only need the half of calculations */
	size /= 2;
	/* Move kernel pointer to the center */
	kernel += size;

	for (i = 0; i <= size; i++) {
		switch (flag) {
		case KERNEL_FLAG_GAUSSIAN:
			/*
			 *          1                          x^2
			 * -------------------- * exp(-1 * -----------)
			 *   ----------------              2 * sigma^2
			 *  /2 * PI * sigma^2
			 */
			kernel[-i] = 1 / sqrt(M_PI * sigma2_x_2) *
				     exp(-1 * i * i / sigma2_x_2);
			break;
		case KERNEL_FLAG_LOG:
			/*
			 *   x^2 - 2 * sigma^2                 x^2
			 * -------------------- * exp(-1 * -----------)
			 *   ------                        2 * sigma^2
			 *  /2 * PI * sigma^5
			 */
			kernel[-i] = (i * i - sigma2_x_2) / pow(sigma, 5) *
				     exp(-1 * i * i / sigma2_x_2) /
				     sqrt(M_PI * sigma2_x_2);
			break;
		default:
			printf("%s: Invalid kernel flag\n", __func__);
			return -EINVAL;
		}

		kernel[i] = kernel[-i];
	}

	return 0;
}

/**
 * Function cal_1d_convolution() computers the convolution of two functions:
 * (f * g)(t) -- function f and function g
 *
 * @f_func: the pointer to the memory space of function f
 * @f_size: the size of the function f
 * @g_func: the pointer to the memory space of function g
 * @g_size: the size of the function g
 * @output: the pointer to the memory space to store the result
 */
int cal_1d_convolution(float *f_func, int f_size, float *g_func, int g_size,
		       float *output)
{
	int g_range = 0;
	int i, j;

	/*
	 * Pattern of convolution:
	 * output[0] = f[0] * g[0]
	 * output[1] = f[0] * g[1] + f[1] * g[0]
	 * output[2] = f[0] * g[2] + f[1] * g[1] + f[2] * g[0]
	 * output[3] = f[0] * g[3] + f[1] * g[2] + f[2] * g[1] + f[3] * g[0]
	 * ......
	 */
	for (i = 0; i < f_size; i++, g_range++) {
		output[i] = 0;
		pr_debug("i=%d, gsize=%d, g_range=%d\n", i, g_size, g_range);
		for (j = 0; j <= g_range; j++) {
			if (g_range - j < g_size) {
				pr_debug("f[%d]=%f, g[%d] = %f\n", j, f_func[j],
					 g_range - j, g_func[g_range - j]);
				output[i] += f_func[j] * g_func[g_range - j];
			}
		}
	}

	return 0;
}

/**
 * Function cal_propotional() computers propotional part of the PID
 *
 * @target: the desired target path of the PID controller
 * @input: the input of the feedback from the closed loop
 * @output: the result of the propotional part of the PID controller
 * @size: the size of input feedback data
 */
int cal_propotional(float target, float *input, float *output,
		    unsigned int size)
{
	int i;

	for (i = 0; i < size; i++)
		output[i] = input[i] - target;

	return 0;
}

/**
 * Function cal_integral() computers integral part of the PID
 *
 * @input: the input of the feedback from the closed loop
 * @output: the result of the integral part of the PID controller
 * @size: the size of input feedback data
 * @upper: the upper boundary of the integral
 * @lower: the lower boundary of the integral
 */
int cal_integral(float *input, float *output, unsigned int size,
		 float upper, float lower)
{
	int i;

	if (upper < lower) {
		printf("%s: upper boundary must >= lower one\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < size; i++) {
		if (!i)
			output[0] = input[0];
		else
			output[i] = output[i - 1] + input[i];

		/* Check and fix the boundary issue */
		if (output[i] > upper)
			output[i] = upper;
		if (output[i] < lower)
			output[i] = lower;
	}

	return 0;
}

/**
 * Function cal_derivative() computers derivative part of the PID
 *
 * @input: the input of the feedback from the closed loop
 * @output: the result of the derivative part of the PID controller
 * @size: the size of input feedback data
 */
int cal_derivative(float *input, float *output, unsigned int size)
{
	int i;

	/* Skip the first and the last one to avoid out of boundries */
	for (i = 1; i < size - 1; i++)
		output[i] = (input[i + 1] - input[i - 1]) / 2;

	return 0;
}

static int set_buzzer_freq(int freq)
{
	int ret = ioctl(fd_pwm, PWM_IOCTL_SET_FREQ, freq);

	if (ret < 0) {
		perror("set the frequency of the buzzer");
		return ret;
	}

	return 0;
}

static void adv_exit(int sig)
{
	ioctl(fd_pwm, PWM_IOCTL_STOP);
	close(fd_adc);
	close(fd_pwm);
	exit(0);
}

static int adc_read_sample(int fd, unsigned int *data)
{
	char tmp[20];
	int len;

	len = read(fd, tmp, sizeof(tmp) - 1);
	if (len > 0) {
		tmp[len] = '\0';
		sprintf(data, "%u", tmp);
	} else {
		perror("read ADC device:");
		return -EINVAL;
	}

	return 0;
}

static void pid_set_params(float *ki, float *kd, float *feedback,
			   unsigned int size)
{
	/* FIXME correct the conditions here */
	*ki = feedback[1] ? PID_KI : 0;
	*kd = feedback[2] ? PID_KD : 0;
}

int main(void)
{
	unsigned int pwm_freq = 1000, sum = 0, tmp;
	float p_tmp[KERNEL_SIZE] = {0};
	float i_tmp[KERNEL_SIZE] = {0};
	float d_tmp[KERNEL_SIZE] = {0};
	float feedback[KERNEL_SIZE];
	float kernel[KERNEL_SIZE];
	float out[KERNEL_SIZE];
	float current, target;
	float ki, kd;
	int i, ret = 0;

	signal(SIGINT, adv_exit);
	printf("Press CTRL+C to exit the program\n");

	fd_adc = open("/dev/adc", 0);
	if (fd_adc < 0) {
		perror("open ADC device:");
		return -ENODEV;
	}

	fd_pwm = open("/dev/pwm", 0);
	if (fd_pwm < 0) {
		perror("open PWM device");
		ret = -ENODEV;
		goto failed_pwm;
	}

	/* Capture one data sample */
	ret = adc_read_sample(fd_adc, &tmp);
	if (ret)
		goto failed;

	current = tmp;

#if DO_CONVOLUTION
	/* Pre-calculate a kernel for convolution computation */
	cal_1d_kernel(kernel, KERNEL_SIZE, 1, KERNEL_FLAG_GAUSSIAN);
#endif

loop:
	printf("Applying PWM frequency: %10d\r", pwm_freq);
	set_buzzer_freq(pwm_freq);

	/* Get 33 samples */
	for (i = 0; i < KERNEL_SIZE; i++) {
		ret = adc_read_sample(fd_adc, &tmp);
		if (ret)
			exit(ret);
		sum += tmp;
	}

	target = sum / KERNEL_SIZE;

	for (i = 0; i < KERNEL_SIZE; i++)
		feedback[i] = current = (current + target) / (i + 1);

#if DO_CONVOLUTION
	cal_1d_convolution(feedback, KERNEL_SIZE, kernel, KERNEL_SIZE, out);
#endif

	/* Check the error to decide if we need integral and derivative parts */
	pid_set_params(&ki, &kd, feedback, KERNEL_SIZE);

#if DO_PID
	cal_propotional(target, out, p_tmp, ARRAY_SIZE(feedback));
	/* FIXME How to decide lower and upper boundaries? */
	if (ki)
		cal_integral(p_tmp, i_tmp, ARRAY_SIZE(feedback), 0, 4096);
	if (kd)
		cal_derivative(p_tmp, d_tmp, ARRAY_SIZE(feedback));

	/* Get average frequency for PID results */
	for (i = 0, pwm_freq = 0; i < KERNEL_SIZE; i++)
		tmp += PID_KP * p_tmp[i] + ki * i_tmp[i] + kd * d_tmp[i];

	tmp /= KERNEL_SIZE;

	/* FIXME Need more moudles before generating pwm frequency */
	pwm_freq = tmp;
#endif

	goto loop;

failed:
	close(fd_pwm);
failed_pwm:
	close(fd_adc);

	return ret;
}
