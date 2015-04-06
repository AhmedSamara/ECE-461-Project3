#ifndef MMA8451_H
#define MMA8451_H

#define MMA_ADDR 0x3A

#define REG_XHI 0x01
#define REG_XLO 0x02
#define REG_YHI 0x03
#define REG_YLO 0x04
#define REG_ZHI	0x05
#define REG_ZLO 0x06

#define REG_WHOAMI 0x0D
#define REG_CTRL1  0x2A
#define REG_CTRL4  0x2D

#define WHOAMI 0x1A

#define COUNTS_PER_G (16384.0f)
#define G_SHIFT (14)

#define M_PI (0.78539816f)
#define QTR_PI  (0.785398163f)
#define THRQTR_PI (2.35619449f)

#define RAD_TO_DEGREES  (57.29582f)

#define X_ACC_2  (100)
#define Y_ACC_2  (100)
#define Z_ACC_2  (100)

#define THRESH (100)

int init_mma(void);
void read_fake_xyz(void);
void read_full_xyz(void);
void read_xyz(void);
void convert_xyz_to_roll_pitch(void);

float sqrt_approx(float);

float arctan_approx(float);
float atan2_approximation1(float, float); 
void update_previous_acc(void);

extern float roll, pitch;

#endif
