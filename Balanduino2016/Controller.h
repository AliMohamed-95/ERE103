#ifndef _controller_h_
#define _controller_h_

#define _usb_h_ // Workaround include trap in the USB Host library

// This struct will store all parameters for a continuous PID controller
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double Tf;
} PID_cont_t;

// This struct will store all parameters for a discrete PID controller
typedef struct {
    double c0;
    double c1;
    double c2;
    double c3;
    
    double D;
    double I;
    double old_error;
 
} PID_discrete_t;

typedef struct {
    double low = low;
    double high = high;
    unsigned long period_us = period_us; 
    unsigned long next_update_us;
    int state;
} pulse_t;

static pulse_t setpoint_generator_pulse_attributes;

static const double EncoderRes = 928.0*2.0;
static const double wheel_radius = 0.049;
//static const long h = 4000; // Sampling interval: 4000 microseconds 

static double pitch_dot;		// Result from Kalman filter
//static double timer, dt;
static double timer_us, h;
static double lastWheelPosition1;

static inline unsigned long current_time();
static inline void wait_until(unsigned long untilTime);
static inline double getTheta();
static inline double getSpeed(double dt);
static inline double getRotSpeed(double dt);
static inline double getPosition();
static inline void updateEncoders();
static inline void actuateControlSignal(double u);
static inline void checkMotors();

static void setup_setpoint_generator_pulse(double low, double high, unsigned long period_us);
static inline double setpoint_generator_pulse();

#endif
