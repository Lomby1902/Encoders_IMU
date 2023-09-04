#include <MovingAveragePlus.h>


MovingAveragePlus<double> test_wheel1(10);
MovingAveragePlus<double> test_wheel2(10);
MovingAveragePlus<double> test_wheel3(10);
MovingAveragePlus<double> test_wheel4(10);

float speed_measurement[4]={0,0,0,0};

HardwareTimer timer(TIM4);
volatile int counter1=0;
volatile int counter2=0;
volatile int counter3=0;
volatile int counter4=0;

int sample_time_ms = 50;
double sample_time_sec = (double)sample_time_ms/1000;
float Interrupt_FREQ=1/sample_time_sec;
int timerOverflow = 65535;    //16 bit registry

const int fori = 20;
double diameter = 79.0/1000.0; 
float deltat=0;
float angolo_prec=0;
float deltaangolo=0;


#include "motion_fx.h"

#include "LSM6DSLSensor.h"

#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f

#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)


#define DECIMATION                      1U

#define G_TO_ACCEL                      9.81
#define DEG_TO_RAD                      0.01745

#if !(__CORTEX_M == 0U)
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mfxstate[STATE_SIZE];
#endif

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

char LibVersion[35];
int LibVersionLen;

static volatile uint32_t TimeStamp = 0;

int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
int32_t MagOffset[3];


LSM6DSLSensor AccGyr(&Wire);

HardwareTimer *MyTim;

volatile uint8_t fusion_flag;


MFX_input_t data_in; 
MFX_output_t data_out;
float delta_time = MOTION_FX_ENGINE_DELTATIME;
