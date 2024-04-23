#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "SFO_V8.h"

//develop from : hrpwm_ex1_duty_sfo from C2000ware 5.02 English->device->F2837xD->F28379D->examples->driverlib->cpu1->hrpwm->ex1
#define EPWM1_TIMER_TBPRD 10UL   // 这将决定ePWM1的频率
#define EPWM2_TIMER_TBPRD 1500UL  // ePWM2的周期设置，满足116高/34低的要求

float32_t dutyFine = 4.0 / ((float32_t)EPWM1_TIMER_TBPRD) * 100.0;
uint16_t status;

int MEP_ScaleFactor;

volatile uint32_t ePWM[] = {
    0, myEPWM1_BASE, myEPWM2_BASE, myEPWM3_BASE, myEPWM4_BASE
};

void initEPWM1(uint32_t period);
void initEPWM2(uint32_t period);
void error(void);

void main(void) {
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    initEPWM1(EPWM1_TIMER_TBPRD);
    initEPWM2(EPWM2_TIMER_TBPRD);

    while(status == SFO_INCOMPLETE) {
        status = SFO();
        if(status == SFO_ERROR) {
            error();
        }
    }

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EINT;  // Enable Global Interrupts
    ERTM;  // Enable Real-time Interrupts

    for(;;) {
        DEVICE_DELAY_US(1000);

        status = SFO(); // Call SFO to handle any calibration needed
        if (status == SFO_ERROR) {
            error(); // Handle errors from SFO
        }
    }
}
//帮我把这两个函数改成通用函数，第一个argument为index，第二个argument为频率，第三个argument 为占空比，第四个argument 为是否 EPWM_SYNC_OUT_PULSE_DISABLED
void initEPWM1(uint32_t period) {
    EPWM_setEmulationMode(ePWM[1], EPWM_EMULATION_FREE_RUN);
           EPWM_setTimeBasePeriod(ePWM[1], period-1);
           EPWM_setPhaseShift(ePWM[1], 0U);
           EPWM_setTimeBaseCounter(ePWM[1], 0U);
           HRPWM_setCounterCompareValue(ePWM[1], HRPWM_COUNTER_COMPARE_A, (period/2 << 8));
           HRPWM_setCounterCompareValue(ePWM[1], HRPWM_COUNTER_COMPARE_B, (period/2 << 8));
           EPWM_setTimeBaseCounterMode(ePWM[1], EPWM_COUNTER_MODE_UP);
           EPWM_setSyncOutPulseMode(ePWM[1], EPWM_SYNC_OUT_PULSE_DISABLED);
           EPWM_setCounterCompareShadowLoadMode(ePWM[1], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
           EPWM_setCounterCompareShadowLoadMode(ePWM[1], EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
           EPWM_setActionQualifierAction(ePWM[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
           EPWM_setActionQualifierAction(ePWM[1], EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
           HRPWM_setMEPEdgeSelect(ePWM[1], HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
           HRPWM_setMEPControlMode(ePWM[1], HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
           HRPWM_setCounterCompareShadowLoadEvent(ePWM[1], HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
           HRPWM_setMEPEdgeSelect(ePWM[1], HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
           HRPWM_setMEPControlMode(ePWM[1], HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
           HRPWM_setCounterCompareShadowLoadEvent(ePWM[1], HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
           HRPWM_enableAutoConversion(ePWM[1]);
}

void initEPWM2(uint32_t period) {
    uint32_t highCycles = 1160;  // 高电平周期数
        uint32_t lowCycles = period - highCycles;  // 低电平周期数, 150 - 116 = 34
    EPWM_setEmulationMode(ePWM[2], EPWM_EMULATION_FREE_RUN);
           EPWM_setTimeBasePeriod(ePWM[2], period-1);
           EPWM_setPhaseShift(ePWM[2], 0U);
           EPWM_setTimeBaseCounter(ePWM[2], 0U);
           HRPWM_setCounterCompareValue(ePWM[2], HRPWM_COUNTER_COMPARE_A,highCycles << 8);
           HRPWM_setCounterCompareValue(ePWM[2], HRPWM_COUNTER_COMPARE_B,highCycles << 8);
           EPWM_setTimeBaseCounterMode(ePWM[2], EPWM_COUNTER_MODE_UP);
           EPWM_setSyncOutPulseMode(ePWM[2], EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);//EPWM_SYNC_OUT_PULSE_DISABLED or 0 EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO 2 4 5 6
           EPWM_setCounterCompareShadowLoadMode(ePWM[2], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
           EPWM_setCounterCompareShadowLoadMode(ePWM[2], EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
           EPWM_setActionQualifierAction(ePWM[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
           EPWM_setActionQualifierAction(ePWM[2], EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
           HRPWM_setMEPEdgeSelect(ePWM[2], HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
           HRPWM_setMEPControlMode(ePWM[2], HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
           HRPWM_setCounterCompareShadowLoadEvent(ePWM[2], HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
           HRPWM_setMEPEdgeSelect(ePWM[2], HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
           HRPWM_setMEPControlMode(ePWM[2], HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
           HRPWM_setCounterCompareShadowLoadEvent(ePWM[2], HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
           HRPWM_enableAutoConversion(ePWM[2]);
}

void error(void) {
    ESTOP0;  // Halt the system and indicate error
}
