#include "mpu6050_i2c.h"
#include "ti_msp_dl_config.h"

#define I2C_WAIT_MAX    (200000U)

/* 中断里不做总线恢复，避免 ISR 里长时间复位 I2C */
static void mpu_i2c_recover(void)
{
    if (__get_IPSR() == 0U) {
        mpu6050_i2c_sda_unlock();
    }
}

static bool i2c_wait_idle(void)
{
    uint32_t timeout = I2C_WAIT_MAX;

    while (!(DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        if (--timeout == 0U) {
            return false;
        }
    }
    return true;
}

static bool i2c_wait_tx_done(void)
{
    uint32_t timeout = I2C_WAIT_MAX;

    while (!DL_I2C_getRawInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        if (--timeout == 0U) {
            return false;
        }
    }
    return true;
}

static void mpu_i2c_disable(void)
{
    DL_I2C_reset(I2C_MPU6050_INST);

    DL_GPIO_initDigitalOutput(GPIO_I2C_MPU6050_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_MPU6050_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
}

static void mpu_i2c_enable(void)
{
    DL_I2C_reset(I2C_MPU6050_INST);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_MPU6050_IOMUX_SDA,
        GPIO_I2C_MPU6050_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_MPU6050_IOMUX_SCL,
        GPIO_I2C_MPU6050_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_enableHiZ(GPIO_I2C_MPU6050_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_MPU6050_IOMUX_SCL);

    DL_I2C_enablePower(I2C_MPU6050_INST);
    SYSCFG_DL_I2C_MPU6050_init();
}

void mpu6050_i2c_sda_unlock(void)
{
    uint32_t i;

    mpu_i2c_disable();

    for (i = 0; i < 100U; i++) {
        DL_GPIO_clearPins(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
        __NOP(); __NOP(); __NOP(); __NOP();
        DL_GPIO_setPins(GPIO_I2C_MPU6050_SCL_PORT, GPIO_I2C_MPU6050_SCL_PIN);
        __NOP(); __NOP(); __NOP(); __NOP();

        if (DL_GPIO_readPins(GPIO_I2C_MPU6050_SDA_PORT, GPIO_I2C_MPU6050_SDA_PIN)) {
            break;
        }
    }

    mpu_i2c_enable();
}

bool mpu_i2c_write_byte(uint8_t devAddr, uint8_t reg, uint8_t val)
{
    uint8_t data = val;
    uint32_t cnt = 1U;
    uint32_t timeout = I2C_WAIT_MAX;

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    DL_I2C_transmitControllerData(I2C_MPU6050_INST, reg);

    if (!i2c_wait_idle()) {
        mpu_i2c_recover();
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    while (cnt > 0U) {
        if (!DL_I2C_isControllerTXFIFOFull(I2C_MPU6050_INST)) {
            DL_I2C_transmitControllerData(I2C_MPU6050_INST, data);
            cnt--;
        }

        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            mpu_i2c_recover();
            return false;
        }

        if (--timeout == 0U) {
            mpu_i2c_recover();
            return false;
        }
    }

    if (!i2c_wait_tx_done()) {
        mpu_i2c_recover();
        return false;
    }

    return true;
}

bool mpu_i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t i = 0U;
    uint32_t timeout = I2C_WAIT_MAX;

    if ((buf == 0) || (len == 0U)) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);
    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    DL_I2C_transmitControllerData(I2C_MPU6050_INST, reg);

    /* 关键：按视频思路，寄存器地址发完后自动 repeated-start 进入读 */
    I2C_MPU6050_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;

    if (!i2c_wait_idle()) {
        I2C_MPU6050_INST->MASTER.MCTR = 0;
        mpu_i2c_recover();
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    while (!DL_I2C_getRawInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE)) {
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
            if (i < len) {
                buf[i++] = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
            }
        }

        if (DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            I2C_MPU6050_INST->MASTER.MCTR = 0;
            mpu_i2c_recover();
            return false;
        }

        if (--timeout == 0U) {
            I2C_MPU6050_INST->MASTER.MCTR = 0;
            mpu_i2c_recover();
            return false;
        }
    }

    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST)) {
        if (i < len) {
            buf[i++] = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
        }
    }

    I2C_MPU6050_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_MPU6050_INST);

    return (i == len);
}

bool mpu_i2c_read_byte(uint8_t devAddr, uint8_t reg, uint8_t *val)
{
    return mpu_i2c_read_bytes(devAddr, reg, val, 1U);
}
