/*
 * Copyright (C) 2018 Anatoliy Atanasov, Iliyan Stoyanov
 *
 * This file is port of https://github.com/sparkfun/Battery_Babysitter code
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_bq27441
 * @{
 *
 * @file
 * @brief       drivers_bq27441 fuel gauge device driver
 *
 * @author      Anatoliy Atanasov <anatoliy@6lowpan.io>
 * @author      Iliyan Stoyanov <iliyan@6lowpan.io>
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "debug.h"
#include "bq27441.h"
#include "bq27441-internal.h"

#define BQ72441_I2C_TIMEOUT (2000U)
#define BQ72441_I2C_BUFSIZE (128U)

#ifndef I2C_ACK
#define I2C_ACK             (0)
#endif

#define ENABLE_DEBUG        (0)

uint8_t _constrain(uint8_t amt, uint8_t low, uint8_t high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

static void debug_print_i2c_read(i2c_t dev, uint8_t reg, uint8_t *buf, int len)
{
    DEBUG_PRINT("Success: i2c_%i read %i byte(s) from reg 0x%02x ", dev, len, reg);
    DEBUG_PRINT(": [");
    for (int i = 0; i < len; i++) {
        if (i != 0) {
            DEBUG_PRINT(", ");
        }
        DEBUG_PRINT("0x%02x", buf[i]);
    }
    DEBUG_PRINT("]\n");
}

static int debug_print_i2c_error(int res)
{
    if (res == -EOPNOTSUPP) {
        DEBUG_PRINT("Error: EOPNOTSUPP [%d]\n", -res);
        return 1;
    }
    else if (res == -EINVAL) {
        DEBUG_PRINT("Error: EINVAL [%d]\n", -res);
        return 1;
    }
    else if (res == -EAGAIN) {
        DEBUG_PRINT("Error: EAGAIN [%d]\n", -res);
        return 1;
    }
    else if (res == -ENXIO) {
        DEBUG_PRINT("Error: ENXIO [%d]\n", -res);
        return 1;
    }
    else if (res == -EIO) {
        DEBUG_PRINT("Error: EIO [%d]\n", -res);
        return 1;
    }
    else if (res == -ETIMEDOUT) {
        DEBUG_PRINT("Error: ETIMEDOUT [%d]\n", -res);
        return 1;
    }
    else if (res == I2C_ACK) {
        DEBUG_PRINT("Success: I2C_ACK [%d]\n", res);
        return 0;
    }
    DEBUG_PRINT("Error: Unknown error [%d]\n", res);
    return 1;
}

bool bq27441_init(bq27441_t *dev, const bq27441_param_t *param)
{
    assert(dev && param);
    dev->param = *param;
    dev->bus = param->bus;
    dev->addr = param->addr;
    dev->sealFlag =  false;
    dev->userConfigControl = false;
    i2c_init(dev->bus);
    gpio_init_int(dev->param.alarm_pin, GPIO_IN, GPIO_FALLING, dev->cb, dev->arg);
    return true;
}

bool bq27441_set_capacity(bq27441_t *dev, uint16_t capacity)
{
    /* Write to STATE subclass (82) of BQ27441 extended memory.
       Offset 0x0A (10)
       Design capacity is a 2-byte piece of data - MSB first */
    uint8_t capacityData[2] = { capacity >> 8, capacity & 0x00FF };

    return _bq27441_write_extended_data(dev, BQ27441_ID_STATE, 10, capacityData, 2);
}

uint16_t bq27441_get_voltage(const bq27441_t *dev)
{
    return _bq27441_read_word(dev, BQ27441_COMMAND_VOLTAGE);
}

int16_t bq27441_get_current(const bq27441_t *dev, current_measure_t type)
{
    int16_t current = 0;

    switch (type) {
        case AVG:
            current = (int16_t)_bq27441_read_word(dev, BQ27441_COMMAND_AVG_CURRENT);
            break;
        case STBY:
            current = (int16_t)_bq27441_read_word(dev, BQ27441_COMMAND_STDBY_CURRENT);
            break;
        case MAX:
            current = (int16_t)_bq27441_read_word(dev, BQ27441_COMMAND_MAX_CURRENT);
            break;
    }

    return current;
}

uint16_t bq27441_get_capacity(const bq27441_t *dev, capacity_measure_t type)
{
    uint16_t capacity = 0;

    switch (type) {
        case REMAIN:
            return _bq27441_read_word(dev, BQ27441_COMMAND_REM_CAPACITY);
            break;
        case FULL:
            return _bq27441_read_word(dev, BQ27441_COMMAND_FULL_CAPACITY);
            break;
        case AVAIL:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_NOM_CAPACITY);
            break;
        case AVAIL_FULL:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_AVAIL_CAPACITY);
            break;
        case REMAIN_F:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_REM_CAP_FIL);
            break;
        case REMAIN_UF:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_REM_CAP_UNFL);
            break;
        case FULL_F:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_FULL_CAP_FIL);
            break;
        case FULL_UF:
            capacity = _bq27441_read_word(dev, BQ27441_COMMAND_FULL_CAP_UNFL);
            break;
        case DESIGN:
            capacity = _bq27441_read_word(dev, BQ27441_EXTENDED_CAPACITY);
    }

    return capacity;
}

int16_t bq27441_get_power(const bq27441_t *dev)
{
    return (int16_t)_bq27441_read_word(dev, BQ27441_COMMAND_AVG_POWER);
}

uint16_t bq27441_get_soc(const bq27441_t *dev, soc_measure_t type)
{
    uint16_t socRet = 0;

    switch (type) {
        case FILTERED:
            socRet = _bq27441_read_word(dev, BQ27441_COMMAND_SOC);
            break;
        case UNFILTERED:
            socRet = _bq27441_read_word(dev, BQ27441_COMMAND_SOC_UNFL);
            break;
    }

    return socRet;
}

uint8_t bq27441_get_soh(const bq27441_t *dev, soh_measure_t type)
{
    uint16_t sohRaw = _bq27441_read_word(dev, BQ27441_COMMAND_SOH);
    uint8_t sohStatus = sohRaw >> 8;
    uint8_t sohPercent = sohRaw & 0x00FF;

    if (type == PERCENT) {
        return sohPercent;
    }
    else {
        return sohStatus;
    }
}

uint16_t bq27441_get_temperature(const bq27441_t *dev, temp_measure_t type)
{
    uint16_t temp = 0;

    switch (type) {
        case BATTERY:
            temp = _bq27441_read_word(dev, BQ27441_COMMAND_TEMP);
            break;
        case INTERNAL_TEMP:
            temp = _bq27441_read_word(dev, BQ27441_COMMAND_INT_TEMP);
            break;
    }
    return temp;
}

bool bq27441_get_gpout_polarity(const bq27441_t *dev)
{
    uint16_t opConfigRegister = _bq27441_get_opConfig(dev);

    return (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
}

bool bq27441_set_gpout_polarity(bq27441_t *dev, bool activeHigh)
{
    uint16_t oldOpConfig = _bq27441_get_opConfig(dev);

    /* Check to see if we need to update opConfig: */
    if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL))) {
        return true;
    }

    uint16_t newOpConfig = oldOpConfig;
    if (activeHigh) {
        newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
    }
    else {
        newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);
    }

    return _bq27441_write_op_config(dev, newOpConfig);
}

bool bq27441_get_gpout_function(const bq27441_t *dev)
{
    uint16_t opConfigRegister = _bq27441_get_opConfig(dev);

    return (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);
}

bool bq27441_set_gpout_function(bq27441_t *dev, gpout_function_t function)
{
    uint16_t oldOpConfig = _bq27441_get_opConfig(dev);

    /* Check to see if we need to update opConfig: */
    if ((function && (oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27441_OPCONFIG_BATLOWEN))) {
        return true;
    }

    /* Modify BATLOWN_EN bit of opConfig: */
    uint16_t newOpConfig = oldOpConfig;
    if (function) {
        newOpConfig |= BQ27441_OPCONFIG_BATLOWEN;
    }
    else {
        newOpConfig &= ~(BQ27441_OPCONFIG_BATLOWEN);
    }

    /* Write new opConfig */
    return _bq27441_write_op_config(dev, newOpConfig);
}

uint8_t bq27441_get_soc1_set_threshold(bq27441_t *dev)
{
    return _bq27441_read_extended_data(dev, BQ27441_ID_DISCHARGE, 0);
}

uint8_t bq27441_get_soc1_clear_threshold(bq27441_t *dev)
{
    return _bq27441_read_extended_data(dev, BQ27441_ID_DISCHARGE, 1);
}

bool bq27441_set_soc1_thresholds(bq27441_t *dev, uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];

    thresholds[0] = _constrain(set, 0, 100);
    thresholds[1] = _constrain(clear, 0, 100);
    return _bq27441_write_extended_data(dev, BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

uint8_t bq27441_get_socf_set_threshold(bq27441_t *dev)
{
    return _bq27441_read_extended_data(dev, BQ27441_ID_DISCHARGE, 2);
}

uint8_t bq27441_get_socf_clear_threshold(bq27441_t *dev)
{
    return _bq27441_read_extended_data(dev, BQ27441_ID_DISCHARGE, 3);
}

bool bq27441_set_socf_thresholds(bq27441_t *dev, uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];

    thresholds[0] = _constrain(set, 0, 100);
    thresholds[1] = _constrain(clear, 0, 100);
    return _bq27441_write_extended_data(dev, BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

bool bq27441_get_soc1_flag(const bq27441_t *dev)
{
    uint16_t flagState = bq27441_get_flags(dev);

    return flagState & BQ27441_FLAG_SOC1;
}

bool bq27441_get_socf_flag(const bq27441_t *dev)
{
    uint16_t flagState = bq27441_get_flags(dev);

    return flagState & BQ27441_FLAG_SOCF;

}

uint8_t bq27441_get_soc_int_delta(bq27441_t *dev)
{
    return _bq27441_read_extended_data(dev, BQ27441_ID_STATE, 26);
}

bool bq27441_set_soc_int_delta(bq27441_t *dev, uint8_t delta)
{
    uint8_t soci = _constrain(delta, 0, 100);

    return _bq27441_write_extended_data(dev, BQ27441_ID_STATE, 26, &soci, 1);
}

bool bq27441_pulse_gpout(const bq27441_t *dev)
{
    return _bq27441_execute_control_word(dev, BQ27441_CONTROL_PULSE_SOC_INT);
}

uint16_t bq27441_get_device_type(const bq27441_t *dev)
{
    return _bq27441_read_control_word(dev, BQ27441_CONTROL_DEVICE_TYPE);
}

bool bq27441_enter_config_mode(bq27441_t *dev, bool userControl)
{
    if (userControl) {
        dev->userConfigControl = true;
    }

    if (_bq27441_sealed(dev)) {
        dev->sealFlag = true;
        _bq27441_unseal(dev); /* Must be unsealed before making changes */
    }

    if (_bq27441_execute_control_word(dev, BQ27441_CONTROL_SET_CFGUPDATE)) {
        int16_t timeout = BQ72441_I2C_TIMEOUT;
        while ((timeout--) && (!(bq27441_get_status(dev) & BQ27441_FLAG_CFGUPMODE))) {
            xtimer_nanosleep(1000);
        }

        if (timeout > 0) {
            return true;
        }
    }

    return false;
}

bool bq27441_exit_config_mode(const bq27441_t *dev, bool resim)
{
    /* There are two methods for exiting config mode:
          1. Execute the EXIT_CFGUPDATE command
          2. Execute the SOFT_RESET command
       EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
       measurement, and without resimulating to update unfiltered-SoC and SoC.
       If a new OCV measurement or resimulation is desired, SOFT_RESET or
       EXIT_RESIM should be used to exit config mode. */
    if (resim) {
        if (_bq27441_soft_reset(dev)) {
            int16_t timeout = BQ72441_I2C_TIMEOUT;
            while ((timeout--) && ((bq27441_get_flags(dev) & BQ27441_FLAG_CFGUPMODE))) {
                xtimer_nanosleep(1000);
            }
            if (timeout > 0) {
                if (dev->sealFlag) {
                    _bq27441_seal(dev); /* Seal back up if we IC was sealed coming in */
                }
                return true;
            }
        }
        return false;
    }
    else {
        return _bq27441_execute_control_word(dev, BQ27441_CONTROL_EXIT_CFGUPDATE);
    }
}

uint16_t bq27441_get_flags(const bq27441_t *dev)
{
    return _bq27441_read_word(dev, BQ27441_COMMAND_FLAGS);
}

uint16_t bq27441_get_status(const bq27441_t *dev)
{
    return _bq27441_read_control_word(dev, BQ27441_CONTROL_STATUS);
}

bool _bq27441_sealed(const bq27441_t *dev)
{
    uint16_t stat = bq27441_get_status(dev);

    return stat & BQ27441_STATUS_SS;
}

bool _bq27441_seal(const bq27441_t *dev)
{
    return _bq27441_read_control_word(dev, BQ27441_CONTROL_SEALED);
}

bool _bq27441_unseal(const bq27441_t *dev)
{
    /* To unseal the BQ27441, write the key to the control
       command. Then immediately write the same key to control again. */
    if (_bq27441_read_control_word(dev, BQ27441_UNSEAL_KEY)) {
        return _bq27441_read_control_word(dev, BQ27441_UNSEAL_KEY);
    }
    return false;
}

uint16_t _bq27441_get_opConfig(const bq27441_t *dev)
{
    return _bq27441_read_word(dev, BQ27441_EXTENDED_OPCONFIG);
}

bool _bq27441_write_op_config(bq27441_t *dev, uint16_t value)
{
    uint8_t opConfigData[2] = { value >> 8, value & 0x00FF };

    /* OpConfig register location: BQ27441_ID_REGISTERS id, offset 0 */
    return _bq27441_write_extended_data(dev, BQ27441_ID_REGISTERS, 0, opConfigData, 2);
}

bool _bq27441_soft_reset(const bq27441_t *dev)
{
    return _bq27441_execute_control_word(dev, BQ27441_CONTROL_SOFT_RESET);
}

uint16_t _bq27441_read_word(const bq27441_t *dev, uint16_t subAddress)
{
    uint8_t data[2];

    _bq27441_read_bytes_i2c(dev, subAddress, data, 2);
    return ((uint16_t)data[1] << 8) | data[0];
}

uint16_t _bq27441_read_control_word(const bq27441_t *dev, uint16_t function)
{
    uint8_t command[2] = { function & 0x00FF, function >> 8 };
    uint8_t data[2] = { 0, 0 };

    _bq27441_write_bytes_i2c(dev, (uint8_t)0, command, 2);

    if (_bq27441_read_bytes_i2c(dev, (uint8_t)0, data, 2)) {
        return ((uint16_t)data[1] << 8) | data[0];
    }

    return false;
}

bool _bq27441_execute_control_word(const bq27441_t *dev, uint16_t function)
{
    uint8_t command[2] = { function & 0x00FF, function >> 8 };

    /* uint8_t data[2] = {0, 0}; */

    if (_bq27441_write_bytes_i2c(dev, (uint8_t)0, command, 2)) {
        return true;
    }

    return false;
}

bool _bq27441_block_data_control(const bq27441_t *dev)
{
    uint8_t enableByte = 0x00;

    return _bq27441_write_bytes_i2c(dev, BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

bool _bq27441_block_data_class(const bq27441_t *dev, uint8_t id)
{
    return _bq27441_write_bytes_i2c(dev, BQ27441_EXTENDED_DATACLASS, &id, 1);
}

bool _bq27441_block_data_offset(const bq27441_t *dev, uint8_t offset)
{
    return _bq27441_write_bytes_i2c(dev, BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

uint8_t _bq27441_block_data_checksum(const bq27441_t *dev)
{
    uint8_t csum;

    _bq27441_read_bytes_i2c(dev, BQ27441_EXTENDED_CHECKSUM, &csum, 1);
    return csum;
}

uint8_t _bq27441_read_block_data(const bq27441_t *dev, uint8_t offset)
{
    uint8_t ret;
    uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;

    _bq27441_read_bytes_i2c(dev, address, &ret, 1);
    return ret;
}

bool _bq27441_write_block_data(const bq27441_t *dev, uint8_t offset, uint8_t data)
{
    uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;

    return _bq27441_write_bytes_i2c(dev, address, &data, 1);
}

uint8_t _bq27441_compute_block_checksum(const bq27441_t *dev)
{
    uint8_t data[32];

    _bq27441_read_bytes_i2c(dev, BQ27441_EXTENDED_BLOCKDATA, data, 32);

    uint8_t csum = 0;
    for (int i = 0; i < 32; i++) {
        csum += data[i];
    }
    csum = 255 - csum;

    return csum;
}

bool _bq27441_write_block_checksum(const bq27441_t *dev, uint8_t csum)
{
    return _bq27441_write_bytes_i2c(dev, BQ27441_EXTENDED_CHECKSUM, &csum, 1);
}

uint8_t _bq27441_read_extended_data(bq27441_t *dev, uint8_t classID, uint8_t
                                    offset)
{
    uint8_t retData = 0;

    if (!dev->userConfigControl) {
        bq27441_enter_config_mode(dev, false);
    }

    if (!_bq27441_block_data_control(dev)) { /* Enable block data memory
                                                control */
        return false; /* Return false if enable fails */
    }
    if (!_bq27441_block_data_class(dev, classID)) { /* Write class ID using
                                                       DataBlockClass() */
        return false;
    }

    _bq27441_block_data_offset(dev, offset / 32); /* Write 32-bit block offset
                                                     (usually 0) */

    _bq27441_compute_block_checksum(dev); /* Compute checksum going in */
    /*
    uint8_t oldCsum = blockDataChecksum();
    for (int i=0; i<32; i++)
        Serial.print(String(readBlockData(i)) + " ");
    */
    retData = _bq27441_read_block_data(dev, offset % 32); /* Read from offset (limit to 0-31) */

    if (!dev->userConfigControl) {
        bq27441_exit_config_mode(dev, true);
    }

    return retData;
}

bool _bq27441_write_extended_data(bq27441_t *dev, uint8_t classID, uint8_t
                                  offset, uint8_t *data, uint8_t len)
{
    if (len > 32) {
        return false;
    }

    if (!dev->userConfigControl) {
        bq27441_enter_config_mode(dev, false);
    }

    if (!_bq27441_block_data_control(dev)) { /* Enable block data memory
                                                control */
        return false; /* Return false if enable fails */
    }
    if (!_bq27441_block_data_class(dev, classID)) { /* Write class ID using
                                                       DataBlockClass() */
        return false;
    }

    _bq27441_block_data_offset(dev, offset / 32);   /* Write 32-bit block
                                                       offset (usually 0) */
    _bq27441_compute_block_checksum(dev); /* Compute checksum going in */
    /*
    uint8_t oldCsum = blockDataChecksum();
    */

    /* Write data bytes: */
    for (int i = 0; i < len; i++) {
        /* Write to offset, mod 32 if offset is greater than 32
           The blockDataOffset above sets the 32-bit block */
        _bq27441_write_block_data(dev, (offset % 32) + i, data[i]);
    }

    /* Write new checksum using BlockDataChecksum (0x60) */
    uint8_t newCsum = _bq27441_compute_block_checksum(dev); /* Compute the new
                                                               checksum */
    _bq27441_write_block_checksum(dev, newCsum);

    if (!dev->userConfigControl) {
        bq27441_exit_config_mode(dev, true);
    }

    return true;
}

int16_t _bq27441_read_bytes_i2c(const bq27441_t *dev, uint8_t subAddress,
                                uint8_t *dest, uint8_t count)
{
    int16_t timeout = BQ72441_I2C_TIMEOUT;
    uint8_t i2c_buf[BQ72441_I2C_BUFSIZE];

    while (timeout--) {
        xtimer_nanosleep(1000);
        int res = i2c_read_regs(dev->bus, BQ72441_I2C_ADDRESS, subAddress, i2c_buf, count, 0);
        if (res == I2C_ACK) {
            debug_print_i2c_read(dev->bus, subAddress, i2c_buf, count);
            for (int i = 0; i < count; i++) {
                dest[i] = i2c_buf[i];
            }
            break;
        }
        return debug_print_i2c_error(res);
    }
    return timeout;
}

uint16_t _bq27441_write_bytes_i2c(const bq27441_t *dev, uint8_t subAddress,
                                  uint8_t *src, uint8_t count)
{
    int res;
    uint8_t i2c_buf[BQ72441_I2C_BUFSIZE];

    for (int i = 0; i < count; i++) {
        i2c_buf[i] = src[i];
    }

    res = i2c_write_regs(dev->bus, BQ72441_I2C_ADDRESS, subAddress, i2c_buf, count, 0);
    if (res == I2C_ACK) {
        DEBUG_PRINT("Success: i2c_%i wrote %i bytes to reg 0x%02x\n",
                    dev->bus, count, subAddress);
        return 0;
    }
    return debug_print_i2c_error(res);
}
