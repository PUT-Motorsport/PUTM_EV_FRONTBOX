#include "imu.hpp"

#include <algorithm>
#include "ISM330DHCX/ism330dhcx_reg.h"
#include "debugIO.hpp"
#include "imu_sensor_compatibility_layer.h"

namespace IMU {
namespace {
    stmdev_ctx_t sensor_instance;
    std::array<IMUData_t, 3> acc;
    std::array<IMUData_t, 3> gyro;
}

    bool initialize()
    {

        sensor_instance.write_reg = &spi_write;
        sensor_instance.read_reg = &spi_read;
        sensor_instance.mdelay = &HAL_Delay;
        sensor_instance.handle = nullptr; // handle is optional and won't be used

        HAL_Delay(3000); // in case of a loss of power, the IMU needs ~3 s to be able to connect properly

        uint8_t devID;

        uint32_t result{};

        result += ism330dhcx_device_id_get(&sensor_instance, &devID);

        if (devID not_eq ISM330DHCX_ID)
        {
            Device::setState(State::IOSPIError);
            return false;
        }

        // Restore the default configuration
        if (ism330dhcx_reset_set(&sensor_instance, PROPERTY_ENABLE))
        {
            if (not reinitialize(100))
            {
                unrecoverableError(State::IOSPIError);
            }
        }
        // Start device configuration
        result += ism330dhcx_device_conf_set(&sensor_instance, PROPERTY_ENABLE);
        // Enable Block Data Update
        result += ism330dhcx_block_data_update_set(&sensor_instance, PROPERTY_ENABLE);
        // Set Output Data Rate
        result += ism330dhcx_xl_data_rate_set(&sensor_instance, ISM330DHCX_XL_ODR_12Hz5);
        result += ism330dhcx_gy_data_rate_set(&sensor_instance, ISM330DHCX_GY_ODR_12Hz5);
        // Set full scale
        result += ism330dhcx_xl_full_scale_set(&sensor_instance, ISM330DHCX_2g);
        result += ism330dhcx_gy_full_scale_set(&sensor_instance, ISM330DHCX_1000dps);

        result += ism330dhcx_xl_hp_path_on_out_set(&sensor_instance, ISM330DHCX_LP_ODR_DIV_100);
        result += ism330dhcx_xl_filter_lp2_set(&sensor_instance, PROPERTY_ENABLE);

        if (result) {
            Device::setState(State::IOSPIError);
            return false;
            reinitialize(3000);
        }
        return true;
    }

    bool reinitialize(uint32_t timeout)
    {
        uint32_t timeStarted{HAL_GetTick()};
        uint8_t reinitSuccessful = false;
        do
        {
            ism330dhcx_reset_get(&sensor_instance, &reinitSuccessful);
        } while (timeStarted + timeout > HAL_GetTick() and
                 reinitSuccessful == false);

        return reinitSuccessful;
    }

    void updateSensorData()
    {
        uint32_t result{};
        uint8_t reg;
        result += ism330dhcx_xl_flag_data_ready_get(&sensor_instance, &reg);

        if (reg)
        {
            result += ism330dhcx_acceleration_raw_get(&sensor_instance, acc.data());
        }

        ism330dhcx_gy_flag_data_ready_get(&sensor_instance, &reg);

        if (reg)
        {
            result += ism330dhcx_angular_rate_raw_get(&sensor_instance, gyro.data());
        }
        if (result) {
            Device::setState(State::IOSPIError);
        }
    }

    // returns the newest available data
    std::array<IMU::IMUData_t, 3> get_acc_data()
    {
        // todo: add appropriate filtering
        return acc;
    }

    std::array<IMU::IMUData_t, 3> get_gyro_data()
    {
        // todo: add appropriate filtering
        return gyro;
    }

} // namespace IMU
