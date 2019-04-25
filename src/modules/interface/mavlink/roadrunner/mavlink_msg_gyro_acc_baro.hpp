// MESSAGE GYRO_ACC_BARO support class

#pragma once

namespace mavlink {
namespace roadrunner {
namespace msg {

/**
 * @brief GYRO_ACC_BARO message
 *
 * The Gryo and Accelerometer readings S.I. body frame.
 */
struct GYRO_ACC_BARO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 28;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 107;
    static constexpr auto NAME = "GYRO_ACC_BARO";


    float xacc; /*< [m/s/s] X acceleration */
    float yacc; /*< [m/s/s] Y acceleration */
    float zacc; /*< [m/s/s] Z acceleration */
    float xgyro; /*< [rad/s] Angular speed around X axis */
    float ygyro; /*< [rad/s] Angular speed around Y axis */
    float zgyro; /*< [rad/s] Angular speed around Z axis */
    float temperature; /*< [degC] Temperature */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  xacc: " << xacc << std::endl;
        ss << "  yacc: " << yacc << std::endl;
        ss << "  zacc: " << zacc << std::endl;
        ss << "  xgyro: " << xgyro << std::endl;
        ss << "  ygyro: " << ygyro << std::endl;
        ss << "  zgyro: " << zgyro << std::endl;
        ss << "  temperature: " << temperature << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << xacc;                          // offset: 0
        map << yacc;                          // offset: 4
        map << zacc;                          // offset: 8
        map << xgyro;                         // offset: 12
        map << ygyro;                         // offset: 16
        map << zgyro;                         // offset: 20
        map << temperature;                   // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> xacc;                          // offset: 0
        map >> yacc;                          // offset: 4
        map >> zacc;                          // offset: 8
        map >> xgyro;                         // offset: 12
        map >> ygyro;                         // offset: 16
        map >> zgyro;                         // offset: 20
        map >> temperature;                   // offset: 24
    }
};

} // namespace msg
} // namespace roadrunner
} // namespace mavlink
