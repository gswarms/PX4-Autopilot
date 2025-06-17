#pragma once

#ifndef INTERCEPTION_DATA_HPP
#define INTERCEPTION_DATA_HPP

#include <uORB/topics/interception_data.h>  // your custom uORB topic
#include <px4_platform_common/defines.h>

class MavlinkStreamInterceptionData : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamInterceptionData(mavlink);
    }

    const char *get_name() const override
    {
        return MavlinkStreamInterceptionData::get_name_static();
    }

    static const char *get_name_static()
    {
        return "INTERCEPTION_DATA";
    }

    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_INTERCEPTION_DATA;
    }

    uint16_t get_id() override
    {
        return get_id_static();
    }

    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_INTERCEPTION_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;;
    }

private:
    uORB::Subscription _sub{ORB_ID(interception_data)};

    MavlinkStreamInterceptionData(MavlinkStreamInterceptionData &) = delete;
    MavlinkStreamInterceptionData &operator=(const MavlinkStreamInterceptionData &) = delete;

protected:
    explicit MavlinkStreamInterceptionData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    bool send() override
    {
        interception_data_s data;

        if (_sub.update(&data)) {
            mavlink_interception_data_t msg{};

            msg.timestamp = data.timestamp;
            msg.id = data.id;
            msg.tgo = data.tgo;
            msg.miss_distance = data.miss_distance;
            msg.position_std_norm = data.position_std_norm;
            msg.substate = data.substate;
            msg.target_detected = data.target_detected;
            msg.estimated_relative_position[0] = data.estimated_relative_position[0];
            msg.estimated_relative_position[1] = data.estimated_relative_position[1];
            msg.estimated_relative_position[2] = data.estimated_relative_position[2];

            mavlink_msg_interception_data_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }

        return false;
    }
};
#endif // INTERCEPTION_DATA_HPP