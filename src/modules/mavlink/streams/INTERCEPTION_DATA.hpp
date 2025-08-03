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

            PX4_DEBUG("Sending INTERCEPTION_DATA: id=%d tgo=%.2f md=%.2f pos=[%.2f %.2f %.2f]",
                data.id,
                static_cast<double>(data.tgo),
                static_cast<double>(data.miss_distance),
                static_cast<double>(data.estimated_relative_position[0]),
                static_cast<double>(data.estimated_relative_position[1]),
                static_cast<double>(data.estimated_relative_position[2]));

            msg.timestamp = data.timestamp;
            msg.id = data.id;
            msg.tgo = data.tgo;
            msg.miss_distance = data.miss_distance;
            msg.position_std_norm = data.position_std_norm;
            msg.substate = data.substate;
            msg.target_detected = data.target_detected;
            msg.target_time_delay = data.target_time_delay;
            msg.estimated_relative_position[0] = data.estimated_relative_position[0];
            msg.estimated_relative_position[1] = data.estimated_relative_position[1];
            msg.estimated_relative_position[2] = data.estimated_relative_position[2];
            msg.interceptor_active = data.interceptor_active;
            msg.target_estimation_active = data.target_estimation_active;
            msg.camera_driver_active = data.camera_driver_active;
            msg.monitor_active = data.monitor_active;
            msg.recorder_active = data.recorder_active;
            msg.detector_active = data.detector_active;
            msg.osd_active = data.osd_active;

            mavlink_msg_interception_data_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }

        return false;
    }
};
#endif // INTERCEPTION_DATA_HPP