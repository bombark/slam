// ============================================================================
//  Header
// ============================================================================

#include <lt_api.h>
#include <vector>

constexpr size_t LIDAR_MAX = 512;

// ============================================================================
//  Robot
// ============================================================================

class Robot {
public:
    Robot() : m_left_vel{0}, m_right_vel{0} {
        m_lidar_values.reserve(LIDAR_MAX);
        m_encoder_values.resize(2);
    }

    void enable_lidar() {
        m_lidar_lnk = lt_new("@new zmq:topic @host 127.0.0.1 @port 5004 @decoder msgpack:obj");
        lt_start_subscriber(&m_lidar_lnk);
        readLidar();
    }

    void enable_motor() {
        m_motor_lnk = lt_new("@new zmq:topic @host 127.0.0.1 @port 5001 @encoder msgpack:obj");
        lt_start_publisher(&m_motor_lnk);
    }

    void enable_compass() {
        m_compass_lnk = lt_new("@new zmq:topic @host 127.0.0.1 @port 5002 @decoder msgpack:obj");
        lt_start_subscriber(&m_compass_lnk);
    }

    void enable_encoder() {
        m_encoder_lnk = lt_new("@new zmq:topic @host 127.0.0.1 @port 5003 @decoder msgpack:obj");
        lt_start_subscriber(&m_encoder_lnk);
    }

    void setVel(int vel) {
        m_left_vel = vel;
        m_right_vel = vel;
        lt_put(&m_motor_lnk, "ii\n", m_left_vel, m_right_vel);
    }

    void setRotVel(float rot_vel) {
        lt_put(&m_motor_lnk, "ii\n", m_left_vel, m_right_vel);
    }

    std::vector<float>& readLidar() {
        lt_recv(&m_lidar_lnk);
        size_t size = lt_copy_af32(&m_lidar_lnk, LIDAR_MAX, &m_lidar_values[0]);
        m_lidar_values.resize(size);
        return m_lidar_values;
    }

    float readCompass() {
        float north;
        lt_get(&m_compass_lnk, "^f", &north);
        return north;
    }

    std::vector<float>& readEncoder() {
        lt_get(&m_encoder_lnk, "^ff", &m_encoder_values[0], &m_encoder_values[1]);
        return m_encoder_values;
    }

private:
    std::vector<float> m_lidar_values;
    std::vector<float> m_encoder_values;
    int m_left_vel, m_right_vel;
    link_t m_lidar_lnk;
    link_t m_motor_lnk;
    link_t m_compass_lnk;
    link_t m_encoder_lnk;
};
