#include <Trajectory.h>

float hermite_interpolate(const Waypoint &wp1, const Waypoint &wp2, unsigned long elapsed)
{
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0)
        return wp1.position;

    float u = float(elapsed) / dt_ms;
    u = constrain(u, 0.0f, 1.0f);

    float u2 = u * u;
    float u3 = u2 * u;

    float h00 = 2 * u3 - 3 * u2 + 1;
    float h10 = u3 - 2 * u2 + u;
    float h01 = -2 * u3 + 3 * u2;
    float h11 = u3 - u2;

    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    return h00 * wp1.position + h10 * m0 + h01 * wp2.position + h11 * m1;
}

float hermite_velocity(const Waypoint &wp1, const Waypoint &wp2, unsigned long elapsed)
{
    float dt_ms = float(wp2.timestamp - wp1.timestamp);
    if (dt_ms <= 0)
        return 0;

    float u = float(elapsed) / dt_ms;
    if (u <= 0.0f || u >= 1.0f)
        return 0;

    float u2 = u * u;

    float dh00 = 6 * u2 - 6 * u;
    float dh10 = 3 * u2 - 4 * u + 1;
    float dh01 = -6 * u2 + 6 * u;
    float dh11 = 3 * u2 - 2 * u;

    float dt_s = dt_ms * 0.001f;
    float m0 = wp1.velocity * dt_s;
    float m1 = wp2.velocity * dt_s;

    float dPu = dh00 * wp1.position + dh10 * m0 + dh01 * wp2.position + dh11 * m1;

    return dPu / dt_s;
}

ActuatorTrajectory::ActuatorTrajectory(const Waypoint *wps, size_t count)
{
    if (count > MAX_WAYPOINTS)
        count = MAX_WAYPOINTS;
    for (size_t i = 0; i < count; ++i)
    {
        waypoints[i] = wps[i];
    }
    length = count;
}

ActuatorTrajectory::ActuatorTrajectory(const uint8_t *data, size_t len)
{
    if (len < 1)
    {
        length = 0;
        return;
    }

    uint8_t count = data[0];
    if (count > MAX_WAYPOINTS || len < 1 + count * 12)
    {
        length = 0;
        return;
    }

    size_t index = 1;
    for (size_t i = 0; i < count; ++i)
    {
        waypoints[i].position = readFloatLE(&data[index]);
        index += 4;
        waypoints[i].velocity = readFloatLE(&data[index]);
        index += 4;
        waypoints[i].timestamp = readUint32LE(&data[index]);
        index += 4;
    }
    length = count;
}

size_t ActuatorTrajectory::serialize(uint8_t *outBuffer, size_t maxLen)
{
    size_t totalBytes = 1 + length * 12;
    if (maxLen < totalBytes)
        return 0;

    outBuffer[0] = static_cast<uint8_t>(length);
    size_t index = 1;

    for (size_t i = 0; i < length; ++i)
    {
        writeFloatLE(&outBuffer[index], waypoints[i].position);
        index += 4;
        writeFloatLE(&outBuffer[index], waypoints[i].velocity);
        index += 4;
        writeUint32LE(&outBuffer[index], waypoints[i].timestamp);
        index += 4;
    }

    return totalBytes;
}