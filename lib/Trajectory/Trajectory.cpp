#include <Trajectory.h>

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