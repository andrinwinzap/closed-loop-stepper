# include "Trajectory.h"

Trajectory::Trajectory(const Waypoint* wps, size_t count) {
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    for (size_t i = 0; i < count; ++i) {
        waypoints[i] = wps[i];
    }
    length = count;
}

Trajectory::Trajectory(const uint8_t* data, size_t len) {
    if (len < 1) {
        length = 0;
        return;
    }

    uint8_t count = data[0];
    if (count > MAX_WAYPOINTS || len < 1 + count * 12) {
        length = 0;
        return;
    }

    size_t index = 1;
    for (size_t i = 0; i < count; ++i) {
        waypoints[i].position  = readFloatLE(&data[index]); index += 4;
        waypoints[i].velocity  = readFloatLE(&data[index]); index += 4;
        waypoints[i].timestamp = readUint32LE(&data[index]); index += 4;
    }
    length = count;
}

float Trajectory::readFloatLE(const uint8_t* buf) {
    float val;
    memcpy(&val, buf, 4);
    return val;
}

void Trajectory::writeFloatLE(uint8_t* buf, float value) {
    memcpy(buf, &value, 4);
}

uint32_t Trajectory::readUint32LE(const uint8_t* buf) {
    uint32_t val;
    memcpy(&val, buf, 4);
    return val;
}

void Trajectory::writeUint32LE(uint8_t* buf, uint32_t value) {
    memcpy(buf, &value, 4);
}

size_t Trajectory::serialize(uint8_t* outBuffer, size_t maxLen) {
    size_t totalBytes = 1 + length * 12;
    if (maxLen < totalBytes) return 0;

    outBuffer[0] = static_cast<uint8_t>(length);
    size_t index = 1;

    for (size_t i = 0; i < length; ++i) {
        writeFloatLE(&outBuffer[index], waypoints[i].position); index += 4;
        writeFloatLE(&outBuffer[index], waypoints[i].velocity); index += 4;
        writeUint32LE(&outBuffer[index], waypoints[i].timestamp); index += 4;
    }

    return totalBytes;
}