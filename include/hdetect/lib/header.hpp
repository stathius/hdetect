#ifndef HEADER_HPP
#define HEADER_HPP

#include <string>

namespace Header
{
    enum
    {
        NOT_HUMAN = -1,
        OTHER,
        FUSION_HUMAN,
        LASER_HUMAN,
        LABEL_HUMAN
    };

    const std::string imageTopic = "/recognizeRT/imageTopic";

    extern float curTimestamp;
    extern float preTimestamp;
}

#endif // HEADER_HPP
