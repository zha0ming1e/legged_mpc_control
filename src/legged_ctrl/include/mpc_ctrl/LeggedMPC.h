#pragma once
#include "LeggedState.h"


namespace legged
{
class LeggedMPC {
public:
    LeggedMPC();

    LeggedMPC(ros::NodeHandle &_nh);

    virtual bool update(LeggedState &state, double t, double dt) = 0;
};

}  // namespace legged