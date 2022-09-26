/*
 *  Important shared variables between the robot interface and the cotroller 
 */

#include <Eigen/Dense>
#include <ros/ros.h>

#include "LeggedParams.h"
#include "LeggedState.h"

namespace legged
{
void LeggedFeedback::reset() {

}

void LeggedCtrl::reset() {
}


}  // namespace legged