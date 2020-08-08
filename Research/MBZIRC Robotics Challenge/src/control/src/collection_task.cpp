#include <control/collection_task.h>

CollectionTask::CollectionTask(Action action, const geometry_msgs::Pose& waypoint) {
  this->mAction = action;
  this->mWaypoint = waypoint;
}

CollectionTask::Action CollectionTask::getAction() const {
  return this->mAction;
}

const geometry_msgs::Pose& CollectionTask::getWaypoint() const {
  return this->mWaypoint;
}
