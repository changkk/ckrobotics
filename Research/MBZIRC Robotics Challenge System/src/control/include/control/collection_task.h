#ifndef COLLECTION_TASK_H
#define COLLECTION_TASK_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class CollectionTask
{
public:
  enum Action {
    NONE,
    WAYPOINT,
    ACQUIRE,
    DROP
  };

  CollectionTask(Action action, const geometry_msgs::Pose& waypoint);
  CollectionTask::Action getAction() const;
  const geometry_msgs::Pose& getWaypoint() const;

private:
  CollectionTask();

  Action mAction;
  geometry_msgs::Pose mWaypoint;
};

#endif
