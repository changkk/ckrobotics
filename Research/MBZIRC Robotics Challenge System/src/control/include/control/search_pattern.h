#ifndef SEARCH_PATTERN_H
#define SEARCH_PATTERN_H

#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>

class SearchPattern
{
public:
  SearchPattern(std::string filename);
  void startPattern();
  bool moveNext();
  void cancel();
  bool isComplete() const;
  const geometry_msgs::Pose& getWaypoint() const;

private:
  SearchPattern();
  void loadSearchPattern(std::string filename);

  std::string mName;
  std::vector<geometry_msgs::Pose> mWaypointVector;
  std::vector<geometry_msgs::Pose>::const_iterator mCurrentWaypoint;
};

#endif
