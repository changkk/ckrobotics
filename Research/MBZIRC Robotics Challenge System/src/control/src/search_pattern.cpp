#include <control/search_pattern.h>

#include <fstream>
#include <sstream>

SearchPattern::SearchPattern(std::string filename) {
  this->loadSearchPattern(filename);
  this->startPattern();
}

void SearchPattern::startPattern() {
  this->mCurrentWaypoint = this->mWaypointVector.begin();
}

bool SearchPattern::moveNext() {
  if (!this->isComplete()) {
    ++(this->mCurrentWaypoint);
    return true;
  }
  return false;
}

void SearchPattern::cancel() {
  this->mCurrentWaypoint = this->mWaypointVector.end();
}

bool SearchPattern::isComplete() const {
  return this->mCurrentWaypoint == this->mWaypointVector.end();
}

const geometry_msgs::Pose& SearchPattern::getWaypoint() const {
  return *(this->mCurrentWaypoint);
}

void SearchPattern::loadSearchPattern(std::string filename) {
  this->mWaypointVector.clear();
  std::ifstream file(filename);
  std::string line;
  if (file.is_open()) {
    while (std::getline(file, line)) {
      geometry_msgs::Pose waypoint;
      std::stringstream ss(line);
      ss >> waypoint.position.x >> waypoint.position.y >> waypoint.position.z;
      this->mWaypointVector.push_back(waypoint);
    }
    file.close();
  }
}
