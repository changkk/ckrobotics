#include <perception/target.h>

#include <boost/bimap.hpp>
#include <boost/assign.hpp>

static const float CLOSE_ENOUGH = 5.0;
static const float OBSERVATION_CLOSENESS = 1.0;
static const int MIN_OBSERVATIONS = 3;

static const boost::bimap<Target::Color, std::string> COLOR_MAP = boost::assign::list_of<boost::bimap<Target::Color, std::string>::relation>
(Target::Color::RED, "red")
(Target::Color::GREEN, "green")
(Target::Color::BLUE, "blue")
(Target::Color::YELLOW, "yellow")
(Target::Color::ORANGE, "orange");

Target::Target() {
  this->mId = 0;
  this->mColor = Color::NONE;
  this->mIsConfirmed = false;
  this->mObservationTimes.push_back(ros::Time::now());
}

bool Target::operator==(const Target& rhs) const {
  return this->mColor == rhs.getColor() &&
         fabs(std::get<0>(this->mPosition) - std::get<0>(rhs.getPosition())) < CLOSE_ENOUGH &&
         fabs(std::get<1>(this->mPosition) - std::get<1>(rhs.getPosition())) < CLOSE_ENOUGH;
}

void Target::addObservation(const ros::Time& time) {
  this->mObservationTimes.push_back(time);
  if (!this->mIsConfirmed) {
    for (std::vector<ros::Time>::const_iterator iter1 = this->mObservationTimes.begin(); iter1 != this->mObservationTimes.end(); ++iter1) {
      int count = 0;
      for (std::vector<ros::Time>::const_iterator iter2 = this->mObservationTimes.begin(); iter2 != this->mObservationTimes.end(); ++iter2) {
        if (*iter2 - *iter1 < ros::Duration(OBSERVATION_CLOSENESS)) {
          ++count;
        }
      }
      if (count >= MIN_OBSERVATIONS) {
        this->mIsConfirmed = true;
      }
    }
  }
}

bool Target::confirmedTarget() const {
  return this->mIsConfirmed;
}

unsigned int Target::getId() const {
  return this->mId;
}

Target::Color Target::getColor() const {
  return this->mColor;
}

std::tuple<float, float> Target::getPosition() const {
  return this->mPosition;
}

void Target::setId(unsigned int id) {
  this->mId = id;
}

void Target::setColor(Target::Color color) {
  this->mColor = color;
}

void Target::setPosition(std::tuple<float, float> position) {
  this->mPosition = position;
  this->mPositionHistory.push_back(position);
}

perception::TargetMsg Target::serialize() const {
  perception::TargetMsg targetMsg;
  targetMsg.id = this->mId;
  targetMsg.x_position = std::get<0>(this->mPosition);
  targetMsg.y_position = std::get<1>(this->mPosition);
  targetMsg.color = COLOR_MAP.left.find(this->getColor())->second;
  return targetMsg;
}

Target Target::deserialize(const perception::TargetMsg& targetMsg) {
  Target target;
  target.setId(targetMsg.id);
  target.setPosition(std::make_tuple(targetMsg.x_position, targetMsg.y_position));
  target.setColor(COLOR_MAP.right.find(targetMsg.color)->second);
  return target;
}
