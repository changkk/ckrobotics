#ifndef TARGET_H
#define TARGET_H

#include <perception/TargetMsg.h>

#include <ros/time.h>
#include <tuple>
#include <vector>

class Target {
public:
  enum class Color { NONE, RED, GREEN, BLUE, YELLOW, ORANGE };

  Target();
  bool operator==(const Target& rhs) const;

  void addObservation(const ros::Time& time);
  bool confirmedTarget() const;

  unsigned int getId() const;
  Color getColor() const;
  std::tuple<float, float> getPosition() const;

  void setId(unsigned int id);
  void setColor(Color color);
  void setPosition(std::tuple<float, float> position);

  perception::TargetMsg serialize() const;
  static Target deserialize(const perception::TargetMsg& targetMsg);

private:
  unsigned int mId;
  bool mIsConfirmed;
  Color mColor;
  std::tuple<float, float> mPosition;
  std::vector<std::tuple<float, float>> mPositionHistory;
  std::vector<ros::Time> mObservationTimes;
};

#endif
