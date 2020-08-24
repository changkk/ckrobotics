#ifndef REGION_OF_INTEREST_H
#define REGION_OF_INTEREST_H

#include <vector>
#include <string>

class RegionOfInterest
{
public:
  RegionOfInterest(std::string filename);
  RegionOfInterest(const RegionOfInterest& regionOfInterest);
  ~RegionOfInterest();
  bool insideRegion(float testX, float testY) const;
  bool isValidRegion() const;
  std::tuple<float, float> getVertex(unsigned int index) const;

private:
  RegionOfInterest();
  void loadRegionOfInterest(std::string filename, std::vector<std::tuple<float, float>>& verticies);
  void convertVerticiesToArrays(const std::vector<std::tuple<float, float>>& verticies);
  static int pnpoly(int nvert, const float* vertx, const float* verty, float testx, float testy);

  std::string mName;
  bool mIsValidRegion;
  int mNumOfVertices;
  float* mVerticesX;
  float* mVerticesY;
};

#endif
