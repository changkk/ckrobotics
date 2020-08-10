#include <perception/region_of_interest.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <tuple>

RegionOfInterest::RegionOfInterest(std::string filename) {
  std::vector<std::tuple<float, float>> verticies;
  this->mIsValidRegion = false;
  this->loadRegionOfInterest(filename, verticies);
  this->mNumOfVertices = verticies.size();
  this->mVerticesX = new float[this->mNumOfVertices];
  this->mVerticesY = new float[this->mNumOfVertices];
  this->convertVerticiesToArrays(verticies);
}

RegionOfInterest::RegionOfInterest(const RegionOfInterest& regionOfInterest) {
  this->mName = regionOfInterest.mName;
  this->mIsValidRegion = regionOfInterest.mIsValidRegion;
  this->mNumOfVertices = regionOfInterest.mNumOfVertices;
  this->mVerticesX = new float[this->mNumOfVertices];
  this->mVerticesY = new float[this->mNumOfVertices];
  for (int i = 0; i < this->mNumOfVertices; ++i) {
    this->mVerticesX[i] = regionOfInterest.mVerticesX[i];
    this->mVerticesY[i] = regionOfInterest.mVerticesY[i];
  }
}

RegionOfInterest::~RegionOfInterest() {
  if (this->mVerticesX != NULL) {
    delete[] this->mVerticesX;
    this->mVerticesX = NULL;
  }
  if (this->mVerticesY != NULL) {
    delete[] this->mVerticesY;
    this->mVerticesY = NULL;
  }
}

bool RegionOfInterest::insideRegion(float testX, float testY) const {
  return RegionOfInterest::pnpoly(4, this->mVerticesX, this->mVerticesY, testX, testY) == 1;
}

bool RegionOfInterest::isValidRegion() const {
  return this->mIsValidRegion;
}

std::tuple<float, float> RegionOfInterest::getVertex(unsigned int index) const {
  std::tuple<float, float> vertex;
  if (index < this->mNumOfVertices) {
    std::get<0>(vertex) = this->mVerticesX[index];
    std::get<1>(vertex) = this->mVerticesY[index];
  }
  return vertex;
}

void RegionOfInterest::loadRegionOfInterest(std::string filename, std::vector<std::tuple<float, float>>& verticies) {
  std::ifstream file(filename);
  std::string line;
  if (file.is_open()) {
    std::getline(file, line);
    std::stringstream ss(line);
    ss >> this->mName >> this->mIsValidRegion;
    while (std::getline(file, line)) {
      std::tuple<float, float> vertex;
      std::stringstream ss(line);
      ss >> std::get<0>(vertex) >> std::get<1>(vertex);
      verticies.push_back(vertex);
    }
    file.close();
  }
}

void RegionOfInterest::convertVerticiesToArrays(const std::vector<std::tuple<float, float>>& verticies) {
  int i = 0;
  for (std::vector<std::tuple<float, float>>::const_iterator iter = verticies.begin(); iter != verticies.end(); ++iter) {
    this->mVerticesX[i] = std::get<0>(*iter);
    this->mVerticesY[i] = std::get<1>(*iter);
    // std::cout << "Vertex (" << i << ") " << this->mVerticesX[i] << ", " << this->mVerticesY[i] << std::endl;
    ++i;
  }
}

// Borrowed from the internets
int RegionOfInterest::pnpoly(int nvert, const float* vertx, const float* verty, float testx, float testy) {
  int i, j, c = 0;
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i])) {
      c = !c;
    }
  }
  return c;
}
