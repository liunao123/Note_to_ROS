#pragma once
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "HdMapElement.h"

namespace fs = std::filesystem;

namespace tydwm {

class HdMap {
private:
  fs::path m_HdMapFolder;
  std::vector<std::string> m_Elements;

  Eigen::Vector3f m_localOrigin; //  origin of local map

  // all the elements are under local map coordinate system
  std::vector<PolyLane> m_Lanes;
  std::vector<PolyRoadSide> m_RoadSides;
  std::vector<PolyArrow> m_Arrows;
  std::vector<PolyPole> m_Poles;
  std::vector<PolyRoadArea> m_RoadAreas;
  std::vector<PolyMarkingLineString> m_MarkingLineStrings;
  std::vector<PolyCrosswalk> m_Crosswalks;
  std::vector<PolyLink> m_Links;
  std::vector<PolyLinkNode> m_LinkNodes;
  std::vector<PolyRoadSurface> m_RoadSurfaces;
  std::vector<PolyStopline> m_Stoplines;
  std::vector<PolyRoadHhLine> m_RoadHhLines;
  std::vector<PolyTrafficSign> m_TrafficSigns;
  std::vector<PolyTrafficLight> m_TrafficLights;

  void LoadLane();
  void LoadRoadSide();
  void LoadArrow();
  void LoadPole();
  void LoadRoadArea();
  void LoadMarkingLineString();
  void LoadTrafficSigns();

public:
  HdMap();
  ~HdMap();

  void Load(const fs::path &hdmap_folder,
            const std::vector<std::string> &elements);

  void setLocalOrigin(const Eigen::Vector3f &origin);
  Eigen::Vector3f getLocalOrigin() const;

  const std::vector<PolyLane> &getLanes() const;
  const std::vector<PolyRoadSide> &getRoadSides() const;
  const std::vector<PolyArrow> &getArrows() const;
  const std::vector<PolyPole> &getPoles() const;
  const std::vector<PolyRoadArea> &getRoadAreas() const;
  const std::vector<PolyMarkingLineString> &getMarkingLineStrings() const;
  const std::vector<PolyTrafficSign> &getTrafficSigns() const;

  void printInfo() const;
};

} // namespace tydwm