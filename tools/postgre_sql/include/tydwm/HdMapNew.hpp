#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "HdMapElement.h"

namespace fs = std::filesystem;

namespace tydwm {

class HdMapNew {
public:
  HdMapNew();
  ~HdMapNew() = default;

  bool Load(const std::string &folder);

  // Local origin methods
  void setLocalOrigin(const Eigen::Vector3f &origin);
  Eigen::Vector3f getLocalOrigin() const;

  std::vector<Eigen::Vector3f>
  GetPoints(std::vector<std::string> element_types);

  // Getters for different element types
  const std::vector<PolyLane> &GetLanes() const { return m_lanes; }
  const std::vector<PolyArrow> &GetArrows() const { return m_arrows; }
  const std::vector<PolyBumper> &GetBumpers() const { return m_bumpers; }
  const std::vector<PolyJunction> &GetJunctions() const { return m_junctions; }
  const std::vector<PolyLaneNode> &GetLaneNodes() const { return m_lane_nodes; }
  const std::vector<PolyClearZone> &GetClearZones() const {
    return m_clear_zones;
  }
  const std::vector<PolyCurb> &GetCurbs() const { return m_curbs; }
  const std::vector<PolyBoundary> &GetBoundaries() const {
    return m_boundaries;
  }

  // Utility functions
  bool HasCoordinateTransform() const {
    return m_localOrigin != Eigen::Vector3f::Zero();
  }
  void SetCoordinateTransform(double origin_x, double origin_y,
                              double origin_z = 0.0);

private:
  fs::path m_HdMapFolder;

  Eigen::Vector3f m_localOrigin; // origin of local map

  // Element containers
  std::vector<PolyLane> m_lanes;
  std::vector<PolyArrow> m_arrows;
  std::vector<PolyBumper> m_bumpers;
  std::vector<PolyJunction> m_junctions;
  std::vector<PolyLaneNode> m_lane_nodes;
  std::vector<PolyClearZone> m_clear_zones;
  std::vector<PolyCurb> m_curbs;
  std::vector<PolyBoundary> m_boundaries;

  // File name mapping for new format
  std::unordered_map<std::string, std::string> m_element_file_map = {
      {"lane", "lane.geojson"},           {"arrow", "arrow.geojson"},
      {"bumper", "bumper.geojson"},       {"junction", "junction.geojson"},
      {"lane_node", "lane_node.geojson"}, {"clear_zone", "clearzone.geojson"},
      {"curb", "curb.geojson"},           {"boundary", "boundary.geojson"}};

  // Helper functions
  bool LoadLanes();
  bool LoadArrows();
  bool LoadBumpers();
  bool LoadJunctions();
  bool LoadLaneNodes();
  bool LoadClearZones();
  bool LoadCurbs();
  bool LoadBoundaries();

  // Coordinate transformation helper
  Eigen::Vector3f TransformCoordinate(double x, double y, double z) const;

  // JSON parsing helpers
  bool ParseJsonDocument(const fs::path &filepath, rapidjson::Document &doc);
  std::vector<Eigen::Vector3f>
  ParseCoordinates(const rapidjson::Value &coordinates);
  std::vector<std::vector<Eigen::Vector3f>>
  ParsePolygonCoordinates(const rapidjson::Value &coordinates);
};

} // namespace tydwm