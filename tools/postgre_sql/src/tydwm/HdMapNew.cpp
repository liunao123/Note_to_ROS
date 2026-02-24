#include "tydwm/HdMapNew.hpp"

namespace tydwm {

HdMapNew::HdMapNew() : m_localOrigin(Eigen::Vector3f(0, 0, 0)) {}

void HdMapNew::setLocalOrigin(const Eigen::Vector3f &origin) {
  this->m_localOrigin = origin;
}

Eigen::Vector3f HdMapNew::getLocalOrigin() const { return this->m_localOrigin; }

bool HdMapNew::Load(const std::string &folder) {
  m_HdMapFolder = fs::path(folder);

  if (!fs::exists(m_HdMapFolder) || !fs::is_directory(m_HdMapFolder)) {
    std::cerr << "HD Map folder does not exist: " << folder << std::endl;
    return false;
  }

  // Clear existing data
  m_lanes.clear();
  m_arrows.clear();
  m_bumpers.clear();
  m_junctions.clear();
  m_lane_nodes.clear();
  m_clear_zones.clear();
  m_curbs.clear();
  m_boundaries.clear();

  // Load all element types
  bool success = true;
  success &= LoadLanes();
  success &= LoadArrows();
  success &= LoadBumpers();
  success &= LoadJunctions();
  success &= LoadLaneNodes();
  success &= LoadClearZones();
  success &= LoadCurbs();
  success &= LoadBoundaries();

  return success;
}

void HdMapNew::SetCoordinateTransform(double origin_x, double origin_y,
                                      double origin_z) {
  setLocalOrigin(Eigen::Vector3f(origin_x, origin_y, origin_z));
}

std::vector<Eigen::Vector3f>
HdMapNew::GetPoints(std::vector<std::string> element_types) {
  std::vector<Eigen::Vector3f> points;
  for (const auto &type : element_types) {
    if (type == "lane") {
      for (const auto &lane : m_lanes) {
        points.insert(points.end(), lane.points.begin(), lane.points.end());
      }
    } else if (type == "arrow") {
      for (const auto &arrow : m_arrows) {
        for (const auto &polygon : arrow.polygons) {
          points.insert(points.end(), polygon.begin(), polygon.end());
        }
      }
    } else if (type == "bumper") {
      for (const auto &bumper : m_bumpers) {
        for (const auto &polygon : bumper.polygons) {
          points.insert(points.end(), polygon.begin(), polygon.end());
        }
      }
    } else if (type == "junction") {
      // Junctions do not have point data
      continue;
    } else if (type == "lane_node") {
      // LaneNodes do not have point data
      continue;
    } else if (type == "clear_zone") {
      for (const auto &cz : m_clear_zones) {
        for (const auto &polygon : cz.polygons) {
          points.insert(points.end(), polygon.begin(), polygon.end());
        }
      }
    } else if (type == "curb") {
      for (const auto &curb : m_curbs) {
        for (const auto &point : curb.points) {
          points.push_back(point);
        }
      }
    } else if (type == "boundary") {
      for (const auto &boundary : m_boundaries) {
        for (const auto &point : boundary.points) {
          points.push_back(point);
        }
      }
    }
  }
  return points;
}

Eigen::Vector3f HdMapNew::TransformCoordinate(double x, double y,
                                              double z) const {
  return Eigen::Vector3f(x - m_localOrigin.x(), y - m_localOrigin.y(),
                         z - m_localOrigin.z());
}

bool HdMapNew::ParseJsonDocument(const fs::path &filepath,
                                 rapidjson::Document &doc) {
  if (!fs::exists(filepath)) {
    std::cerr << "File does not exist: " << filepath << std::endl;
    return false;
  }

  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Cannot open file: " << filepath << std::endl;
    return false;
  }

  // Read entire file into string
  std::string json_str((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
  file.close();

  // Parse JSON
  doc.Parse(json_str.c_str());
  if (doc.HasParseError()) {
    std::cerr << "JSON parse error in file: " << filepath
              << " Error: " << doc.GetParseError() << std::endl;
    return false;
  }

  return true;
}

std::vector<Eigen::Vector3f>
HdMapNew::ParseCoordinates(const rapidjson::Value &coordinates) {
  std::vector<Eigen::Vector3f> points;

  if (!coordinates.IsArray()) {
    return points;
  }

  for (const auto &coord : coordinates.GetArray()) {
    if (coord.IsArray() && coord.Size() >= 2) {
      double x = coord[0].GetDouble();
      double y = coord[1].GetDouble();
      double z = coord.Size() > 2 ? coord[2].GetDouble() : 0.0;
      points.push_back(TransformCoordinate(x, y, z));
    }
  }

  return points;
}

std::vector<std::vector<Eigen::Vector3f>>
HdMapNew::ParsePolygonCoordinates(const rapidjson::Value &coordinates) {
  std::vector<std::vector<Eigen::Vector3f>> rings;

  if (!coordinates.IsArray()) {
    return rings;
  }

  for (const auto &ring : coordinates.GetArray()) {
    if (ring.IsArray()) {
      rings.push_back(ParseCoordinates(ring));
    }
  }

  return rings;
}

bool HdMapNew::LoadLanes() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["lane"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load lanes from " << filepath << std::endl;
    return true; // Not critical failure
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid lanes GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    if (!geometry.HasMember("coordinates") ||
        !geometry["coordinates"].IsArray()) {
      continue;
    }

    PolyLane lane;

    // Parse properties
    if (properties.HasMember("lane_id")) {
      if (properties["lane_id"].IsString()) {
        lane.lane_id = properties["lane_id"].GetString();
      } else if (properties["lane_id"].IsInt()) {
        lane.lane_id = std::to_string(properties["lane_id"].GetInt());
      }
    }
    if (properties.HasMember("mesh_id")) {
      if (properties["mesh_id"].IsString()) {
        // Convert string to int64_t for mesh_id
        std::string mesh_str = properties["mesh_id"].GetString();
        lane.mesh_id = std::stoll(mesh_str);
      } else if (properties["mesh_id"].IsInt()) {
        lane.mesh_id = properties["mesh_id"].GetInt();
      }
    }
    if (properties.HasMember("link_id") && properties["link_id"].IsString()) {
      lane.link_id = properties["link_id"].GetString();
    }
    if (properties.HasMember("snode_id")) {
      if (properties["snode_id"].IsString()) {
        lane.snode_id = properties["snode_id"].GetString();
      } else if (properties["snode_id"].IsInt()) {
        lane.snode_id = std::to_string(properties["snode_id"].GetInt());
      }
    }
    if (properties.HasMember("enode_id")) {
      if (properties["enode_id"].IsString()) {
        lane.enode_id = properties["enode_id"].GetString();
      } else if (properties["enode_id"].IsInt()) {
        lane.enode_id = std::to_string(properties["enode_id"].GetInt());
      }
    }
    if (properties.HasMember("direction") && properties["direction"].IsInt()) {
      lane.direction = properties["direction"].GetInt();
    }
    if (properties.HasMember("lane_seq") && properties["lane_seq"].IsInt()) {
      lane.lane_seq = properties["lane_seq"].GetInt();
    }
    if (properties.HasMember("width") && properties["width"].IsNumber()) {
      lane.width = properties["width"].GetDouble();
    }

    // Parse geometry
    const auto &coordinates = geometry["coordinates"];
    if (geometry.HasMember("type") &&
        std::string(geometry["type"].GetString()) == "Polygon") {
      auto rings = ParsePolygonCoordinates(coordinates);
      if (!rings.empty()) {
        lane.points = rings[0]; // Use first ring as points
      }
    } else if (geometry.HasMember("type") &&
               std::string(geometry["type"].GetString()) == "LineString") {
      lane.points = ParseCoordinates(coordinates);
    }

    if (!lane.points.empty()) {
      m_lanes.push_back(lane);
    }
  }

  std::cout << "Loaded " << m_lanes.size() << " lanes" << std::endl;
  return true;
}

bool HdMapNew::LoadArrows() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["arrow"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load arrows from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid arrows GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyArrow arrow;

    // Parse properties
    if (properties.HasMember("obj_id") && properties["obj_id"].IsString()) {
      arrow.obj_id = properties["obj_id"].GetString();
    }
    if (properties.HasMember("link_id") && properties["link_id"].IsString()) {
      arrow.link_ids = properties["link_id"].GetString();
    }
    if (properties.HasMember("type") && properties["type"].IsString()) {
      // Convert string to int64_t for type
      std::string type_str = properties["type"].GetString();
      try {
        arrow.type = std::stoll(type_str);
      } catch (...) {
        arrow.type = 0; // Default value if conversion fails
      }
    }

    // Parse geometry
    if (geometry.HasMember("coordinates") &&
        geometry["coordinates"].IsArray()) {
      if (geometry.HasMember("type") &&
          std::string(geometry["type"].GetString()) == "Polygon") {
        auto rings = ParsePolygonCoordinates(geometry["coordinates"]);
        if (!rings.empty()) {
          arrow.polygons = rings;
        }
      }
    }

    if (!arrow.polygons.empty()) {
      m_arrows.push_back(arrow);
    }
  }

  std::cout << "Loaded " << m_arrows.size() << " arrows" << std::endl;
  return true;
}

bool HdMapNew::LoadBumpers() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["bumper"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load bumpers from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid bumpers GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyBumper bumper;

    // Parse properties
    if (properties.HasMember("obj_id") && properties["obj_id"].IsString()) {
      bumper.obj_id = properties["obj_id"].GetString();
    }

    // Parse geometry
    if (geometry.HasMember("coordinates") &&
        geometry["coordinates"].IsArray()) {
      if (geometry.HasMember("type") &&
          std::string(geometry["type"].GetString()) == "Polygon") {
        auto rings = ParsePolygonCoordinates(geometry["coordinates"]);
        if (!rings.empty()) {
          bumper.polygons = rings;
        }
      }
    }

    if (!bumper.polygons.empty()) {
      m_bumpers.push_back(bumper);
    }
  }

  std::cout << "Loaded " << m_bumpers.size() << " bumpers" << std::endl;
  return true;
}

bool HdMapNew::LoadJunctions() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["junction"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load junctions from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid junctions GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyJunction junction;

    // Parse properties
    if (properties.HasMember("obj_id") && properties["obj_id"].IsString()) {
      junction.jc_id = properties["obj_id"].GetString();
    }

    // Parse geometry - Note: PolyJunction doesn't have polygons field in
    // definition We'll skip geometry parsing for now since it's not in the
    // structure The original structure only has area_cd and road names but no
    // geometry

    // Always add junction since we have at least an ID
    m_junctions.push_back(junction);
  }

  std::cout << "Loaded " << m_junctions.size() << " junctions" << std::endl;
  return true;
}

bool HdMapNew::LoadLaneNodes() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["lane_node"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load lane nodes from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid lane nodes GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyLaneNode node;

    // Parse properties
    if (properties.HasMember("node_id") && properties["node_id"].IsString()) {
      node.node_id = properties["node_id"].GetString();
    }

    // Parse geometry - Note: PolyLaneNode doesn't have points field, only
    // basic properties We'll skip geometry parsing for this type since it's
    // not in the structure

    // Always add node since we have at least an ID
    m_lane_nodes.push_back(node);
  }

  std::cout << "Loaded " << m_lane_nodes.size() << " lane nodes" << std::endl;
  return true;
}

bool HdMapNew::LoadClearZones() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["clear_zone"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load clear zones from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid clear zones GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyClearZone clearzone;

    // Parse properties
    if (properties.HasMember("obj_id") && properties["obj_id"].IsString()) {
      clearzone.obj_id = properties["obj_id"].GetString();
    }

    // Parse geometry
    if (geometry.HasMember("coordinates") &&
        geometry["coordinates"].IsArray()) {
      if (geometry.HasMember("type") &&
          std::string(geometry["type"].GetString()) == "Polygon") {
        auto rings = ParsePolygonCoordinates(geometry["coordinates"]);
        if (!rings.empty()) {
          clearzone.polygons = rings;
        }
      }
    }

    if (!clearzone.polygons.empty()) {
      m_clear_zones.push_back(clearzone);
    }
  }

  std::cout << "Loaded " << m_clear_zones.size() << " clear zones" << std::endl;
  return true;
}

bool HdMapNew::LoadCurbs() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["curb"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load curbs from " << filepath << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid curbs GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyCurb curb;

    // Parse properties
    if (properties.HasMember("obj_id") && properties["obj_id"].IsString()) {
      curb.obj_id = properties["obj_id"].GetString();
    }

    // Parse geometry
    if (geometry.HasMember("coordinates") &&
        geometry["coordinates"].IsArray()) {
      if (geometry.HasMember("type") &&
          std::string(geometry["type"].GetString()) == "LineString") {
        curb.points = ParseCoordinates(geometry["coordinates"]);
      }
    }

    if (!curb.points.empty()) {
      m_curbs.push_back(curb);
    }
  }

  std::cout << "Loaded " << m_curbs.size() << " curbs" << std::endl;
  return true;
}

bool HdMapNew::LoadBoundaries() {
  fs::path filepath = m_HdMapFolder / m_element_file_map["boundary"];
  rapidjson::Document doc;

  if (!ParseJsonDocument(filepath, doc)) {
    std::cout << "Warning: Could not load boundaries from " << filepath
              << std::endl;
    return true;
  }

  if (!doc.HasMember("features") || !doc["features"].IsArray()) {
    std::cerr << "Invalid boundaries GeoJSON format" << std::endl;
    return false;
  }

  for (const auto &feature : doc["features"].GetArray()) {
    if (!feature.HasMember("geometry") || !feature.HasMember("properties")) {
      continue;
    }

    const auto &geometry = feature["geometry"];
    const auto &properties = feature["properties"];

    PolyBoundary boundary;

    // Parse properties
    if (properties.HasMember("marking_id") &&
        properties["marking_id"].IsString()) {
      boundary.obj_id = properties["marking_id"].GetString();
    }
    if (properties.HasMember("type") && properties["type"].IsString()) {
      // Convert string to int64_t for type
      std::string type_str = properties["type"].GetString();
      try {
        boundary.type = std::stoll(type_str);
      } catch (...) {
        boundary.type = 0; // Default value if conversion fails
      }
    }

    // Parse geometry
    if (geometry.HasMember("coordinates") &&
        geometry["coordinates"].IsArray()) {
      if (geometry.HasMember("type") &&
          std::string(geometry["type"].GetString()) == "LineString") {
        boundary.points = ParseCoordinates(geometry["coordinates"]);
      }
    }

    if (!boundary.points.empty()) {
      m_boundaries.push_back(boundary);
    }
  }

  std::cout << "Loaded " << m_boundaries.size() << " boundaries" << std::endl;
  return true;
}

} // namespace tydwm