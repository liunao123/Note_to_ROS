#include "HdMap.hpp"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <fstream>
#include <sstream>

namespace tydwm {

HdMap::HdMap() : m_localOrigin(Eigen::Vector3f(0, 0, 0)) {}

HdMap::~HdMap() {
  //
}

void HdMap::setLocalOrigin(const Eigen::Vector3f &origin) {
  this->m_localOrigin = origin;
}

Eigen::Vector3f HdMap::getLocalOrigin() const {
  //
  return this->m_localOrigin;
}

const std::vector<PolyLane> &HdMap::getLanes() const { return m_Lanes; }
const std::vector<PolyRoadSide> &HdMap::getRoadSides() const {
  return m_RoadSides;
}
const std::vector<PolyArrow> &HdMap::getArrows() const { return m_Arrows; }
const std::vector<PolyPole> &HdMap::getPoles() const { return m_Poles; }
const std::vector<PolyRoadArea> &HdMap::getRoadAreas() const {
  return m_RoadAreas;
}
const std::vector<PolyMarkingLineString> &HdMap::getMarkingLineStrings() const {
  return m_MarkingLineStrings;
}
const std::vector<PolyTrafficSign> &HdMap::getTrafficSigns() const {
  return m_TrafficSigns;
}

void HdMap::Load(const fs::path &hdmap_folder,
                 const std::vector<std::string> &elements) {
  m_HdMapFolder = hdmap_folder;
  m_Elements = elements;

  for (const auto &element : m_Elements) {
    fs::path filepath = m_HdMapFolder / ("hd_" + element + ".json.geojson");
    if (fs::exists(filepath)) {
      std::cout << "Loading HD Map element from: " << filepath << std::endl;
      if (element == "lane") {
        LoadLane();
      } else if (element == "road_side") {
        LoadRoadSide();
      } else if (element == "arrow") {
        LoadArrow();
      } else if (element == "pole") {
        LoadPole();
      } else if (element == "road_area") {
        LoadRoadArea();
      } else if (element == "marking_line_string") {
        LoadMarkingLineString();
      } else if (element == "traffic_sign") {
        LoadTrafficSigns();
      } else {
        std::cout << "Warning: Unsupported HD Map element: " << element
                  << std::endl;
      }
    }
  }
}

void HdMap::printInfo() const {
  std::cout << "\n=== HdMap Information ===" << std::endl;
  std::cout << "HD Map Folder: " << m_HdMapFolder << std::endl;
  std::cout << "Total Lanes: " << m_Lanes.size() << std::endl;
  std::cout << "Total Road Sides: " << m_RoadSides.size() << std::endl;
  std::cout << "Total Arrows: " << m_Arrows.size() << std::endl;
  std::cout << "Total Poles: " << m_Poles.size() << std::endl;
  std::cout << "Total Road Areas: " << m_RoadAreas.size() << std::endl;
  std::cout << "Total Marking Line Strings: " << m_MarkingLineStrings.size()
            << std::endl;
  std::cout << "Total Traffic Signs: " << m_TrafficSigns.size() << std::endl;

  size_t printCount = 3;
  if (!m_Lanes.empty()) {
    std::cout << "\nLane Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_Lanes.size(), printCount); ++i) {
      const auto &lane = m_Lanes[i];
      std::cout << "  Lane " << i + 1 << ":" << std::endl;
      std::cout << "    Lane ID: " << lane.lane_id << std::endl;
      std::cout << "    Link ID: " << lane.link_id << std::endl;
      std::cout << "    Type: " << lane.lane_type << std::endl;
      std::cout << "    Length: " << lane.length << " m" << std::endl;
      std::cout << "    Width: " << lane.width << " m" << std::endl;
      std::cout << "    Lane Sequence: " << lane.lane_seq << std::endl;
      std::cout << "    Direction: " << lane.direction << std::endl;
      std::cout << "    Turn Info: " << lane.turn_info << std::endl;
      std::cout << "    Speed Max: " << lane.spd_max << " km/h" << std::endl;
      std::cout << "    Speed Min: " << lane.spd_min << " km/h" << std::endl;
      std::cout << "    Is Junction: " << (lane.is_junc ? "Yes" : "No")
                << std::endl;
      std::cout << "    Road Name: " << (!lane.name.empty() ? lane.name : "N/A")
                << std::endl;
      std::cout << "    Points: " << lane.points.size() << std::endl;
    }

    if (m_Lanes.size() > printCount) {
      std::cout << "  ... and " << (m_Lanes.size() - printCount)
                << " more lanes" << std::endl;
    }
  }

  if (!m_RoadSides.empty()) {
    std::cout << "\nRoad Side Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_RoadSides.size(), printCount); ++i) {
      const auto &roadSide = m_RoadSides[i];
      std::cout << "  Road Side " << i + 1 << ":" << std::endl;
      std::cout << "    Object ID: " << roadSide.obj_id << std::endl;
      std::cout << "    Link IDs: " << roadSide.link_ids << std::endl;
      std::cout << "    Lane IDs: " << roadSide.lane_ids << std::endl;
      std::cout << "    Type: " << roadSide.type;

      // 添加类型说明
      switch (roadSide.type) {
      case 61:
        std::cout << " (路缘石)";
        break;
      case 62:
        std::cout << " (护栏)";
        break;
      case 63:
        std::cout << " (防护墙)";
        break;
      case 64:
        std::cout << " (隔离带)";
        break;
      case 65:
        std::cout << " (绿化带)";
        break;
      case 66:
        std::cout << " (声屏障)";
        break;
      case 67:
        std::cout << " (水泥隔离墩)";
        break;
      case 68:
        std::cout << " (塑料隔离墩)";
        break;
      case 69:
        std::cout << " (施工隔离墙)";
        break;
      case 80:
        std::cout << " (其它)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Junction IDs: "
                << (!roadSide.rjuc_ids.empty() ? roadSide.rjuc_ids : "N/A")
                << std::endl;
      std::cout << "    Area Code: " << roadSide.area_cd << std::endl;
      std::cout << "    Points: " << roadSide.points.size() << std::endl;
    }

    if (m_RoadSides.size() > printCount) {
      std::cout << "  ... and " << (m_RoadSides.size() - printCount)
                << " more road sides" << std::endl;
    }
  }

  if (!m_Arrows.empty()) {
    std::cout << "\nArrow Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_Arrows.size(), printCount); ++i) {
      const auto &arrow = m_Arrows[i];
      std::cout << "  Arrow " << i + 1 << ":" << std::endl;
      std::cout << "    Object ID: " << arrow.obj_id << std::endl;
      std::cout << "    Link IDs: " << arrow.link_ids << std::endl;
      std::cout << "    Lane IDs: " << arrow.lane_ids << std::endl;
      std::cout << "    Type: " << arrow.type;

      // 添加类型说明
      switch (arrow.type) {
      case 110:
        std::cout << " (直行)";
        break;
      case 111:
        std::cout << " (左转)";
        break;
      case 112:
        std::cout << " (右转)";
        break;
      case 113:
        std::cout << " (左直行)";
        break;
      case 114:
        std::cout << " (右直行)";
        break;
      case 115:
        std::cout << " (左右转)";
        break;
      case 116:
        std::cout << " (掉头)";
        break;
      case 117:
        std::cout << " (左转掉头)";
        break;
      case 118:
        std::cout << " (左直掉头)";
        break;
      case 119:
        std::cout << " (右转掉头)";
        break;
      case 120:
        std::cout << " (右直掉头)";
        break;
      case 121:
        std::cout << " (左右掉头)";
        break;
      case 122:
        std::cout << " (左直右)";
        break;
      case 123:
        std::cout << " (左右直掉头)";
        break;
      case 124:
        std::cout << " (全方向)";
        break;
      case 125:
        std::cout << " (公交专用)";
        break;
      case 126:
        std::cout << " (非机动车)";
        break;
      case 127:
        std::cout << " (出租车)";
        break;
      case 128:
        std::cout << " (应急车道)";
        break;
      case 129:
        std::cout << " (货车专用)";
        break;
      case 130:
        std::cout << " (HOV车道)";
        break;
      case 131:
        std::cout << " (ETC专用)";
        break;
      case 132:
        std::cout << " (收费)";
        break;
      case 133:
        std::cout << " (停车)";
        break;
      case 134:
        std::cout << " (停车位箭头)";
        break;
      case 150:
        std::cout << " (其它)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Color: " << arrow.color;
      switch (arrow.color) {
      case 1:
        std::cout << " (白色)";
        break;
      case 2:
        std::cout << " (黄色)";
        break;
      case 3:
        std::cout << " (红色)";
        break;
      case 4:
        std::cout << " (绿色)";
        break;
      case 5:
        std::cout << " (蓝色)";
        break;
      case 6:
        std::cout << " (橘色)";
        break;
      default:
        std::cout << " (未知颜色)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Junction IDs: "
                << (!arrow.rjuc_ids.empty() ? arrow.rjuc_ids : "N/A")
                << std::endl;
      std::cout << "    Area Code: " << arrow.area_cd << std::endl;
      std::cout << "    Polygons: " << arrow.polygons.size() << std::endl;

      // 显示总顶点数
      size_t total_vertices = 0;
      for (const auto &polygon : arrow.polygons) {
        total_vertices += polygon.size();
      }
      std::cout << "    Total Vertices: " << total_vertices << std::endl;
    }

    if (m_Arrows.size() > printCount) {
      std::cout << "  ... and " << (m_Arrows.size() - printCount)
                << " more arrows" << std::endl;
    }
  }

  if (!m_Poles.empty()) {
    std::cout << "\nPole Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_Poles.size(), printCount); ++i) {
      const auto &pole = m_Poles[i];
      std::cout << "  Pole " << i + 1 << ":" << std::endl;
      std::cout << "    Object ID: " << pole.obj_id << std::endl;
      std::cout << "    Link IDs: " << pole.link_ids << std::endl;
      std::cout << "    Lane IDs: " << pole.lane_ids << std::endl;
      std::cout << "    Type: " << pole.type;

      // 添加类型说明
      switch (pole.type) {
      case 41:
        std::cout << " (路灯杆)";
        break;
      case 42:
        std::cout << " (标志杆)";
        break;
      case 43:
        std::cout << " (信号杆)";
        break;
      case 44:
        std::cout << " (监控杆)";
        break;
      case 45:
        std::cout << " (电力杆)";
        break;
      case 46:
        std::cout << " (通信杆)";
        break;
      case 47:
        std::cout << " (横杆)";
        break;
      case 60:
        std::cout << " (其它)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Area Code: " << pole.area_cd << std::endl;
      std::cout << "    Points: " << pole.points.size() << std::endl;
    }

    if (m_Poles.size() > printCount) {
      std::cout << "  ... and " << (m_Poles.size() - printCount)
                << " more poles" << std::endl;
    }
  }

  if (!m_RoadAreas.empty()) {
    std::cout << "\nRoad Area Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_RoadAreas.size(), printCount); ++i) {
      const auto &roadArea = m_RoadAreas[i];
      std::cout << "  Road Area " << i + 1 << ":" << std::endl;
      std::cout << "    Object ID: " << roadArea.obj_id << std::endl;
      std::cout << "    Link IDs: " << roadArea.link_ids << std::endl;
      std::cout << "    Lane IDs: " << roadArea.lane_ids << std::endl;
      std::cout << "    Type: " << roadArea.type;

      // 添加类型说明
      switch (roadArea.type) {
      case 3:
        std::cout << " (导流带)";
        break;
      case 4:
        std::cout << " (禁止通行区)";
        break;
      case 5:
        std::cout << " (停车区)";
        break;
      case 6:
        std::cout << " (减速带)";
        break;
      case 7:
        std::cout << " (斑马线预告区)";
        break;
      case 8:
        std::cout << " (导向车道区)";
        break;
      case 9:
        std::cout << " (网格线区)";
        break;
      case 10:
        std::cout << " (标识区)";
        break;
      case 11:
        std::cout << " (变速车道)";
        break;
      case 12:
        std::cout << " (收费站)";
        break;
      case 20:
        std::cout << " (其它)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Junction IDs: "
                << (!roadArea.rjuc_ids.empty() ? roadArea.rjuc_ids : "N/A")
                << std::endl;
      std::cout << "    Area Code: " << roadArea.area_cd << std::endl;
      std::cout << "    Polygons: " << roadArea.polygons.size() << std::endl;

      // 显示总顶点数
      size_t total_vertices = 0;
      for (const auto &polygon : roadArea.polygons) {
        total_vertices += polygon.size();
      }
      std::cout << "    Total Vertices: " << total_vertices << std::endl;
    }

    if (m_RoadAreas.size() > printCount) {
      std::cout << "  ... and " << (m_RoadAreas.size() - printCount)
                << " more road areas" << std::endl;
    }
  }

  if (!m_MarkingLineStrings.empty()) {
    std::cout << "\nMarking Line String Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_MarkingLineStrings.size(), printCount);
         ++i) {
      const auto &marking = m_MarkingLineStrings[i];
      std::cout << "  Marking " << i + 1 << ":" << std::endl;
      std::cout << "    Marking ID: " << marking.marking_id << std::endl;
      std::cout << "    Link ID: " << marking.link_id << std::endl;
      std::cout << "    Line ID: " << marking.line_id << std::endl;
      std::cout << "    Type: " << marking.type;

      // 添加类型说明
      switch (marking.type) {
      case 0:
        std::cout << " (虚拟)";
        break;
      case 1:
        std::cout << " (单实线)";
        break;
      case 2:
        std::cout << " (双实线)";
        break;
      case 3:
        std::cout << " (虚线)";
        break;
      case 4:
        std::cout << " (虚实线)";
        break;
      case 5:
        std::cout << " (实虚线)";
        break;
      case 6:
        std::cout << " (双虚线)";
        break;
      case 7:
        std::cout << " (导向线)";
        break;
      case 8:
        std::cout << " (变道线)";
        break;
      case 9:
        std::cout << " (潮汐线)";
        break;
      case 10:
        std::cout << " (可变车道线)";
        break;
      case 99:
        std::cout << " (其他)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Color: " << marking.color;
      switch (marking.color) {
      case 1:
        std::cout << " (白色)";
        break;
      case 2:
        std::cout << " (黄色)";
        break;
      case 3:
        std::cout << " (红色)";
        break;
      case 4:
        std::cout << " (绿色)";
        break;
      case 5:
        std::cout << " (蓝色)";
        break;
      case 6:
        std::cout << " (橘色)";
        break;
      case 99:
        std::cout << " (其他)";
        break;
      default:
        std::cout << " (未知颜色)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    Length: " << marking.length << " m" << std::endl;
      std::cout << "    Width: " << marking.width << " m" << std::endl;
      std::cout << "    Is Junction: " << (marking.is_junc ? "Yes" : "No")
                << std::endl;
      std::cout << "    Area Code: " << marking.area_cd << std::endl;
      std::cout << "    Points: " << marking.points.size() << std::endl;
    }

    if (m_MarkingLineStrings.size() > printCount) {
      std::cout << "  ... and " << (m_MarkingLineStrings.size() - printCount)
                << " more marking line strings" << std::endl;
    }
  }

  if (!m_TrafficSigns.empty()) {
    std::cout << "\nTraffic Sign Details:" << std::endl;
    for (size_t i = 0; i < std::min(m_TrafficSigns.size(), printCount); ++i) {
      const auto &trafficSign = m_TrafficSigns[i];
      std::cout << "  Traffic Sign " << i + 1 << ":" << std::endl;
      std::cout << "    Object ID: " << trafficSign.obj_id << std::endl;
      std::cout << "    Link IDs: " << trafficSign.link_ids << std::endl;
      std::cout << "    Lane IDs: " << trafficSign.lane_ids << std::endl;
      std::cout << "    Type: " << trafficSign.type;

      // 添加类型说明
      switch (trafficSign.type) {
      case 81:
        std::cout << " (交通标牌)";
        break;
      case 90:
        std::cout << " (其它)";
        break;
      default:
        std::cout << " (未知类型)";
        break;
      }
      std::cout << std::endl;

      std::cout << "    National Code: " << trafficSign.code << std::endl;
      std::cout << "    Value: "
                << (!trafficSign.value.empty() ? trafficSign.value : "N/A")
                << std::endl;
      std::cout << "    Name: "
                << (!trafficSign.name.empty() ? trafficSign.name : "N/A")
                << std::endl;
      std::cout << "    Junction IDs: "
                << (!trafficSign.rjuc_ids.empty() ? trafficSign.rjuc_ids
                                                  : "N/A")
                << std::endl;
      std::cout << "    Area Code: " << trafficSign.area_cd << std::endl;
      std::cout << "    Position: (" << trafficSign.pos.x() << ", "
                << trafficSign.pos.y() << ", " << trafficSign.pos.z() << ")"
                << std::endl;
    }

    if (m_TrafficSigns.size() > printCount) {
      std::cout << "  ... and " << (m_TrafficSigns.size() - printCount)
                << " more traffic signs" << std::endl;
    }
  }
}

///////////////////////////////////////////////////////

void HdMap::LoadLane() {
  fs::path lane_file = m_HdMapFolder / "hd_lane.json.geojson";

  if (!fs::exists(lane_file)) {
    std::cout << "Lane file not found: " << lane_file << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(lane_file);
  if (!file.is_open()) {
    std::cout << "Failed to open lane file: " << lane_file << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse lane GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_Lanes.clear();
  m_Lanes.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyLane lane;

    // 初始化默认值
    lane.mesh_id = 0;
    lane.length = 0.0;
    lane.direction = 1; // 默认顺方向
    lane.lane_seq = 0;
    lane.width = 0.0;
    lane.turn_info = 1; // 默认NONE_TURN
    lane.vt_type = 0;   // 默认路口外
    lane.spd_max = 0;
    lane.spd_min = 0;
    lane.lanesec_id = 0;
    lane.spd_pre = 0;
    lane.is_junc = 0; // 默认非路口内车道
    lane.lane_num = 0;
    lane.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("lane_id") && props["lane_id"].IsString()) {
      lane.lane_id = props["lane_id"].GetString();
    }

    if (props.HasMember("link_id") && props["link_id"].IsString()) {
      lane.link_id = props["link_id"].GetString();
    }

    if (props.HasMember("snode_id") && props["snode_id"].IsString()) {
      lane.snode_id = props["snode_id"].GetString();
    }

    if (props.HasMember("enode_id") && props["enode_id"].IsString()) {
      lane.enode_id = props["enode_id"].GetString();
    }

    if (props.HasMember("lane_type") && props["lane_type"].IsString()) {
      lane.lane_type = props["lane_type"].GetString();
    }

    if (props.HasMember("length") && props["length"].IsNumber()) {
      lane.length = props["length"].GetDouble();
    }

    if (props.HasMember("width") && props["width"].IsNumber()) {
      lane.width = props["width"].GetDouble();
    }

    if (props.HasMember("direction") && props["direction"].IsInt64()) {
      lane.direction = props["direction"].GetInt64();
    }

    if (props.HasMember("lane_seq") && props["lane_seq"].IsInt64()) {
      lane.lane_seq = props["lane_seq"].GetInt64();
    }

    if (props.HasMember("lmkg_id") && props["lmkg_id"].IsString()) {
      lane.lmkg_id = props["lmkg_id"].GetString();
    }

    if (props.HasMember("rmkg_id") && props["rmkg_id"].IsString()) {
      lane.rmkg_id = props["rmkg_id"].GetString();
    }

    if (props.HasMember("turn_info") && props["turn_info"].IsInt64()) {
      lane.turn_info = props["turn_info"].GetInt64();
    }

    if (props.HasMember("vt_type") && props["vt_type"].IsInt64()) {
      lane.vt_type = props["vt_type"].GetInt64();
    }

    if (props.HasMember("pre_lanes") && props["pre_lanes"].IsString()) {
      lane.pre_lanes = props["pre_lanes"].GetString();
    }

    if (props.HasMember("suc_lanes") && props["suc_lanes"].IsString()) {
      lane.suc_lanes = props["suc_lanes"].GetString();
    }

    if (props.HasMember("spd_max") && props["spd_max"].IsInt64()) {
      lane.spd_max = props["spd_max"].GetInt64();
    }

    if (props.HasMember("spd_min") && props["spd_min"].IsInt64()) {
      lane.spd_min = props["spd_min"].GetInt64();
    }

    if (props.HasMember("name") && props["name"].IsString()) {
      lane.name = props["name"].GetString();
    }

    if (props.HasMember("sec_name") && props["sec_name"].IsString()) {
      lane.sec_name = props["sec_name"].GetString();
    }

    if (props.HasMember("heading") && props["heading"].IsString()) {
      lane.heading = props["heading"].GetString();
    }

    if (props.HasMember("is_junc") && props["is_junc"].IsInt()) {
      lane.is_junc = props["is_junc"].GetInt();
    }

    if (props.HasMember("lane_num") && props["lane_num"].IsInt()) {
      lane.lane_num = props["lane_num"].GetInt();
    }

    // 解析几何信息
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiLineString" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiLineString 包含多个 LineString
        for (const auto &linestring : coordinates) {
          if (linestring.IsArray()) {
            const auto &points_array = linestring.GetArray();

            // 解析每个点的坐标
            for (const auto &point : points_array) {
              if (point.IsArray() && point.Size() >= 3) {
                const auto &coord = point.GetArray();
                float x = static_cast<float>(coord[0].GetDouble());
                float y = static_cast<float>(coord[1].GetDouble());
                float z = static_cast<float>(coord[2].GetDouble());

                lane.points.emplace_back(x - m_localOrigin.x(),
                                         y - m_localOrigin.y(),
                                         z - m_localOrigin.z());
              }
            }
          }
        }
      }
    }

    m_Lanes.push_back(std::move(lane));
  }
}

void HdMap::LoadRoadSide() {
  fs::path road_side_file = m_HdMapFolder / "hd_road_side.json.geojson";

  if (!fs::exists(road_side_file)) {
    std::cout << "Road side file not found: " << road_side_file << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(road_side_file);
  if (!file.is_open()) {
    std::cout << "Failed to open road side file: " << road_side_file
              << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse road side GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_RoadSides.clear();
  m_RoadSides.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyRoadSide roadSide;

    // 初始化默认值
    roadSide.type = 80;          // 默认其它类型
    roadSide.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("obj_id") && props["obj_id"].IsString()) {
      roadSide.obj_id = props["obj_id"].GetString();
    }

    if (props.HasMember("link_ids") && props["link_ids"].IsString()) {
      roadSide.link_ids = props["link_ids"].GetString();
    }

    if (props.HasMember("lane_ids") && props["lane_ids"].IsString()) {
      roadSide.lane_ids = props["lane_ids"].GetString();
    }

    if (props.HasMember("Rjuc_ids")) {
      if (props["Rjuc_ids"].IsString()) {
        roadSide.rjuc_ids = props["Rjuc_ids"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    if (props.HasMember("area_cd")) {
      if (props["area_cd"].IsInt64()) {
        roadSide.area_cd = std::to_string(props["area_cd"].GetInt64());
      } else if (props["area_cd"].IsString()) {
        roadSide.area_cd = props["area_cd"].GetString();
      }
    }

    if (props.HasMember("type")) {
      if (props["type"].IsInt64()) {
        roadSide.type = props["type"].GetInt64();
      } else if (props["type"].IsDouble()) {
        roadSide.type = static_cast<int64_t>(props["type"].GetDouble());
      }
    }

    // 解析几何信息
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiLineString" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiLineString 包含多个 LineString
        for (const auto &linestring : coordinates) {
          if (linestring.IsArray()) {
            const auto &points_array = linestring.GetArray();

            // 解析每个点的坐标
            for (const auto &point : points_array) {
              if (point.IsArray() && point.Size() >= 3) {
                const auto &coord = point.GetArray();
                float x = static_cast<float>(coord[0].GetDouble());
                float y = static_cast<float>(coord[1].GetDouble());
                float z = static_cast<float>(coord[2].GetDouble());

                roadSide.points.emplace_back(x - m_localOrigin.x(),
                                             y - m_localOrigin.y(),
                                             z - m_localOrigin.z());
              }
            }
          }
        }
      }
    }

    m_RoadSides.push_back(std::move(roadSide));
  }

  std::cout << "Loaded " << m_RoadSides.size() << " road side features"
            << std::endl;
}

void HdMap::LoadArrow() {
  fs::path arrow_file = m_HdMapFolder / "hd_arrow.json.geojson";

  if (!fs::exists(arrow_file)) {
    std::cout << "Arrow file not found: " << arrow_file << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(arrow_file);
  if (!file.is_open()) {
    std::cout << "Failed to open arrow file: " << arrow_file << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse arrow GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_Arrows.clear();
  m_Arrows.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyArrow arrow;

    // 初始化默认值
    arrow.color = 1;          // 默认白色
    arrow.type = 150;         // 默认其它
    arrow.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("obj_id") && props["obj_id"].IsString()) {
      arrow.obj_id = props["obj_id"].GetString();
    }

    if (props.HasMember("link_ids") && props["link_ids"].IsString()) {
      arrow.link_ids = props["link_ids"].GetString();
    }

    if (props.HasMember("lane_ids") && props["lane_ids"].IsString()) {
      arrow.lane_ids = props["lane_ids"].GetString();
    }

    if (props.HasMember("Rjuc_ids")) {
      if (props["Rjuc_ids"].IsString()) {
        arrow.rjuc_ids = props["Rjuc_ids"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    if (props.HasMember("area_cd")) {
      if (props["area_cd"].IsInt64()) {
        arrow.area_cd = std::to_string(props["area_cd"].GetInt64());
      } else if (props["area_cd"].IsString()) {
        arrow.area_cd = props["area_cd"].GetString();
      }
    }

    if (props.HasMember("color") && props["color"].IsInt64()) {
      arrow.color = props["color"].GetInt64();
    }

    if (props.HasMember("type") && props["type"].IsInt64()) {
      arrow.type = props["type"].GetInt64();
    }

    // 解析几何信息 (MultiPolygon)
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiPolygon" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiPolygon 包含多个 Polygon
        for (const auto &polygon : coordinates) {
          if (polygon.IsArray()) {
            const auto &rings = polygon.GetArray();

            // 每个 Polygon 包含多个 LinearRing（外环 + 内环）
            for (const auto &ring : rings) {
              if (ring.IsArray()) {
                const auto &points_array = ring.GetArray();
                std::vector<Eigen::Vector3f> polygon_points;
                polygon_points.reserve(points_array.Size());

                // 解析每个点的坐标
                for (const auto &point : points_array) {
                  if (point.IsArray() && point.Size() >= 3) {
                    const auto &coord = point.GetArray();
                    float x = static_cast<float>(coord[0].GetDouble());
                    float y = static_cast<float>(coord[1].GetDouble());
                    float z = static_cast<float>(coord[2].GetDouble());

                    polygon_points.emplace_back(x - m_localOrigin.x(),
                                                y - m_localOrigin.y(),
                                                z - m_localOrigin.z());
                  }
                }

                if (!polygon_points.empty()) {
                  arrow.polygons.push_back(std::move(polygon_points));
                }
              }
            }
          }
        }
      }
    }

    m_Arrows.push_back(std::move(arrow));
  }

  std::cout << "Loaded " << m_Arrows.size() << " arrow features" << std::endl;
}

void HdMap::LoadPole() {
  fs::path pole_file = m_HdMapFolder / "hd_pole.json.geojson";

  if (!fs::exists(pole_file)) {
    std::cout << "Pole file not found: " << pole_file << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(pole_file);
  if (!file.is_open()) {
    std::cout << "Failed to open pole file: " << pole_file << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse pole GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_Poles.clear();
  m_Poles.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyPole pole;

    // 初始化默认值
    pole.type = 60;          // 默认其它类型
    pole.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("obj_id") && props["obj_id"].IsString()) {
      pole.obj_id = props["obj_id"].GetString();
    }

    if (props.HasMember("link_ids") && props["link_ids"].IsString()) {
      pole.link_ids = props["link_ids"].GetString();
    }

    if (props.HasMember("lane_ids") && props["lane_ids"].IsString()) {
      pole.lane_ids = props["lane_ids"].GetString();
    }

    if (props.HasMember("area_cd")) {
      if (props["area_cd"].IsString()) {
        pole.area_cd = props["area_cd"].GetString();
      } else if (props["area_cd"].IsInt64()) {
        pole.area_cd = std::to_string(props["area_cd"].GetInt64());
      }
    }

    if (props.HasMember("type")) {
      if (props["type"].IsInt64()) {
        pole.type = props["type"].GetInt64();
      } else if (props["type"].IsDouble()) {
        pole.type = static_cast<int64_t>(props["type"].GetDouble());
      }
    }

    // 解析几何信息 (MultiLineString)
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiLineString" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiLineString 包含多个 LineString
        for (const auto &linestring : coordinates) {
          if (linestring.IsArray()) {
            const auto &points_array = linestring.GetArray();

            // 解析每个点的坐标
            for (const auto &point : points_array) {
              if (point.IsArray() && point.Size() >= 3) {
                const auto &coord = point.GetArray();
                float x = static_cast<float>(coord[0].GetDouble());
                float y = static_cast<float>(coord[1].GetDouble());
                float z = static_cast<float>(coord[2].GetDouble());

                pole.points.emplace_back(x - m_localOrigin.x(),
                                         y - m_localOrigin.y(),
                                         z - m_localOrigin.z());
              }
            }
          }
        }
      }
    }

    m_Poles.push_back(std::move(pole));
  }

  std::cout << "Loaded " << m_Poles.size() << " pole features" << std::endl;
}

void HdMap::LoadRoadArea() {
  fs::path road_area_file = m_HdMapFolder / "hd_road_area.json.geojson";

  if (!fs::exists(road_area_file)) {
    std::cout << "Road area file not found: " << road_area_file << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(road_area_file);
  if (!file.is_open()) {
    std::cout << "Failed to open road area file: " << road_area_file
              << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse road area GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_RoadAreas.clear();
  m_RoadAreas.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyRoadArea roadArea;

    // 初始化默认值
    roadArea.type = 20;          // 默认其它类型
    roadArea.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("obj_id") && props["obj_id"].IsString()) {
      roadArea.obj_id = props["obj_id"].GetString();
    }

    if (props.HasMember("link_ids") && props["link_ids"].IsString()) {
      roadArea.link_ids = props["link_ids"].GetString();
    }

    if (props.HasMember("lane_ids") && props["lane_ids"].IsString()) {
      roadArea.lane_ids = props["lane_ids"].GetString();
    }

    if (props.HasMember("Rjuc_ids")) {
      if (props["Rjuc_ids"].IsString()) {
        roadArea.rjuc_ids = props["Rjuc_ids"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    if (props.HasMember("area_cd")) {
      if (props["area_cd"].IsInt64()) {
        roadArea.area_cd = std::to_string(props["area_cd"].GetInt64());
      } else if (props["area_cd"].IsString()) {
        roadArea.area_cd = props["area_cd"].GetString();
      }
    }

    if (props.HasMember("type") && props["type"].IsInt64()) {
      roadArea.type = props["type"].GetInt64();
    }

    // 解析几何信息 (MultiPolygon)
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiPolygon" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiPolygon 包含多个 Polygon
        for (const auto &polygon : coordinates) {
          if (polygon.IsArray()) {
            const auto &rings = polygon.GetArray();

            // 每个 Polygon 包含多个 LinearRing（外环 + 内环）
            for (const auto &ring : rings) {
              if (ring.IsArray()) {
                const auto &points_array = ring.GetArray();
                std::vector<Eigen::Vector3f> polygon_points;
                polygon_points.reserve(points_array.Size());

                // 解析每个点的坐标
                for (const auto &point : points_array) {
                  if (point.IsArray() && point.Size() >= 3) {
                    const auto &coord = point.GetArray();
                    float x = static_cast<float>(coord[0].GetDouble());
                    float y = static_cast<float>(coord[1].GetDouble());
                    float z = static_cast<float>(coord[2].GetDouble());

                    polygon_points.emplace_back(x - m_localOrigin.x(),
                                                y - m_localOrigin.y(),
                                                z - m_localOrigin.z());
                  }
                }

                if (!polygon_points.empty()) {
                  roadArea.polygons.push_back(std::move(polygon_points));
                }
              }
            }
          }
        }
      }
    }

    m_RoadAreas.push_back(std::move(roadArea));
  }

  std::cout << "Loaded " << m_RoadAreas.size() << " road area features"
            << std::endl;
}

void HdMap::LoadMarkingLineString() {
  fs::path marking_file = m_HdMapFolder / "hd_marking_line_string.json.geojson";

  if (!fs::exists(marking_file)) {
    std::cout << "Marking line string file not found: " << marking_file
              << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(marking_file);
  if (!file.is_open()) {
    std::cout << "Failed to open marking line string file: " << marking_file
              << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse marking line string GeoJSON file"
              << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_MarkingLineStrings.clear();
  m_MarkingLineStrings.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyMarkingLineString marking;

    // 初始化默认值
    marking.type = 99;          // 默认其它类型
    marking.color = 99;         // 默认其它颜色
    marking.material = 99;      // 默认其它材质
    marking.width = 0.0;        // 默认宽度
    marking.length = 0.0;       // 默认长度
    marking.is_junc = 0;        // 默认非路口
    marking.linesec_id = 0;     // 默认序列号
    marking.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("marking_id") && props["marking_id"].IsString()) {
      marking.marking_id = props["marking_id"].GetString();
    }

    if (props.HasMember("link_id") && props["link_id"].IsString()) {
      marking.link_id = props["link_id"].GetString();
    }

    if (props.HasMember("line_id") && props["line_id"].IsString()) {
      marking.line_id = props["line_id"].GetString();
    }

    if (props.HasMember("area_cd") && props["area_cd"].IsString()) {
      marking.area_cd = props["area_cd"].GetString();
    }

    if (props.HasMember("type") && props["type"].IsInt64()) {
      marking.type = props["type"].GetInt64();
    }

    if (props.HasMember("color") && props["color"].IsInt64()) {
      marking.color = props["color"].GetInt64();
    }

    if (props.HasMember("material") && props["material"].IsInt64()) {
      marking.material = props["material"].GetInt64();
    }

    if (props.HasMember("width") && props["width"].IsNumber()) {
      marking.width = props["width"].GetDouble();
    }

    if (props.HasMember("length") && props["length"].IsNumber()) {
      marking.length = props["length"].GetDouble();
    }

    if (props.HasMember("is_junc") && props["is_junc"].IsInt64()) {
      marking.is_junc = props["is_junc"].GetInt64();
    }

    if (props.HasMember("linesec_id") && props["linesec_id"].IsInt64()) {
      marking.linesec_id = props["linesec_id"].GetInt64();
    }

    // 解析几何信息 (MultiLineString)
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();

      if (geom_type == "MultiLineString" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();

        // MultiLineString 包含多个 LineString
        for (const auto &linestring : coordinates) {
          if (linestring.IsArray()) {
            const auto &points_array = linestring.GetArray();

            // 解析每个点的坐标
            for (const auto &point : points_array) {
              if (point.IsArray() && point.Size() >= 3) {
                const auto &coord = point.GetArray();
                float x = static_cast<float>(coord[0].GetDouble());
                float y = static_cast<float>(coord[1].GetDouble());
                float z = static_cast<float>(coord[2].GetDouble());

                marking.points.emplace_back(x - m_localOrigin.x(),
                                            y - m_localOrigin.y(),
                                            z - m_localOrigin.z());
              }
            }
          }
        }
      }
    }

    m_MarkingLineStrings.push_back(std::move(marking));
  }

  std::cout << "Loaded " << m_MarkingLineStrings.size()
            << " marking line string features" << std::endl;
}

void HdMap::LoadTrafficSigns() {
  fs::path traffic_sign_file = m_HdMapFolder / "hd_traffic_sign.json.geojson";

  if (!fs::exists(traffic_sign_file)) {
    std::cout << "Traffic sign file not found: " << traffic_sign_file
              << std::endl;
    return;
  }

  // 读取文件内容
  std::ifstream file(traffic_sign_file);
  if (!file.is_open()) {
    std::cout << "Failed to open traffic sign file: " << traffic_sign_file
              << std::endl;
    return;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  std::string json_content = buffer.str();

  // 解析 JSON
  rapidjson::Document document;
  if (document.Parse(json_content.c_str()).HasParseError()) {
    std::cout << "Failed to parse traffic sign GeoJSON file" << std::endl;
    return;
  }

  // 检查是否为 FeatureCollection
  if (!document.IsObject() || !document.HasMember("type") ||
      std::string(document["type"].GetString()) != "FeatureCollection") {
    std::cout << "Invalid GeoJSON format: not a FeatureCollection" << std::endl;
    return;
  }

  // 解析 features
  if (!document.HasMember("features") || !document["features"].IsArray()) {
    std::cout << "No features found in GeoJSON" << std::endl;
    return;
  }

  const auto &features = document["features"].GetArray();
  m_TrafficSigns.clear();
  m_TrafficSigns.reserve(features.Size());

  for (const auto &feature : features) {
    if (!feature.IsObject() || !feature.HasMember("properties") ||
        !feature.HasMember("geometry")) {
      continue;
    }

    PolyTrafficSign trafficSign;

    // 初始化默认值
    trafficSign.type = 90;          // 默认其它类型
    trafficSign.code = 0;           // 默认国标编码
    trafficSign.area_cd = "320500"; // 固定值

    // 解析 properties
    const auto &props = feature["properties"];

    if (props.HasMember("obj_id") && props["obj_id"].IsString()) {
      trafficSign.obj_id = props["obj_id"].GetString();
    }

    if (props.HasMember("link_ids") && props["link_ids"].IsString()) {
      trafficSign.link_ids = props["link_ids"].GetString();
    }

    if (props.HasMember("lane_ids") && props["lane_ids"].IsString()) {
      trafficSign.lane_ids = props["lane_ids"].GetString();
    }

    if (props.HasMember("Rjuc_ids")) {
      if (props["Rjuc_ids"].IsString()) {
        trafficSign.rjuc_ids = props["Rjuc_ids"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    if (props.HasMember("area_cd")) {
      if (props["area_cd"].IsInt64()) {
        trafficSign.area_cd = std::to_string(props["area_cd"].GetInt64());
      } else if (props["area_cd"].IsString()) {
        trafficSign.area_cd = props["area_cd"].GetString();
      }
    }

    if (props.HasMember("type") && props["type"].IsInt64()) {
      trafficSign.type = props["type"].GetInt64();
    }

    if (props.HasMember("code") && props["code"].IsInt64()) {
      trafficSign.code = props["code"].GetInt64();
    }

    if (props.HasMember("value")) {
      if (props["value"].IsString()) {
        trafficSign.value = props["value"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    if (props.HasMember("name")) {
      if (props["name"].IsString()) {
        trafficSign.name = props["name"].GetString();
      }
      // 如果是 null，保持默认空字符串
    }

    // 优先从geometry.coordinates读取第一个点作为标志位置
    bool found_geom_pos = false;
    const auto &geometry = feature["geometry"];
    if (geometry.HasMember("type") && geometry.HasMember("coordinates")) {
      std::string geom_type = geometry["type"].GetString();
      if (geom_type == "MultiPolygon" && geometry["coordinates"].IsArray()) {
        const auto &coordinates = geometry["coordinates"].GetArray();
        // MultiPolygon -> Polygon -> LinearRing -> Point
        for (const auto &polygon : coordinates) {
          if (polygon.IsArray()) {
            for (const auto &ring : polygon.GetArray()) {
              if (ring.IsArray()) {
                for (const auto &point : ring.GetArray()) {
                  if (point.IsArray() && point.Size() >= 3) {
                    float x = static_cast<float>(point[0].GetDouble());
                    float y = static_cast<float>(point[1].GetDouble());
                    float z = static_cast<float>(point[2].GetDouble());
                    trafficSign.pos = Eigen::Vector3f(x - m_localOrigin.x(),
                                                      y - m_localOrigin.y(),
                                                      z - m_localOrigin.z());
                    found_geom_pos = true;
                    break;
                  }
                }
              }
              if (found_geom_pos)
                break;
            }
          }
          if (found_geom_pos)
            break;
        }
      }
    }

    m_TrafficSigns.push_back(std::move(trafficSign));
  }

  std::cout << "Loaded " << m_TrafficSigns.size() << " traffic sign features"
            << std::endl;
}
} // namespace tydwm