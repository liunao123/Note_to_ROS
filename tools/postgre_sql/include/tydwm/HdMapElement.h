#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

namespace tydwm {

// 1. 道路属性结构体（poly_link）：描述道路线要素属性
struct PolyLink {
  // link编号（String，长度25）
  std::string link_id;
  // 图幅编号（integer64，长度10，预留）
  int64_t mesh_id;
  // 起点编号（String，长度25）
  std::string snode_id;
  // 终点编号（String，长度25）
  std::string enode_id;
  // 路口内link标识（integer64，长度10；0=否，1=是）
  int64_t inner_flag;
  // link长度（Real，长度49，单位：米）
  double length;
  // 通行方向（integer64，长度10，预留，缺省1=顺方向）
  int64_t direction;
  // 起点到终点车道数（integer64，长度10，预留）
  int64_t lanes_fwd;
  // 终点到起点车道数（integer64，长度10，预留）
  int64_t lanes_rvs;
  // 路面材质（integer64，长度10；1=沥青，2=水泥混凝土...99=其他）
  int64_t material;
  // 道路等级（integer64，长度10；0=高速，1=城市高速...99=其他）
  int64_t road_fc;
  // 道路类型（integer64，长度10；1=匝道，2=桥梁...99=其他）
  int64_t road_type;
  // 主辅路标识（integer64，长度10；1=主路，2=辅路...99=其他）
  int64_t relief_flg;
  // 前继道路编号组（String，长度254）
  std::string pre_links;
  // 后续道路编号组（String，长度254）
  std::string suc_links;
  // 路口id（String，长度25）
  std::string inters_id;
  // 道路转向信息（Integer64，长度10，预留）
  int64_t turn_info;
  // 道路名称（String，长度200，苏州项目必填，NULL=无名/路口内道路）
  std::string name;
  // 路段名称（String，长度200，苏州项目必填，NULL=无路名/路口内道路）
  std::string sec_name;
  // 路段走向（String，长度200，苏州项目必填；1=东往西...4=北往南，NULL=路口内道路）
  std::string heading;
  // 最高限速（int，长度10，单位：km/h，应急车道=15，苏州项目必填）
  int spd_max;
  // 最低限速（int，长度10，单位：km/h，0=无最小限速，苏州项目必填）
  int spd_min;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
};

// 2. 道路连接点属性结构体（poly_link_node）：描述道路连接点（点要素）属性
struct PolyLinkNode {
  // 道路连接点编号（String，长度25）
  std::string node_id;
  // 图幅编号（Integer64，长度10）
  int64_t mesh_id;
  // 图幅连接点（Integer64，长度10）
  int64_t conn_flg;
  // 分歧点类型（Integer64，长度10；0=非分歧点，1=合流，2=分流，99=其他）
  int64_t bif_type;
  // 连接点类型（Integer64，长度10；1=路口节点...19=其它节点）
  int64_t type;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
};

// 3. 车道属性结构体（poly_lane）：描述车道线要素属性
struct PolyLane {
  // 车道编号（String，长度25）
  std::string lane_id;
  // 图幅编号（Integer64，长度10，预留）
  int64_t mesh_id;
  // 关联link编号（String，长度25）
  std::string link_id;
  // 起点编号（String，长度25）
  std::string snode_id;
  // 终点编号（String，长度25）
  std::string enode_id;
  // 车道长度（Real，长度49，单位：米）
  double length;
  // 通行方向（Integer64，长度10，预留，缺省1=顺方向）
  int64_t direction;
  // 车道序号（Integer64，长度10，应急车道参与排列）
  int64_t lane_seq;
  // 车道类型（String，长度25；从右向左每一位标识类型，1=常规车道...99=其他）
  std::string lane_type;
  // 车道宽度（Real，长度49，平均值）
  double width;
  // 左标线编号（String，长度25）
  std::string lmkg_id;
  // 右标线编号（String，长度25）
  std::string rmkg_id;
  // 转向信息（Integer64，长度10；1=NONE_TURN...16=全转向，99=其他）
  int64_t turn_info;
  // 车道所处位置类型（Integer64，长度10；0=路口外，1=路口内，99=其他）
  int64_t vt_type;
  // 前继车道编号组（String，长度254）
  std::string pre_lanes;
  // 后续车道编号组（String，长度254）
  std::string suc_lanes;
  // 车道最大限速（Integer64，长度10，单位：km/h，应急车道=15，苏州项目必填）
  int64_t spd_max;
  // 车道最小限速（Integer64，长度10，单位：km/h，0=无，苏州项目必填）
  int64_t spd_min;
  // Lane顺序号（Integer64，长度10，同一link下前进方向lane顺序）
  int64_t lanesec_id;
  // 推荐速度（Integer64，长度10，预留）
  int64_t spd_pre;
  // 道路名称（String，长度200，NULL=无名/路口内道路）
  std::string name;
  // 路段名称（String，长度200，苏州项目必填，NULL=无路名/路口内道路）
  std::string sec_name;
  // 路段走向（String，长度200，苏州项目必填；1=东往西...4=北往南，NULL=路口内道路）
  std::string heading;
  // 是否路口内车道（int，长度10；1=是，0=否）
  int is_junc;
  // 车道数（int，长度10，所处Lanesection中车道数目）
  int lane_num;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 中心线点
  std::vector<Eigen::Vector3f> points;
};

// 4. 车道连接点属性结构体（poly_lane_node）：描述车道连接点（点要素）属性
struct PolyLaneNode {
  // 道路连接点编号（String，长度25）
  std::string node_id;
  // 图幅编号（Integer64，长度10）
  int64_t mesh_id;
  // 类型（Integer64，长度10；1=路口节点...99=其它节点）
  int64_t type;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
};

// 5. 车道标线属性结构体（poly_marking_line_string）：描述车道标线（线要素）属性
struct PolyMarkingLineString {
  // 车道标线编号（String，长度25）
  std::string marking_id;
  // 关联link编号（String，长度254）
  std::string link_id;
  // 标线类型（Integer64，长度10；0=虚拟...100=应急车道虚拟，99=其他）
  int64_t type;
  // 标线颜色（Integer64，长度10；1=白色...6=橘色，99=其他）
  int64_t color;
  // 标线材质（Integer64，长度10；1=油漆，2=凸起，3=油漆+凸起，99=其他）
  int64_t material;
  // 标线宽度（Real，长度49）
  double width;
  // 标线长度（Real，长度19）
  double length;
  // 标线是否位于路口（Integer64，长度10；0=否，1=是）
  int64_t is_junc;
  // 车道标线唯一编号（String，长度25）
  std::string line_id;
  // 标线序列号（Integer64，长度5）
  int64_t linesec_id;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 几何点坐标（线要素）
  std::vector<Eigen::Vector3f> points;
};

// 6. 路口面属性结构体（poly_junction）：描述路口面要素属性
struct PolyJunction {
  // 路口编号（String，长度25）
  std::string jc_id;
  // 路口名称（String，长度200，苏州项目必填）
  std::string jc_name;
  // 注：文档中"is_vtjc"字段无类型/长度，预留（此处暂不定义，需根据实际使用补充）
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 北向道路名称（String，长度50，苏州项目必填）
  std::string N_road;
  // 南向道路名称（String，长度50，苏州项目必填）
  std::string S_road;
  // 东向道路名称（String，长度50，苏州项目必填）
  std::string E_road;
  // 西向道路名称（String，长度50，苏州项目必填）
  std::string W_road;
};

// 7. 道路面属性结构体（poly_road_surface）：描述道路面要素属性
struct PolyRoadSurface {
  // 道路面编号（String，长度25）
  std::string obj_id;
  // 注：文档中序号2字段缺失，此处跳过
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
};

// 8. 人行横道属性结构体（poly_crosswalk）：描述人行横道面要素属性
struct PolyCrosswalk {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer，长度10；1=人行横道，2=安全岛）
  int type;
};

// 9. 地面区域标志属性结构体（poly_road_area）：描述地面区域标志面要素属性
struct PolyRoadArea {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；3=导流带...12=收费站，20=其它）
  int64_t type;
  // 几何点坐标（多边形顶点）
  std::vector<std::vector<Eigen::Vector3f>> polygons;
};

// 10. 停止线属性结构体（poly_stopline）：描述停止线线要素属性
struct PolyStopline {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；21=停止线）
  int64_t type;
  // 颜色（Integer64，长度10；1=白...5=绿，99=其他）
  int64_t color;
  // 材质（Integer64，长度10；1=油漆，2=凸起，3=油漆+凸起，99=其他）
  int64_t material;
  // 宽度（Real，长度10）
  double width;
};

// 11. 横向标线属性结构体（poly_road_hhline）：描述横向标线线要素属性
struct PolyRoadHhLine {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；22=减速让行线...28=虚拟停止线，40=其它）
  int64_t type;
  // 颜色（Integer64，长度10；1=白...5=绿，99=其他）
  int64_t color;
  // 材质（Integer64，长度10；1=油漆，2=凸起，3=油漆+凸起，99=其他）
  int64_t material;
  // 宽度（Real，长度10）
  double width;
};

// 12. 杆状物属性结构体（poly_pole）：描述杆状物（外接矩形框要素）属性
struct PolyPole {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；41=路灯杆...47=横杆，60=其它）
  int64_t type;
  // 几何点坐标（线要素，表示杆状物的位置）
  std::vector<Eigen::Vector3f> points;
};

// 13. 防护设施属性结构体（poly_road_side）：描述防护设施线要素属性
struct PolyRoadSide {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；61=路缘石...69=施工隔离墙，80=其它）
  int64_t type;
  // 几何点坐标
  std::vector<Eigen::Vector3f> points;
};

// 14. 路牌属性结构体（poly_traffic_sign）：描述路牌面要素属性
struct PolyTrafficSign {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；81=交通标牌，90=其它）
  int64_t type;
  // 国标编码（Integer64，长度10）
  int64_t code;
  // 标牌内容（String，长度50；如"60"（限速）、"3t"（限重））
  std::string value;
  // 标牌类型（String，长度50）
  std::string name;
  // 牌子坐标（x/y/z，double类型，保留3位小数，苏州自定坐标）
  Eigen::Vector3f pos;
};

// 15. 交通灯属性结构体（poly_traffic_light）：描述交通灯面要素属性
struct PolyTrafficLight {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；91=圆形...97=机动车，109=其他）
  int64_t type;
  // 排列方式（Integer64，长度10；1=横，2=竖，3=点，99=其他）
  int64_t arrange;
  // 颜色变化（Integer64，长度10；1=红绿两段...5=黄色闪烁，99=其他）
  int64_t color_chg;
  // 控制车道编号（String，长度254）
  std::string ctl_lanes;
  // 关联停止线编号（String，长度254，预留）
  std::string r_stopline;
};

// 16. 地面箭头属性结构体（poly_arrow）：描述地面箭头面要素属性
struct PolyArrow {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 颜色（Integer64，长度10；1=白色...6=橘色）
  int64_t color;
  // 箭头方向（Integer64，长度10；110=直行...134=停车位箭头，150=其它）
  int64_t type;
  // 几何点坐标（多边形顶点）
  std::vector<std::vector<Eigen::Vector3f>> polygons;
};

// 17. 地面数字/文字/符号属性结构体（poly_road_mark）：描述地面标志面要素属性
struct PolyRoadMark {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 颜色（Integer64，长度10；1=白色...6=橘色）
  int64_t color;
  // 标识类型（Integer64，长度10；151=最高限速...166=停车让行，180=其他）
  int64_t type;
  // 字符（String，长度50，填写具体文字）
  std::string value;
};

// 18. 隧道/桥梁属性结构体（poly_tunnel_bridge）：描述隧道/桥梁面要素属性
struct PolyTunnelBridge {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类别（Integer，长度10；1=隧道，2=桥梁）
  int category;
  // 类型（Integer64，长度10；181=标准隧道...190=其它桥梁）
  int64_t type;
  // 名称（String，长度50）
  std::string name;
};

// 19. 停车位属性结构体（poly_parkingspace）：描述停车位面要素属性
struct PolyParkingspace {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10；191=侧方车位，192=垂直车位，193=斜向车位，200=其它）
  int64_t type;
};

// Additional missing structures for new geojson format compatibility
// Bumper structure (based on similar surface elements)
struct PolyBumper {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10）
  int64_t type;
  // 几何点坐标（多边形顶点）
  std::vector<std::vector<Eigen::Vector3f>> polygons;
};

// Clear zone structure (based on road area elements)
struct PolyClearZone {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10）
  int64_t type;
  // 几何点坐标（多边形顶点）
  std::vector<std::vector<Eigen::Vector3f>> polygons;
};

// Curb structure (based on line elements like road side)
struct PolyCurb {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10）
  int64_t type;
  // 几何点坐标
  std::vector<Eigen::Vector3f> points;
};

// Boundary structure (based on line elements like marking line string)
struct PolyBoundary {
  // 设施编号（String，长度25）
  std::string obj_id;
  // 关联link编号（String，长度254）
  std::string link_ids;
  // 关联车道编号（String，长度254）
  std::string lane_ids;
  // 关联路口编号（String，长度254）
  std::string rjuc_ids;
  // 行政区编码（String，长度6，固定值"320500"）
  std::string area_cd;
  // 类型（Integer64，长度10）
  int64_t type;
  // 几何点坐标
  std::vector<Eigen::Vector3f> points;
};

} // namespace tydwm