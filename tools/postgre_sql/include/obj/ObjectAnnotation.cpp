#include "obj/ObjectAnnotation.h"
#include <algorithm>
#include <unordered_map>

namespace tydwm {
namespace obj {

ObjectType ObjectTypeHelper::StringToObjectType(const std::string &aliasName) {
    static const std::unordered_map<std::string, ObjectType> typeMap = {
        {"car", ObjectType::CAR},           {"van", ObjectType::VAN},
        {"bus", ObjectType::BUS},           {"bicycle", ObjectType::BICYCLE},
        {"tricycle", ObjectType::TRICYCLE}, {"robo", ObjectType::ROBO},
        {"cone", ObjectType::CONE}};

    auto it = typeMap.find(aliasName);
    return (it != typeMap.end()) ? it->second : ObjectType::UNKNOWN;
}

std::string ObjectTypeHelper::ObjectTypeToString(ObjectType type) {
    static const std::unordered_map<ObjectType, std::string> stringMap = {
        {ObjectType::CAR, "car"},           {ObjectType::VAN, "van"},
        {ObjectType::BUS, "bus"},           {ObjectType::BICYCLE, "bicycle"},
        {ObjectType::TRICYCLE, "tricycle"}, {ObjectType::ROBO, "robo"},
        {ObjectType::CONE, "cone"},         {ObjectType::UNKNOWN, "unknown"}};

    auto it = stringMap.find(type);
    return (it != stringMap.end()) ? it->second : "unknown";
}

std::string ObjectTypeHelper::GetObjectTypeColor(ObjectType type) {
    static const std::unordered_map<ObjectType, std::string> colorMap = {
        {ObjectType::CAR, "#9166FF"},      // 紫色 - 小汽车
        {ObjectType::VAN, "#FF6B6B"},      // 红色 - 面包车
        {ObjectType::BUS, "#4ECDC4"},      // 青色 - 公交车
        {ObjectType::BICYCLE, "#45B7D1"},  // 蓝色 - 自行车
        {ObjectType::TRICYCLE, "#FFA07A"}, // 橙色 - 三轮车
        {ObjectType::ROBO, "#98D8C8"},     // 绿色 - 机器人
        {ObjectType::CONE, "#F7DC6F"},     // 黄色 - 锥桶
        {ObjectType::UNKNOWN, "#95A5A6"}   // 灰色 - 未知
    };

    auto it = colorMap.find(type);
    return (it != colorMap.end()) ? it->second : "#95A5A6";
}

} // namespace obj
} // namespace tydwm