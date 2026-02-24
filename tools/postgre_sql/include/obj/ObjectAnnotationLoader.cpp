#include "obj/ObjectAnnotationLoader.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>

namespace tydwm {
namespace obj {

ObjectAnnotationLoader::ObjectAnnotationLoader()
    : m_localOrigin(Eigen::Vector3f::Zero())
    , m_useLocalCoordinates(false) {
}

bool ObjectAnnotationLoader::LoadFromFile(const std::string &filePath) {
    return LoadFromFile(fs::path(filePath));
}

bool ObjectAnnotationLoader::LoadFromFile(const fs::path &filePath) {
    if (!fs::exists(filePath)) {
        std::cerr << "Error: Bbox file does not exist: " << filePath
                  << std::endl;
        return false;
    }

    if (!fs::is_regular_file(filePath)) {
        std::cerr << "Error: Path is not a regular file: " << filePath
                  << std::endl;
        return false;
    }

    // 清除之前的数据
    Clear();

    // 读取文件内容
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open bbox file: " << filePath
                  << std::endl;
        return false;
    }

    // 读取整个文件到字符串
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    if (content.empty()) {
        std::cerr << "Error: Bbox file is empty: " << filePath << std::endl;
        return false;
    }

    // 解析JSON内容
    if (!ParseJsonDocument(content)) {
        std::cerr << "Error: Failed to parse bbox JSON file: " << filePath
                  << std::endl;
        return false;
    }

    m_lastLoadedFile = filePath;
    std::cout << "Successfully loaded " << GetObjectCount() << " objects from "
              << filePath.filename() << std::endl;

    return true;
}

bool ObjectAnnotationLoader::ParseJsonDocument(const std::string &content) {
    rapidjson::Document    doc;
    rapidjson::ParseResult result = doc.Parse(content.c_str());

    if (!result) {
        std::cerr << "JSON parse error: "
                  << rapidjson::GetParseError_En(result.Code())
                  << " (offset: " << result.Offset() << ")" << std::endl;
        return false;
    }

    if (!doc.IsObject()) {
        std::cerr << "Error: Root JSON element is not an object" << std::endl;
        return false;
    }

    // 解析各个部分
    bool success = true;

    if (doc.HasMember("objects") && doc["objects"].IsArray()) {
        const auto &objects = doc["objects"];
        success &= ParseObjects(objects);
    } else {
        std::cerr << "Warning: No 'objects' array found in JSON" << std::endl;
    }

    return success;
}

bool ObjectAnnotationLoader::ParseObjects(const rapidjson::Value &value) {
    if (!value.IsArray()) {
        return false;
    }

    for (const auto &objVal : value.GetArray()) {
        if (!objVal.IsObject()) {
            continue;
        }

        AnnotatedObject obj;
        if (ParseObject(objVal, obj)) {
            m_sceneAnnotation.objects.push_back(obj);
        }
    }

    return true;
}

bool ObjectAnnotationLoader::ParseObject(const rapidjson::Value &value,
                                         AnnotatedObject        &obj) {
    if (!value.IsObject()) {
        return false;
    }

    // 解析基本信息
    if (value.HasMember("indexNumber") && value["indexNumber"].IsInt()) {
        obj.indexNumber = value["indexNumber"].GetInt();
    }

    if (value.HasMember("orderIndex")) {
        if (value["orderIndex"].IsNull()) {
            obj.orderIndex = -1;
        } else if (value["orderIndex"].IsInt()) {
            obj.orderIndex = value["orderIndex"].GetInt();
        }
    }

    if (value.HasMember("toolType") && value["toolType"].IsString()) {
        obj.toolType = value["toolType"].GetString();
    }

    if (value.HasMember("color") && value["color"].IsString()) {
        obj.color = value["color"].GetString();
    }

    if (value.HasMember("keyFrame") && value["keyFrame"].IsBool()) {
        obj.keyFrame = value["keyFrame"].GetBool();
    }

    if (value.HasMember("splitFrame")) {
        if (value["splitFrame"].IsNull()) {
            obj.splitFrame = -1;
        } else if (value["splitFrame"].IsInt()) {
            obj.splitFrame = value["splitFrame"].GetInt();
        }
    }

    if (value.HasMember("shapeId") && value["shapeId"].IsString()) {
        obj.shapeId = value["shapeId"].GetString();
    }

    if (value.HasMember("trackId") && value["trackId"].IsInt()) {
        obj.trackId = value["trackId"].GetInt();
    }

    if (value.HasMember("uuid") && value["uuid"].IsString()) {
        obj.uuid = value["uuid"].GetString();
    }

    if (value.HasMember("userId") && value["userId"].IsString()) {
        obj.userId = value["userId"].GetString();
    }

    if (value.HasMember("labelName") && value["labelName"].IsString()) {
        obj.labelName = value["labelName"].GetString();
    }

    if (value.HasMember("aliasName") && value["aliasName"].IsString()) {
        obj.aliasName = value["aliasName"].GetString();
    }

    if (value.HasMember("isDynamic") && value["isDynamic"].IsBool()) {
        obj.isDynamic = value["isDynamic"].GetBool();
    }

    // 解析几何信息
    if (value.HasMember("geometry") && value["geometry"].IsObject()) {
        const auto &geometry = value["geometry"];
        ParseGeometry(geometry, obj.geometry);
        obj.bboxPose     = obj.getWorldTransform();
        obj.bboxVertices = obj.getBbox3DVertices();
    }

    // 解析属性
    if (value.HasMember("properties") && value["properties"].IsObject()) {
        const auto &properties = value["properties"];
        ParseProperties(properties, obj.properties);
    }

    // 解析映射对象
    if (value.HasMember("mappingObjects") &&
        value["mappingObjects"].IsArray()) {
        const auto &mappingObjects = value["mappingObjects"];
        ParseMappingObjects(mappingObjects, obj.mappingObjects);
    }

    return true;
}

bool ObjectAnnotationLoader::ParseGeometry(const rapidjson::Value &value,
                                           Geometry               &geometry) {
    if (!value.IsObject()) {
        return false;
    }

    if (value.HasMember("position") && value["position"].IsObject()) {
        const auto &position = value["position"];
        ParsePosition(position, geometry.position);
    }

    if (value.HasMember("rotation") && value["rotation"].IsObject()) {
        const auto &rotation = value["rotation"];
        ParseRotation(rotation, geometry.rotation);
    }

    if (value.HasMember("size") && value["size"].IsObject()) {
        const auto &size = value["size"];
        ParseSize(size, geometry.size);
    }

    return true;
}

bool ObjectAnnotationLoader::ParsePosition(const rapidjson::Value &value,
                                           Position               &position) {
    if (!value.IsObject()) {
        return false;
    }

    if (value.HasMember("x") && value["x"].IsNumber()) {
        position.x = value["x"].GetDouble();
    }

    if (value.HasMember("y") && value["y"].IsNumber()) {
        position.y = value["y"].GetDouble();
    }

    if (value.HasMember("z") && value["z"].IsNumber()) {
        position.z = value["z"].GetDouble();
    }

    // 如果启用了本地坐标转换，进行坐标变换
    if (m_useLocalCoordinates) {
        position = TransformPosition(position);
    }

    return true;
}

bool ObjectAnnotationLoader::ParseRotation(const rapidjson::Value &value,
                                           Rotation               &rotation) {
    if (!value.IsObject()) {
        return false;
    }

    if (value.HasMember("x") && value["x"].IsNumber()) {
        rotation.x = value["x"].GetDouble();
    }

    if (value.HasMember("y") && value["y"].IsNumber()) {
        rotation.y = value["y"].GetDouble();
    }

    if (value.HasMember("z") && value["z"].IsNumber()) {
        rotation.z = value["z"].GetDouble();
    }

    return true;
}

bool ObjectAnnotationLoader::ParseSize(const rapidjson::Value &value,
                                       Size                   &size) {
    if (!value.IsObject()) {
        return false;
    }

    if (value.HasMember("x") && value["x"].IsNumber()) {
        size.x = value["x"].GetDouble();
    }

    if (value.HasMember("y") && value["y"].IsNumber()) {
        size.y = value["y"].GetDouble();
    }

    if (value.HasMember("z") && value["z"].IsNumber()) {
        size.z = value["z"].GetDouble();
    }

    return true;
}

bool ObjectAnnotationLoader::ParseProperties(
    const rapidjson::Value                       &value,
    std::unordered_map<std::string, std::string> &properties) {
    if (!value.IsObject()) {
        return false;
    }

    for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
        if (it->value.IsString()) {
            properties[it->name.GetString()] = it->value.GetString();
        }
    }

    return true;
}

bool ObjectAnnotationLoader::ParseMappingObjects(
    const rapidjson::Value &value, std::vector<MappingObject> &mappingObjects) {
    if (!value.IsArray()) {
        return false;
    }

    for (const auto &mapObjVal : value.GetArray()) {
        if (!mapObjVal.IsObject()) {
            continue;
        }

        MappingObject mappingObj;

        if (mapObjVal.HasMember("id") && mapObjVal["id"].IsString()) {
            mappingObj.id = mapObjVal["id"].GetString();
        }

        if (mapObjVal.HasMember("type") && mapObjVal["type"].IsString()) {
            mappingObj.type = mapObjVal["type"].GetString();
        }

        mappingObjects.push_back(mappingObj);
    }

    return true;
}

Position
    ObjectAnnotationLoader::TransformPosition(const Position &worldPos) const {
    Position localPos;
    localPos.x = worldPos.x - static_cast<double>(m_localOrigin.x());
    localPos.y = worldPos.y - static_cast<double>(m_localOrigin.y());
    localPos.z = worldPos.z - static_cast<double>(m_localOrigin.z());
    return localPos;
}

// Getter方法实现
const SceneAnnotation &ObjectAnnotationLoader::GetSceneAnnotation() const {
    return m_sceneAnnotation;
}

std::vector<AnnotatedObject>
    ObjectAnnotationLoader::GetObjectsByType(ObjectType type) const {
    std::vector<AnnotatedObject> result;
    std::string targetAlias = ObjectTypeHelper::ObjectTypeToString(type);

    for (const auto &obj : m_sceneAnnotation.objects) {
        if (obj.aliasName == targetAlias) {
            result.push_back(obj);
        }
    }

    return result;
}

std::vector<AnnotatedObject> ObjectAnnotationLoader::GetObjectsByAliasName(
    const std::string &aliasName) const {
    std::vector<AnnotatedObject> result;

    for (const auto &obj : m_sceneAnnotation.objects) {
        if (obj.aliasName == aliasName) {
            result.push_back(obj);
        }
    }

    return result;
}

std::vector<AnnotatedObject> ObjectAnnotationLoader::GetObjectsByLabelName(
    const std::string &labelName) const {
    std::vector<AnnotatedObject> result;

    for (const auto &obj : m_sceneAnnotation.objects) {
        if (obj.labelName == labelName) {
            result.push_back(obj);
        }
    }

    return result;
}

std::vector<AnnotatedObject>
    ObjectAnnotationLoader::GetObjectsByTrackId(int trackId) const {
    std::vector<AnnotatedObject> result;

    for (const auto &obj : m_sceneAnnotation.objects) {
        if (obj.trackId == trackId) {
            result.push_back(obj);
        }
    }

    return result;
}

size_t ObjectAnnotationLoader::GetObjectCount() const {
    return m_sceneAnnotation.objects.size();
}

size_t ObjectAnnotationLoader::GetObjectCountByType(ObjectType type) const {
    return GetObjectsByType(type).size();
}

std::vector<std::string> ObjectAnnotationLoader::GetUniqueAliasNames() const {
    std::vector<std::string> result;
    std::vector<std::string> aliasNames;

    for (const auto &obj : m_sceneAnnotation.objects) {
        aliasNames.push_back(obj.aliasName);
    }

    std::sort(aliasNames.begin(), aliasNames.end());
    auto last = std::unique(aliasNames.begin(), aliasNames.end());
    aliasNames.erase(last, aliasNames.end());

    return aliasNames;
}

std::vector<std::string> ObjectAnnotationLoader::GetUniqueLabelNames() const {
    std::vector<std::string> result;
    std::vector<std::string> labelNames;

    for (const auto &obj : m_sceneAnnotation.objects) {
        labelNames.push_back(obj.labelName);
    }

    std::sort(labelNames.begin(), labelNames.end());
    auto last = std::unique(labelNames.begin(), labelNames.end());
    labelNames.erase(last, labelNames.end());

    return labelNames;
}

bool ObjectAnnotationLoader::HasObjects() const {
    return !m_sceneAnnotation.objects.empty();
}

void ObjectAnnotationLoader::Clear() {
    m_sceneAnnotation = SceneAnnotation();
    m_lastLoadedFile.clear();
}

void ObjectAnnotationLoader::SetLocalOrigin(const Eigen::Vector3f &origin) {
    m_localOrigin = origin;
}

Eigen::Vector3f ObjectAnnotationLoader::GetLocalOrigin() const {
    return m_localOrigin;
}

void ObjectAnnotationLoader::TransformToLocalCoordinates(bool enable) {
    m_useLocalCoordinates = enable;
}

bool ObjectAnnotationLoader::IsLocalCoordinatesEnabled() const {
    return m_useLocalCoordinates;
}

} // namespace obj
} // namespace tydwm