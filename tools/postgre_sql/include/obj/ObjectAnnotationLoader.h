#pragma once

#include "ObjectAnnotation.h"
#include "rapidjson/document.h"
#include <filesystem>
#include <memory>
#include <string>

namespace tydwm {
namespace obj {

namespace fs = std::filesystem;

class ObjectAnnotationLoader {
public:
    ObjectAnnotationLoader();
    ~ObjectAnnotationLoader() = default;

    // 加载bbox.json文件
    bool LoadFromFile(const std::string &filePath);
    bool LoadFromFile(const fs::path &filePath);

    // 获取加载的数据
    const SceneAnnotation &GetSceneAnnotation() const;

    // 获取特定类型的对象
    std::vector<AnnotatedObject> GetObjectsByType(ObjectType type) const;
    std::vector<AnnotatedObject>
        GetObjectsByAliasName(const std::string &aliasName) const;
    std::vector<AnnotatedObject>
        GetObjectsByLabelName(const std::string &labelName) const;

    // 获取特定track ID的对象
    std::vector<AnnotatedObject> GetObjectsByTrackId(int trackId) const;

    // 统计信息
    size_t                   GetObjectCount() const;
    size_t                   GetObjectCountByType(ObjectType type) const;
    std::vector<std::string> GetUniqueAliasNames() const;
    std::vector<std::string> GetUniqueLabelNames() const;

    // 实用功能
    bool HasObjects() const;
    void Clear();

    // 坐标变换支持
    void            SetLocalOrigin(const Eigen::Vector3f &origin);
    Eigen::Vector3f GetLocalOrigin() const;
    void            TransformToLocalCoordinates(bool enable);
    bool            IsLocalCoordinatesEnabled() const;

private:
    // 解析JSON的私有方法
    bool ParseJsonDocument(const std::string &content);
    bool ParseObjects(const rapidjson::Value &value);
    bool ParseObject(const rapidjson::Value &value, AnnotatedObject &obj);
    bool ParseGeometry(const rapidjson::Value &value, Geometry &geometry);
    bool ParsePosition(const rapidjson::Value &value, Position &position);
    bool ParseRotation(const rapidjson::Value &value, Rotation &rotation);
    bool ParseSize(const rapidjson::Value &value, Size &size);
    bool ParseProperties(
        const rapidjson::Value                       &value,
        std::unordered_map<std::string, std::string> &properties);
    bool ParseMappingObjects(const rapidjson::Value     &value,
                             std::vector<MappingObject> &mappingObjects);

    // 坐标变换工具
    Position TransformPosition(const Position &worldPos) const;

private:
    SceneAnnotation m_sceneAnnotation;

    // 坐标变换相关
    Eigen::Vector3f m_localOrigin;
    bool            m_useLocalCoordinates;

    // 文件路径缓存
    fs::path m_lastLoadedFile;
};

} // namespace obj
} // namespace tydwm