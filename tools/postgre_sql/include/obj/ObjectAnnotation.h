#pragma once

#include <eigen3/Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>

namespace tydwm {
namespace obj {

// 3D坐标结构体
struct Position {
    double x;
    double y;
    double z;

    Position()
        : x(0.0)
        , y(0.0)
        , z(0.0) {
    }
    Position(double x_, double y_, double z_)
        : x(x_)
        , y(y_)
        , z(z_) {
    }
};

// 3D旋转结构体 (欧拉角)
struct Rotation {
    double x;
    double y;
    double z;

    Rotation()
        : x(0.0)
        , y(0.0)
        , z(0.0) {
    }
    Rotation(double x_, double y_, double z_)
        : x(x_)
        , y(y_)
        , z(z_) {
    }
};

// 3D尺寸结构体
struct Size {
    double x;
    double y;
    double z;

    Size()
        : x(0.0)
        , y(0.0)
        , z(0.0) {
    }
    Size(double x_, double y_, double z_)
        : x(x_)
        , y(y_)
        , z(z_) {
    }
};

// 3D几何信息结构体
struct Geometry {
    Position position;
    Rotation rotation;
    Size     size;
};

// 映射对象结构体
struct MappingObject {
    std::string id;
    std::string type;
    // 可以根据需要扩展更多字段
};

// 目标对象结构体
struct AnnotatedObject {
    // 基本信息
    int         indexNumber;
    int         orderIndex; // nullable
    std::string toolType;
    std::string color;
    bool        keyFrame;
    int         splitFrame; // nullable
    std::string shapeId;
    int         trackId;
    std::string uuid;
    std::string userId;
    std::string labelName; // 中文标签名，如"小汽车"
    std::string aliasName; // 英文别名，如"car"
    bool        isDynamic; // 是否为动态物体

    // 几何信息
    Geometry                     geometry;
    Eigen::Matrix4f              bboxPose;
    std::vector<Eigen::Vector3f> bboxVertices;

    // 属性信息
    std::unordered_map<std::string, std::string> properties;

    // 映射对象
    std::vector<MappingObject> mappingObjects;

    AnnotatedObject()
        : indexNumber(0)
        , orderIndex(-1)
        , keyFrame(false)
        , splitFrame(-1)
        , trackId(0)
        , isDynamic(false) {
    }

    // 获取3D包围框的8个顶点（世界坐标系）
    // 顶点顺序: 底面4个顶点(逆时针) + 顶面4个顶点(逆时针)
    // 0: 后左下, 1: 前左下, 2: 前右下, 3: 后右下
    // 4: 后左上, 5: 前左上, 6: 前右上, 7: 后右上
    std::vector<Eigen::Vector3f> getBbox3DVertices() const {
        std::vector<Eigen::Vector3f> vertices(8);

        // 获取局部包围框的半尺寸
        float halfX = static_cast<float>(geometry.size.x * 0.5);
        float halfY = static_cast<float>(geometry.size.y * 0.5);
        float halfZ = static_cast<float>(geometry.size.z * 0.5);

        // 定义局部坐标系下的8个顶点
        Eigen::Vector3f localVertices[8] = {
            Eigen::Vector3f(-halfX, -halfY, -halfZ), // 0: 后左下
            Eigen::Vector3f(halfX, -halfY, -halfZ),  // 1: 前左下
            Eigen::Vector3f(halfX, halfY, -halfZ),   // 2: 前右下
            Eigen::Vector3f(-halfX, halfY, -halfZ),  // 3: 后右下
            Eigen::Vector3f(-halfX, -halfY, halfZ),  // 4: 后左上
            Eigen::Vector3f(halfX, -halfY, halfZ),   // 5: 前左上
            Eigen::Vector3f(halfX, halfY, halfZ),    // 6: 前右上
            Eigen::Vector3f(-halfX, halfY, halfZ)    // 7: 后右上
        };

        // 获取世界坐标系变换矩阵
        Eigen::Matrix4f transform = getWorldTransform();

        // 将局部顶点变换到世界坐标系
        for (int i = 0; i < 8; ++i) {
            Eigen::Vector4f homogeneous(localVertices[i].x(),
                                        localVertices[i].y(),
                                        localVertices[i].z(), 1.0f);
            Eigen::Vector4f worldHomogeneous = transform * homogeneous;
            vertices[i] =
                Eigen::Vector3f(worldHomogeneous.x(), worldHomogeneous.y(),
                                worldHomogeneous.z());
        }

        return vertices;
    }

    // 获取对象在世界坐标系中的位姿矩阵（4x4齐次变换矩阵）
    Eigen::Matrix4f getWorldTransform() const {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // 获取位置和旋转
        float tx = static_cast<float>(geometry.position.x);
        float ty = static_cast<float>(geometry.position.y);
        float tz = static_cast<float>(geometry.position.z);
        float rx = static_cast<float>(geometry.rotation.x);
        float ry = static_cast<float>(geometry.rotation.y);
        float rz = static_cast<float>(geometry.rotation.z);

        // 创建旋转矩阵（按ZYX顺序，即Roll-Pitch-Yaw）
        Eigen::AngleAxisf rollAngle(rx, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(ry, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(rz, Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f rotationMatrix =
            (yawAngle * pitchAngle * rollAngle).matrix();

        // 构建4x4齐次变换矩阵
        transform.block<3, 3>(0, 0) = rotationMatrix; // 旋转部分
        transform.block<3, 1>(0, 3) = Eigen::Vector3f(tx, ty, tz); // 平移部分
        // 最后一行保持为 [0, 0, 0, 1]

        return transform;
    }
};

// 场景标注数据结构体
struct SceneAnnotation {
    // todo :add other properties if needed
    std::vector<AnnotatedObject> objects;
};

// 目标类型枚举
enum class ObjectType { CAR, VAN, BUS, BICYCLE, TRICYCLE, ROBO, CONE, UNKNOWN };

// 对象类型转换工具
class ObjectTypeHelper {
public:
    static ObjectType  StringToObjectType(const std::string &aliasName);
    static std::string ObjectTypeToString(ObjectType type);
    static std::string GetObjectTypeColor(ObjectType type);
};

} // namespace obj
} // namespace tydwm