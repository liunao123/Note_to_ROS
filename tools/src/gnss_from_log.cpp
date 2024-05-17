#include "gnss.hpp"
#include "zlog.h"
#include <sys/stat.h>
#include <ros/duration.h>


namespace GNSS
{
    gnssBase::gnssBase()
    {
        rot_cov = 99999.0;
        WGS84_A = 6378137.0;
        WGS84_E = 0.0818191908;
        UTM_E2 = (WGS84_E * WGS84_E);
        UTM_K0 = 0.9996;
        RADIANS_PER_DEGREE = M_PI / 180.0;

        count_init = 0;
        initLocal = true;
        freset = false;
        transUTM2LocalCenter = Eigen::Isometry3d::Identity();
        transUTM2Local = Eigen::Isometry3d::Identity();
        transLocal2UTM = Eigen::Isometry3d::Identity();

        centerInstallationRelative_x = -0.475;
        centerInstallationRelative_y = 0.16;
        UTMInitialization_x = 642597.0;
        UTMInitialization_y = 2937584.0;


        alignment = false;
        bIsLogPrintM = true;

        //放置进动态参数里
        portPath = "/home/xf/Desktop/gnss/gnss_2022-06-16.log";
        try
        {
            infile.open(portPath, ios::in);
            if (!infile.is_open())
            {
                cout << "读取文件失败" << endl;
                return;
            }
            cout << "读取文件ok" << endl;


        }
        catch (serial::IOException &e)
        {
            dzlog_error("###### Unable to open log file <%s>", portPath.c_str());
            return;
        }
    }

    gnssBase::~gnssBase() {}

    void gnssBase::get_lonlat_from_log()
    {
        string buf;
        string one_data;
        ros::Rate rate_loop(500);

        while (getline(infile, buf))
        {
            // 2.截取数据、解析数据：
            // GPS起始标志
            std::string gstart = "#BESTPOS";
            // GPS中间位置
            std::string gmiddle = "#HEADINGA";
            // GPS终止标志
            std::string gend = "\r\n";
            int middle = -1, start = -1, end = -1;
            //如果数据中找不到两次'\n'，则要一次数据，直到有两个以上的'\n'在解析

            // ROS_INFO("%s", buf.c_str());
            rate_loop.sleep();

            if (0)
            {
                break;
            }
            else
            {
                one_data += buf;
                auto pos_bestposa = one_data.find(gstart);
                auto pos_headinga = one_data.find(gmiddle);

                if( pos_bestposa == one_data.npos || pos_headinga == one_data.npos)
                {
                    continue;
                }
                else
                {
                    ROS_ERROR("%s", one_data.substr(pos_bestposa,end).c_str());
                    auto pos_FLOAT = one_data.find("NARROW_INT");
                    //http://t.zoukankan.com/21207-iHome-p-5237701.html 粒子滤波
                    if(pos_FLOAT != one_data.npos)
                    {
                        try
                        {
                            processData(one_data.substr(pos_bestposa,end));
                            one_data.clear();
                        }
                        catch(const std::exception& e)
                        {
                            std::cerr << e.what() << "-------- \n";
                            continue;
                        }
                    }
                    else
                    {
                        one_data.clear();
                        continue;
                    }

                }
            }
        

        }
    }

    bool gnssBase::setup(ros::NodeHandle &node)
    {
        GNSSllPublisher = node.advertise<sensor_msgs::NavSatFix>("GNSS_ll", 100);
        GNSSutmPublisher = node.advertise<nav_msgs::Odometry>("GNSS_utm", 100);
        GNSSlocalPublisher = node.advertise<nav_msgs::Odometry>("GNSS_local", 100);
        config();
        return true;
    }

    void gnssBase::readSpin()
    {
        ros::Rate rate(100);

        while (ros::ok())
        {
            if (freset)
            {
                fresetModel();
            }
            
            // cropData();
            get_lonlat_from_log();

            ros::spinOnce();
            rate.sleep();
        }
    }

    void gnssBase::cropData()
    {
        static short iCnt = 3;
        if (++iCnt >= 3)
        {
            iCnt = 0;
            bIsLogPrintM = true;
        }
        else
        {
            bIsLogPrintM = false;
        }

        if (ser.available())
        {
            // 1.读取串口信息：
            // dzlog_info("@@@@@@ cropData() Reading from serial port");
            strRece += ser.read(ser.available());
            // 2.截取数据、解析数据：
            // GPS起始标志
            std::string gstart = "#BESTPOS";
            // GPS中间位置
            std::string gmiddle = "#HEADINGA";
            // GPS终止标志
            std::string gend = "\r\n";
            int middle = -1, start = -1, end = -1;
            //如果数据中找不到两次'\n'，则要一次数据，直到有两个以上的'\n'在解析
            while (count(strRece.begin(), strRece.end(), '\n') > 1)
            {
                //找起始标志
                start = strRece.find(gstart);
                //如果没找到，丢弃这部分数据，但要留下最后7位,避免遗漏掉起始标志(根据起始标志-1设定)
                if (start == strRece.npos)
                {
                    if (strRece.length() > 7)
                        strRece = strRece.substr(strRece.length() - 7);
                    break;
                }
                else
                {
                    //如果找到了起始标志，开始找中间标志
                    //如果找到了起始位置，则保存从起始位置往后的全部数据，为了标准化数据头
                    strRece = strRece.substr(start);
                    start = 0;
                    middle = strRece.find(gmiddle);
                    if (middle == strRece.npos)
                    {
                        break;
                    }
                    else
                    {
                        //找终止标志
                        end = strRece.find(gend, middle);
                        //如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
                        if (end == strRece.npos)
                        {
                            break;
                        }
                        else
                        {
                            //如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
                            //把有效的数据给解析的函数以获取经纬度
                            processData(strRece.substr(0, end + 2));
                            if (bIsLogPrintM)
                            {
                                // std::cout << "Raw_data:  " << strRece.substr(0,end+2) << std::endl;
                                dzlog_info("@@@@@@ cropData() Raw_data = %s", strRece.substr(0, end + 2).c_str());
                            }
                            strRece = strRece.substr(end + 2);
                        }
                    }
                }
            }
        }
    }

    void gnssBase::processData(std::string s)
    {
        //分割有效数据，存入vector中
        std::vector<std::string> v;
        std::string::size_type pos1, pos2, pos3;
        pos1 = 0;
        pos2 = s.find(",");
        pos3 = 0;
        pos3 = s.find("\r\n", 0);

        while (pos2 != std::string::npos)
        {
            if (pos2 > pos3)
            {
                v.push_back(s.substr(pos1, pos3 - pos1));
                pos1 = pos3 + 2;
                pos3 = s.find("\r\n", pos1);
            }
            v.push_back(s.substr(pos1, pos2 - pos1));
            pos1 = pos2 + 1;
            pos2 = s.find(",", pos1);
        }
        if (pos1 < pos3)
            v.push_back(s.substr(pos1, pos3 - pos1));
        if (v.max_size() >= 22)
        {
            if (getRawLLData(v))
            {
                pubGnssLLData();
                pubGnssUTMData();
                if (bIsLogPrintM)
                {
                    dzlog_info("@@@@@@ processData() utmCenterHeadinginit: %f,UTMdata.UTME: %f,UTMdata.UTMN: %f,UTMdata.heading: %f,"
                               "ll heading: %f",
                               utmCenterHeadinginit, UTMdata.UTME, UTMdata.UTMN, UTMdata.heading, LLdata.heading);
                    dzlog_info("@@@@@@ processData() UTMInit vector[0]: %f,vector[1]: %f,vector[2]: %f",
                               centerUTMInit[0], centerUTMInit[1], centerUTMInit[2]);
                }
                calculateUTM2Local();
            }
        }
    }

    void gnssBase::initLLdata()
    {
        LLdata.lat = nan("ll");
        LLdata.lon = nan("ll");
        LLdata.alt = nan("ll");
        LLdata.heading = nan("ll");
        LLdata.satsNum = nan("ll");
        LLdata.positionType = "";
        LLdata.covariance = {{rot_cov, 0, 0,
                              0, 0, 0,
                              0, rot_cov, 0,
                              0, 0, 0,
                              0, 0, rot_cov,
                              0, 0, 0,
                              0, 0, 0, rot_cov, 0, 0,
                              0, 0, 0, 0, rot_cov, 0,
                              0, 0, 0, 0, 0, rot_cov}};

        LLdata.covariance3x3 = {{
            rot_cov,
            0,
            0,
            0,
            rot_cov,
            0,
            0,
            0,
            rot_cov,
        }};
    }

    //获取原始经纬度数据
    bool gnssBase::getRawLLData(std::vector<std::string> gnssRawData)
    {
        double latDev = nan("raw");
        double lonDev = nan("raw");
        double hgtDev = nan("raw");
        double headDev = nan("raw");

        initLLdata();

        //纬度
        if (gnssRawData[11] != "")
            LLdata.lat = std::atof(gnssRawData[11].c_str());
        //经度
        if (gnssRawData[12] != "")
            LLdata.lon = std::atof(gnssRawData[12].c_str());
        //海拔高
        if (gnssRawData[13] != "" || gnssRawData[14] != "")
            LLdata.alt = std::atof(gnssRawData[13].c_str()) +
                         std::atof(gnssRawData[14].c_str());
        //偏航角，与正北方向
        LLdata.heading = -std::atof(gnssRawData[42].c_str());
        //定位状态
        if (gnssRawData[10] != "")
            LLdata.positionType = gnssRawData[10].c_str();
        //可使用卫星数
        if (gnssRawData[23] != "")
            LLdata.satsNum = std::atof(gnssRawData[23].c_str());
        //方差
        if (gnssRawData[16] != "" || gnssRawData[17] != "" || gnssRawData[18] != "")
        {
            //位置方差
            latDev = pow(std::atof(gnssRawData[16].c_str()), 2);
            lonDev = pow(std::atof(gnssRawData[17].c_str()), 2);
            // hgtDev = pow(std::atof(gnssRawData[18].c_str()),2);
            hgtDev = rot_cov;
            headDev = pow(std::atof(gnssRawData[45].c_str()), 2) * 0.01;

            if (latDev > 0.0015 || latDev < 0.000001 || lonDev < 0.000001 || headDev < 0.000001)
                latDev = rot_cov;
            if (lonDev > 0.0015 || latDev < 0.000001 || lonDev < 0.000001 || headDev < 0.000001)
                lonDev = rot_cov;
            if (headDev > 0.05 || latDev < 0.000001 || lonDev < 0.000001 || headDev < 0.000001)
                headDev = rot_cov;

            LLdata.covariance = {{latDev, 0, 0, 0, 0, 0,
                                  0, lonDev, 0, 0, 0, 0,
                                  0, 0, hgtDev, 0, 0, 0,
                                  0, 0, 0, rot_cov, 0, 0,
                                  0, 0, 0, 0, rot_cov, 0,
                                  0, 0, 0, 0, 0, headDev}};

            LLdata.covariance3x3 = {{latDev, 0, 0,
                                     0, lonDev, 0,
                                     0, 0, hgtDev}};
        }

        if (gnssRawData[11] == "" || std::atof(gnssRawData[11].c_str()) < 0.00001 ||
            gnssRawData[12] == "" || std::atof(gnssRawData[12].c_str()) < 0.00001)
        {
            return false;
        }
        return true;
    }

    //发布经纬度数据
    void gnssBase::pubGnssLLData()
    {
        if (GNSSllPublisher)
        {
            // gps状态
            sensor_msgs::NavSatStatus gnssNavStatus;
            //定位状态
            //无定位
            if (LLdata.positionType == "NONE")
                gnssNavStatus.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

            //单点定位
            if (LLdata.positionType == "SINGLE" || LLdata.positionType == "INS_PSRSP")
                gnssNavStatus.status = sensor_msgs::NavSatStatus::STATUS_FIX;

            //伪距差分或SBAS定位
            if (LLdata.positionType.find("DIFF") != string::npos ||
                LLdata.positionType == "WAAS")
                gnssNavStatus.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;

            //固定解或者浮点解
            if (LLdata.positionType.find("INT") != string::npos ||
                LLdata.positionType.find("FLOA") != string::npos ||
                LLdata.positionType == "INS_RTKFIXED")
                gnssNavStatus.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

            //消息类型
            sensor_msgs::NavSatFix GNSSllData;
            //时间戳
            GNSSllData.header.stamp = ros::Time::now();
            //坐标系
            GNSSllData.header.frame_id = GNSSllDataHeaderFrame_id;
            //数值
            GNSSllData.latitude = LLdata.lat;
            GNSSllData.longitude = LLdata.lon;
            //设置海拔高为0
            //设置海拔高显示航向角
            GNSSllData.altitude = LLdata.heading /*LLdata.alt*/;
            GNSSllData.status = gnssNavStatus;

            for (int i = 0; i < 9; i++)
                GNSSllData.position_covariance = LLdata.covariance3x3;

            GNSSllPublisher.publish(GNSSllData);
        }

        //界面显示调试信息
        if (bIsLogPrintM)
        {
            dzlog_info("@@@@@@ pubGnssLLData() 维度: %f,经度: %f,海拔高: %f,航向角: %f,使用卫星数: %f,位置类型: %s",
                       LLdata.lat, LLdata.lon, LLdata.alt, LLdata.heading, LLdata.satsNum, LLdata.positionType.c_str());
            dzlog_info("@@@@@@ pubGnssLLData() 方差:");
            for (int i = 0; i < 36; i += 6)
            {
                dzlog_info("%f,%f,%f,%f,%f,%f", LLdata.covariance[i], LLdata.covariance[i + 1], LLdata.covariance[i + 2],
                           LLdata.covariance[i + 3], LLdata.covariance[i + 4], LLdata.covariance[i + 5]);
            }
        }
    }

    void gnssBase::LL2UTM()
    {
        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;
        double LongOrigin;
        double eccPrimeSquared;
        double N, T, C, A, M;

        // Make sure the longitude is between -180.00 .. 179.9
        double LongTemp = (LLdata.lon + 180) - int((LLdata.lon + 180) / 360) * 360 - 180;
        double LatRad = LLdata.lat * RADIANS_PER_DEGREE;
        double LongRad = LongTemp * RADIANS_PER_DEGREE;
        double LongOriginRad;
        int ZoneNumber = int((LongTemp + 180) / 6) + 1;

        // ZoneNumber = int((LongTemp + 180)/6) + 1;
        if (LLdata.lat >= 56.0 && LLdata.lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
            ZoneNumber = 32;

        // Special zones for Svalbard
        if (LLdata.lat >= 72.0 && LLdata.lat < 84.0)
        {
            if (LongTemp >= 0.0 && LongTemp < 9.0)
                ZoneNumber = 31;
            else if (LongTemp >= 9.0 && LongTemp < 21.0)
                ZoneNumber = 33;
            else if (LongTemp >= 21.0 && LongTemp < 33.0)
                ZoneNumber = 35;
            else if (LongTemp >= 33.0 && LongTemp < 42.0)
                ZoneNumber = 37;
        }

        LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
        LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

        eccPrimeSquared = (eccSquared) / (1 - eccSquared);
        N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
        T = tan(LatRad) * tan(LatRad);
        C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
        A = cos(LatRad) * (LongRad - LongOriginRad);

        M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad) + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad) - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

        UTMdata.UTME = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0);

        UTMdata.UTMN = (double)(k0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));
        if (LLdata.lat < 0)
            UTMdata.UTMN += 10000000.0; // 10000000 meter offset for southern hemisphere
    }

    void gnssBase::initUTMdata()
    {
        UTMdata.UTME = nan("utm");
        UTMdata.UTMN = nan("utm");
        UTMdata.heading = nan("utm");
        UTMdata.satsNum = nan("utm");
        UTMdata.covariance = {{rot_cov, 0, 0,
                               0, 0, 0,
                               0, rot_cov, 0,
                               0, 0, 0,
                               0, 0, rot_cov,
                               0, 0, 0,
                               0, 0, 0, rot_cov, 0, 0,
                               0, 0, 0, 0, rot_cov, 0,
                               0, 0, 0, 0, 0, rot_cov}};
        UTMdata.heading = LLdata.heading * RADIANS_PER_DEGREE;
        UTMdata.covariance = LLdata.covariance;
        UTMdata.satsNum = UTMdata.satsNum;
    }

    void gnssBase::pubGnssUTMData()
    {
        if (LLdata.positionType == "NONE")
        {
            dzlog_error("###### pubGnssUTMData() No fix !!!");
            return;
        }

        initUTMdata();
        LL2UTM();

        if (GNSSutmPublisher)
        {
            nav_msgs::Odometry GNSSutmData;
            GNSSutmData.header.stamp = ros::Time::now();
            GNSSutmData.header.frame_id = "utm";
            GNSSutmData.header.frame_id = GNSSutmDataHeaderFrame_id;
            GNSSutmData.pose.pose.position.x = UTMdata.UTME;
            GNSSutmData.pose.pose.position.y = UTMdata.UTMN;
            // GNSSutmData.pose.pose.position.x = UTMdata.UTME;
            // GNSSutmData.pose.pose.position.y = UTMdata.UTMN;
            GNSSutmData.pose.pose.position.z = 0;
            //需要海拔高把下面注释消掉
            // GNSSutmData.pose.pose.position.z = GPSdata.alt;
            GNSSutmData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UTMdata.heading);
            GNSSutmData.pose.covariance = UTMdata.covariance;
            //发布
            GNSSutmPublisher.publish(GNSSutmData);
        }
    }

    bool gnssBase::initLocalGNSS()
    {
        if (!initLocal)
        {
            // double localUTME, localUTMN, localHeading;
            // double localLatDev, localLonDev, localHeadDev;
            if (count_init < 15)
            {
                initLocaltemp.UTMEtemp += UTMdata.UTME;
                initLocaltemp.UTMNtemp += UTMdata.UTMN;
                initLocaltemp.headingtemp += UTMdata.heading;
                initLocaltemp.latDevtemp += UTMdata.covariance[0];
                initLocaltemp.lonDevtemp += UTMdata.covariance[7];
                initLocaltemp.headDevtemp += UTMdata.covariance[35];
                count_init++;
                dzlog_info("@@@@@@ initLocalGNSS() GNSS Local Initialization Processing!");
                return false;
            }
            else
            {
                //初始位置处定位信息
                initLocaltemp.UTMEtemp = initLocaltemp.UTMEtemp / count_init;
                initLocaltemp.UTMNtemp = initLocaltemp.UTMNtemp / count_init;
                initLocaltemp.headingtemp = initLocaltemp.headingtemp / count_init;
                initLocaltemp.latDevtemp = initLocaltemp.latDevtemp / sqrt(count_init);
                initLocaltemp.lonDevtemp = initLocaltemp.lonDevtemp / sqrt(count_init);
                initLocaltemp.headDevtemp = initLocaltemp.headDevtemp / sqrt(count_init);
                //计算变换全局到本地的变换矩阵(非对齐)
                Eigen::Isometry3d transUTM2Local = Eigen::Isometry3d::Identity();
                Eigen::AngleAxisd rotationUTM2Local(initLocaltemp.headingtemp, Eigen::Vector3d(0, 0, 1));
                transUTM2Local.rotate(rotationUTM2Local);
                transUTM2Local.pretranslate(Eigen::Vector3d(initLocaltemp.UTMEtemp, initLocaltemp.UTMNtemp, 0));
                centerUTMInit = transUTM2Local * Eigen::Vector3d(centerInstallationRelative_x, centerInstallationRelative_y, 0);

                //计算本地中心到全局的变换矩阵（对齐）
                Eigen::Isometry3d transUTM2LocalCenter = Eigen::Isometry3d::Identity();
                transUTM2LocalCenter.rotate(rotationUTM2Local);
                transUTM2LocalCenter.pretranslate(centerUTMInit);
                transLocal2UTM = transUTM2LocalCenter.inverse();
                utmCenterHeadinginit = initLocaltemp.headingtemp;

                dzlog_info("@@@@@@ initLocalGNSS() GNSS Local Initialization Success !!!");
                initLocal = true;
                return true;
            }
        }
        return true;
    }

    void gnssBase::calculateUTM2Local()
    {
        Eigen::Vector3d centerLocal;
        double headingLocal;
        if (alignment)
        {
            Eigen::AngleAxisd RA(UTMdata.heading, Eigen::Vector3d(0, 0, 1));
            Eigen::Isometry3d transUTM2Center = Eigen::Isometry3d::Identity();
            transUTM2Center.rotate(RA);
            transUTM2Center.pretranslate(Eigen::Vector3d(UTMdata.UTME, UTMdata.UTMN, 0));
            Eigen::Vector3d centerUTM = transUTM2Center * Eigen::Vector3d(centerInstallationRelative_x, centerInstallationRelative_y, 0);
            // ROS_INFO("570 %f, %f", centerUTM[0],centerUTM[1]);
            Eigen::Vector3d UTMInitialization = Eigen::Vector3d(642690, 2937600, 0);
            centerLocal = centerUTM - UTMInitialization;
            headingLocal = UTMdata.heading;
            // ROS_INFO("%f, %f", centerLocal[0],centerLocal[1]);
        }
        else
        {
            Eigen::AngleAxisd RA(UTMdata.heading, Eigen::Vector3d(0, 0, 1));
            Eigen::Isometry3d transUTM2Center = Eigen::Isometry3d::Identity();
            transUTM2Center.rotate(RA);
            transUTM2Center.pretranslate(Eigen::Vector3d(UTMdata.UTME, UTMdata.UTMN, 0));
            Eigen::Vector3d centerUTM = transUTM2Center * Eigen::Vector3d(centerInstallationRelative_x, centerInstallationRelative_y, 0);
            centerLocal = transLocal2UTM * centerUTM;
            headingLocal = UTMdata.heading - utmCenterHeadinginit;

        }
        dzlog_info("@@@@@@ calculateUTM2Local() headingLocal = %f", headingLocal);
        pubLocalCenter(centerLocal, headingLocal);
    }

    void gnssBase::pubLocalCenter(Eigen::Vector3d centerLocal, double headingLocal)
    {
        nav_msgs::Odometry centerLocalmsgs;
        centerLocalmsgs.header.stamp = ros::Time::now();
        centerLocalmsgs.header.frame_id = "odom_center";

        centerLocalmsgs.pose.pose.position.x = -centerLocal(0);
        centerLocalmsgs.pose.pose.position.y = -centerLocal(1);
        centerLocalmsgs.pose.pose.position.z = 0;

        centerLocalmsgs.pose.pose.orientation = tf::createQuaternionMsgFromYaw(headingLocal);
        centerLocalmsgs.pose.covariance = UTMdata.covariance;
        GNSSlocalPublisher.publish(centerLocalmsgs);
    }

    void gnssBase::configCallBack(cobra_gnss::cobra_gnss_Config &config, uint32_t level)
    {
        // centerInstall = config.CenterInstall;
        freset = config.freSet;
        // coordinateInitialization = config.coordinateInitialization;
        // transformSuccess = config.transformSuccess;
        UTMInitialization_x = config.UTMInitialization_x;
        UTMInitialization_y = config.UTMInitialization_y;
        // headUTMInitialization = config.headUTMInitialization;
        // posAccuracy = config.posAccuracy;
        centerInstallationRelative_x = -config.centerInstallationRelative_x;
        centerInstallationRelative_y = config.centerInstallationRelative_y;
        alignment = config.alignment;
        config.UTMInitialization_x = UTMInitialization_x;
        config.UTMInitialization_y = UTMInitialization_y;

        // ROS_INFO("Config Parameter Update!");
        // cout << std::setiosflags(std::ios::fixed) << std::setprecision(7) <<
        //      "旋转中心安装:               " << config.CenterInstall << endl <<
        //      "重启北斗模块:               " << config.freSet << endl <<
        //      "UTM坐标系本地对齐:           " << config.coordinateInitialization << endl <<
        //      "对齐坐标与航向（x，y，head）: " << "("<<config.UTMInitialization_x << ","
        //                                   << config.UTMInitialization_y <<","
        //                                   << endl;
        //                                   << config.headUTMInitialization <<")"<<endl<<
        //      "设定定位精度:               " << config.posAccuracy << endl <<
        //      "非中心安装位置要求（x，y）:   " << "("<<config.centerInstallationRelative_x << ","
        //                                   << config.centerInstallationRelative_y <<")" << endl;
        dzlog_info("@@@@@@ configCallBack() freSet = %d,UTMInit_x = %.3f,UTMInit_y = %.3f,cent_x = %f,cent_y = %f",
                   config.freSet, config.UTMInitialization_x, config.UTMInitialization_y, centerInstallationRelative_x,
                   centerInstallationRelative_y);
    }

    void gnssBase::config()
    {
        f = boost::bind(&gnssBase::configCallBack, this, _1, _2);
        server.setCallback(f);
    }

    void gnssBase::fresetModel()
    {
        ser.write("Freset\r\n");
        sleep(5);
        ser.write("log com2 gpgga ontime 0.1\r\n");
        ser.write("log bestposa ontime 0.1\r\n");
        ser.write("log headinga ontime 0.1\r\n");
        // ser.write("log versiona\r\n");
        ser.write("saveconfig\r\n");
        // ROS_INFO_STREAM("Write serial port\n");
        dzlog_info("@@@@@@ fresetModel() Write serial port");
        sleep(5);
        freset = false;
    }
}

int initZlog()
{
    if (-1 == access("/home/roslog", F_OK))
    {
        mkdir("/home/roslog", 0777);
    }
    if (dzlog_init("/home/config/zlog.conf", "gnss_cat") != 0)
    {
        printf("gnss init zlog failed\n");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    //初始化zlog
    // if (0 == initZlog())
    // {
    //     dzlog_info("@@@@@@ cobra_gnss init zlog success !!!");
    // }
    //初始化节点
    ros::init(argc, argv, "log_gnss_serial_node");
    //声明节点句柄
    ros::NodeHandle nh;

    GNSS::gnssBase GPSdata;


    if (GPSdata.setup(nh))
    {
        GPSdata.readSpin();
    }
    ros::spin();
}
