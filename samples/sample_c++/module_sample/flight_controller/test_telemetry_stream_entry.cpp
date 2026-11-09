/**
 ********************************************************************
 * @file    test_telemetry_stream_entry.cpp
 * @brief   Continuous telemetry sample for velocity, GPS position, Euler angles, GPS time, RTK position
 *          and remote controller stick inputs.
 ********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <sys/select.h>

#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>

#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_fc_subscription.h"
#include "test_telemetry_stream_entry.h"

/* Private constants ---------------------------------------------------------*/
#define DJI_TELEMETRY_STREAM_SLEEP_MS (100)

/* Private values ------------------------------------------------------------*/
static volatile sig_atomic_t s_telemetryStopRequested = 0;
static std::ofstream s_telemetryCsvFile;
static std::string s_telemetryCsvFileName;
/* RTK topics are optional: if the aircraft does not provide them the logger
 * keeps running and the rtk_* CSV columns stay empty. */
static bool s_rtkPositionSubscribed = false;
static bool s_rtkPositionInfoSubscribed = false;
static bool s_rtkConnectStatusSubscribed = false;
static bool s_gpsDateSubscribed = false;   /* yyyymmdd from the aircraft GNSS; wall_clock can be wrong after a cold boot */
static bool s_rcSubscribed = false;        /* remote controller sticks (NN input); rc_* columns stay empty if unavailable */

/* Private functions declaration ---------------------------------------------*/
static void DjiUser_TelemetryStopSignalHandler(int signalNum);
static bool DjiUser_TelemetryExitRequested(void);
static T_DjiReturnCode DjiUser_EnsureDirectoryExists(const std::string &directoryPath);
static T_DjiVector3f DjiUser_QuaternionToEulerAngles(const T_DjiFcSubscriptionQuaternion &quaternion);
static std::string DjiUser_GetTelemetryCsvFileName(void);
static T_DjiReturnCode DjiUser_PrintTelemetryOnce(void);
static bool DjiUser_SubscribeOptional(E_DjiFcSubscriptionTopic topic, const char *topicName);

/* Exported functions definition ---------------------------------------------*/
void DjiUser_RunTelemetryStreamSample(void)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    s_telemetryStopRequested = 0;
    signal(SIGINT, DjiUser_TelemetryStopSignalHandler);
    signal(SIGTERM, DjiUser_TelemetryStopSignalHandler);

    s_telemetryCsvFileName = DjiUser_GetTelemetryCsvFileName();
    if (DjiUser_EnsureDirectoryExists("../../../Logs/telemetrydata") != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("failed to create telemetry csv directory.");
        return;
    }

    s_telemetryCsvFile.open(s_telemetryCsvFileName.c_str(), std::ios::out | std::ios::trunc);
    if (!s_telemetryCsvFile.is_open()) {
        USER_LOG_ERROR("failed to open telemetry csv file: %s", s_telemetryCsvFileName.c_str());
        return;
    }

    s_telemetryCsvFile << "wall_clock,velocity_x,velocity_y,velocity_z,gps_x,gps_y,gps_z,pitch,roll,yaw,gps_time,"
                          "rtk_lat,rtk_lon,rtk_h_msl,rtk_status,rtk_connected,gps_date,"
                          "rc_pitch,rc_roll,rc_yaw,rc_throttle,rc_connected\n";
    s_telemetryCsvFile.flush();

    USER_LOG_INFO("Continuous telemetry sample start. Type exit and press Enter to stop.");
    USER_LOG_INFO("Saving telemetry CSV to %s", s_telemetryCsvFileName.c_str());

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init fc subscription module error.");
        return;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("subscribe velocity topic error.");
        goto exit;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("subscribe gps position at 10 Hz failed, retrying at 5 Hz.");
        returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("subscribe gps position topic error.");
        goto exit;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("subscribe quaternion topic error.");
        goto exit;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("subscribe gps time at 10 Hz failed, retrying at 5 Hz.");
        returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("subscribe gps time topic error.");
        goto exit;
    }

    s_rtkPositionSubscribed = DjiUser_SubscribeOptional(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, "rtk position");
    s_rtkPositionInfoSubscribed = DjiUser_SubscribeOptional(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, "rtk position info");
    s_rtkConnectStatusSubscribed = DjiUser_SubscribeOptional(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, "rtk connect status");
    s_gpsDateSubscribed = DjiUser_SubscribeOptional(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, "gps date");
    s_rcSubscribed = DjiUser_SubscribeOptional(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA, "rc with flag data");

    while (s_telemetryStopRequested == 0) {
        returnCode = DjiUser_PrintTelemetryOnce();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("telemetry read cycle returned error: 0x%08llX", returnCode);
        }

        if (DjiUser_TelemetryExitRequested()) {
            USER_LOG_INFO("exit command received.");
            s_telemetryStopRequested = 1;
            break;
        }

        osalHandler->TaskSleepMs(DJI_TELEMETRY_STREAM_SLEEP_MS);
    }

exit:
    if (s_telemetryCsvFile.is_open()) {
        s_telemetryCsvFile.flush();
        s_telemetryCsvFile.close();
    }

    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME);
    if (s_rtkPositionSubscribed) DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    if (s_rtkPositionInfoSubscribed) DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);
    if (s_rtkConnectStatusSubscribed) DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS);
    if (s_gpsDateSubscribed) DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE);
    if (s_rcSubscribed) DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA);
    DjiFcSubscription_DeInit();

    USER_LOG_INFO("Continuous telemetry sample end.");
}

/* Private functions definition-----------------------------------------------*/
static void DjiUser_TelemetryStopSignalHandler(int signalNum)
{
    (void) signalNum;
    s_telemetryStopRequested = 1;
}

static bool DjiUser_TelemetryExitRequested(void)
{
    fd_set readFds;
    struct timeval timeout;
    char inputBuffer[64] = {0};

    FD_ZERO(&readFds);
    FD_SET(STDIN_FILENO, &readFds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    if (select(STDIN_FILENO + 1, &readFds, NULL, NULL, &timeout) <= 0) {
        return false;
    }

    if (std::cin.getline(inputBuffer, sizeof(inputBuffer))) {
        return std::string(inputBuffer) == "exit";
    }

    std::cin.clear();
    return false;
}

static bool DjiUser_SubscribeOptional(E_DjiFcSubscriptionTopic topic, const char *topicName)
{
    T_DjiReturnCode returnCode = DjiFcSubscription_SubscribeTopic(topic, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        returnCode = DjiFcSubscription_SubscribeTopic(topic, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("subscribe %s topic failed (0x%08llX); its CSV columns will be empty.", topicName, returnCode);
        return false;
    }
    USER_LOG_INFO("subscribed %s topic.", topicName);
    return true;
}

static T_DjiVector3f DjiUser_QuaternionToEulerAngles(const T_DjiFcSubscriptionQuaternion &quaternion)
{
    T_DjiVector3f eulerAngles = {0};
    const dji_f64_t pitch = asinf(-2.0 * quaternion.q1 * quaternion.q3 + 2.0 * quaternion.q0 * quaternion.q2) * 57.3;
    const dji_f64_t roll = atan2f(2.0 * quaternion.q2 * quaternion.q3 + 2.0 * quaternion.q0 * quaternion.q1,
                                  -2.0 * quaternion.q1 * quaternion.q1 - 2.0 * quaternion.q2 * quaternion.q2 + 1.0) *
                           57.3;
    const dji_f64_t yaw = atan2f(2.0 * quaternion.q1 * quaternion.q2 + 2.0 * quaternion.q0 * quaternion.q3,
                                 -2.0 * quaternion.q2 * quaternion.q2 - 2.0 * quaternion.q3 * quaternion.q3 + 1.0) *
                          57.3;

    eulerAngles.x = (dji_f32_t) pitch;
    eulerAngles.y = (dji_f32_t) roll;
    eulerAngles.z = (dji_f32_t) yaw;

    return eulerAngles;
}

static std::string DjiUser_GetTelemetryCsvFileName(void)
{
    std::time_t now = std::time(NULL);
    std::tm localTimeInfo;
    char timeBuffer[32] = {0};

#if defined(_POSIX_THREAD_SAFE_FUNCTIONS) && !defined(__APPLE__)
    localtime_r(&now, &localTimeInfo);
#else
    std::tm *timePointer = std::localtime(&now);
    if (timePointer != NULL) {
        localTimeInfo = *timePointer;
    } else {
        std::memset(&localTimeInfo, 0, sizeof(localTimeInfo));
    }
#endif

    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d_%H-%M-%S", &localTimeInfo);

    std::ostringstream fileNameStream;
    fileNameStream << "../../../Logs/telemetrydata/telemetry_" << timeBuffer << ".csv";

    return fileNameStream.str();
}

static T_DjiReturnCode DjiUser_EnsureDirectoryExists(const std::string &directoryPath)
{
    std::string partialPath;

    for (size_t i = 0; i < directoryPath.size(); ++i) {
        partialPath.push_back(directoryPath[i]);

        if (directoryPath[i] != '/' && i + 1 != directoryPath.size()) {
            continue;
        }

        if (partialPath.empty() || partialPath == "." || partialPath == ".." || partialPath == "/") {
            continue;
        }

        if (mkdir(partialPath.c_str(), 0775) != 0 && errno != EEXIST) {
            USER_LOG_ERROR("mkdir failed for %s, errno = %d", partialPath.c_str(), errno);
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiUser_PrintTelemetryOnce(void)
{
    T_DjiReturnCode returnCode;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsTime gpsTime = 0;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiVector3f eulerAngles = {0};
    std::ostringstream csvLine;
    char wallClockBuffer[32] = {0};
    std::chrono::system_clock::time_point wallClockNow = std::chrono::system_clock::now();
    std::time_t now = std::chrono::system_clock::to_time_t(wallClockNow);
    long wallClockMs = (long) (std::chrono::duration_cast<std::chrono::milliseconds>(
                                   wallClockNow.time_since_epoch()).count() % 1000);
    std::tm *wallClockTm = std::localtime(&now);

    if (wallClockTm != NULL) {
        char wallClockSecBuffer[24] = {0};
        std::strftime(wallClockSecBuffer, sizeof(wallClockSecBuffer), "%Y-%m-%d %H:%M:%S", wallClockTm);
        std::snprintf(wallClockBuffer, sizeof(wallClockBuffer), "%s.%03ld", wallClockSecBuffer, wallClockMs);
    } else {
        std::snprintf(wallClockBuffer, sizeof(wallClockBuffer), "unknown");
    }

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                         (uint8_t *) &velocity,
                                                         sizeof(T_DjiFcSubscriptionVelocity),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get velocity topic error: 0x%08llX", returnCode);
        return returnCode;
    }
    USER_LOG_INFO("velocity: x = %.3f y = %.3f z = %.3f, health = %u, ts = %u.%u",
                  velocity.data.x, velocity.data.y, velocity.data.z, velocity.health,
                  timestamp.millisecond, timestamp.microsecond);

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                         (uint8_t *) &gpsPosition,
                                                         sizeof(T_DjiFcSubscriptionGpsPosition),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get gps position topic error: 0x%08llX", returnCode);
        return returnCode;
    }
    USER_LOG_INFO("gps position: x = %.3f y = %.3f z = %.3f, ts = %u.%u",
                  gpsPosition.x, gpsPosition.y, gpsPosition.z,
                  timestamp.millisecond, timestamp.microsecond);

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                         (uint8_t *) &quaternion,
                                                         sizeof(T_DjiFcSubscriptionQuaternion),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get quaternion topic error: 0x%08llX", returnCode);
        return returnCode;
    }
    eulerAngles = DjiUser_QuaternionToEulerAngles(quaternion);
    USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f, ts = %u.%u",
                  eulerAngles.x, eulerAngles.y, eulerAngles.z,
                  timestamp.millisecond, timestamp.microsecond);

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                                         (uint8_t *) &gpsTime,
                                                         sizeof(T_DjiFcSubscriptionGpsTime),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get gps time topic error: 0x%08llX", returnCode);
        return returnCode;
    }
    USER_LOG_INFO("gps time: %u, ts = %u.%u", gpsTime, timestamp.millisecond, timestamp.microsecond);

    /* RTK: written as empty fields whenever the topic is unavailable or the read fails,
     * so a missing RTK fix is never confused with a value of zero. */
    std::string rtkLat, rtkLon, rtkHeight, rtkStatus, rtkConnected;
    if (s_rtkPositionSubscribed) {
        T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, (uint8_t *) &rtkPosition,
                                                    sizeof(rtkPosition), &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            char buf[40];
            std::snprintf(buf, sizeof(buf), "%.8f", rtkPosition.latitude); rtkLat = buf;
            std::snprintf(buf, sizeof(buf), "%.8f", rtkPosition.longitude); rtkLon = buf;
            std::snprintf(buf, sizeof(buf), "%.3f", rtkPosition.hfsl); rtkHeight = buf;
        }
    }
    if (s_rtkPositionInfoSubscribed) {
        T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo = 0;
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, (uint8_t *) &rtkPositionInfo,
                                                    sizeof(rtkPositionInfo), &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            rtkStatus = std::to_string((unsigned) rtkPositionInfo);
        }
    }
    if (s_rtkConnectStatusSubscribed) {
        T_DjiFcSubscriptionRTKConnectStatus rtkConnectStatus = {0};
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, (uint8_t *) &rtkConnectStatus,
                                                    sizeof(rtkConnectStatus), &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            rtkConnected = std::to_string((unsigned) rtkConnectStatus.rtkConnected);
        }
    }
    std::string gpsDate;
    if (s_gpsDateSubscribed) {
        T_DjiFcSubscriptionGpsDate gpsDateValue = 0;
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, (uint8_t *) &gpsDateValue,
                                                    sizeof(gpsDateValue), &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            gpsDate = std::to_string((unsigned long) gpsDateValue);
        }
    }
    /* Remote controller sticks (RC_WITH_FLAG_DATA): -0.999..0.999 with the centre deadzone already applied;
     * pitch/throttle up = +, roll/yaw right = +. Empty fields when the topic is unavailable or the read fails. */
    std::string rcPitch, rcRoll, rcYaw, rcThrottle, rcConnected;
    if (s_rcSubscribed) {
        T_DjiFcSubscriptionRCWithFlagData rc = {0};
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA, (uint8_t *) &rc,
                                                    sizeof(rc), &timestamp) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            char buf[24];
            std::snprintf(buf, sizeof(buf), "%.3f", rc.pitch); rcPitch = buf;
            std::snprintf(buf, sizeof(buf), "%.3f", rc.roll); rcRoll = buf;
            std::snprintf(buf, sizeof(buf), "%.3f", rc.yaw); rcYaw = buf;
            std::snprintf(buf, sizeof(buf), "%.3f", rc.throttle); rcThrottle = buf;
            rcConnected = std::to_string((unsigned) rc.flag.logicConnected);
        }
    }
    USER_LOG_INFO("rtk: lat = %s lon = %s h = %s status = %s (50 = fixed, 34 = float, 16 = single) connected = %s",
                  rtkLat.empty() ? "-" : rtkLat.c_str(), rtkLon.empty() ? "-" : rtkLon.c_str(),
                  rtkHeight.empty() ? "-" : rtkHeight.c_str(), rtkStatus.empty() ? "-" : rtkStatus.c_str(),
                  rtkConnected.empty() ? "-" : rtkConnected.c_str());
    USER_LOG_INFO("rc: pitch = %s roll = %s yaw = %s throttle = %s connected = %s",
                  rcPitch.empty() ? "-" : rcPitch.c_str(), rcRoll.empty() ? "-" : rcRoll.c_str(),
                  rcYaw.empty() ? "-" : rcYaw.c_str(), rcThrottle.empty() ? "-" : rcThrottle.c_str(),
                  rcConnected.empty() ? "-" : rcConnected.c_str());

    if (s_telemetryCsvFile.is_open()) {
        csvLine << wallClockBuffer << ','
                << std::fixed << std::setprecision(3)
                << velocity.data.x << ',' << velocity.data.y << ',' << velocity.data.z << ','
                << gpsPosition.x << ',' << gpsPosition.y << ',' << gpsPosition.z << ','
                << std::setprecision(2)
                << eulerAngles.x << ',' << eulerAngles.y << ',' << eulerAngles.z << ','
                << gpsTime << ','
                << rtkLat << ',' << rtkLon << ',' << rtkHeight << ',' << rtkStatus << ',' << rtkConnected << ','
                << gpsDate << ','
                << rcPitch << ',' << rcRoll << ',' << rcYaw << ',' << rcThrottle << ',' << rcConnected;

        s_telemetryCsvFile << csvLine.str() << '\n';
        s_telemetryCsvFile.flush();
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
