/**
 ********************************************************************
 * @file    test_payload_cam_emu_media.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "dji_logger.h"
#include "test_raspberry_pi_camera.h"
#include <fcntl.h>
#include <unistd.h>
#include "utils/util_misc.h"
#include "utils/util_time.h"
#include "utils/util_file.h"
#include "utils/util_buffer.h"
#include "dji_platform.h"
#include "time.h"
#include <sys/stat.h>

/* Private constants ---------------------------------------------------------*/
#define VIDEO_FRAME_AUD_LENGTH                  6
#define RSP_MEDIA_FILE_STORE_PATH __FILE__
#define VIDEO_BUFFER_SIZE                       1024 * 4
#define START_VIDEO_STREAM_CMD "libcamera-vid -t 0 --width 1280 --height 720 --framerate 25 --codec h264 --inline --output - --denoise auto --profile high --bitrate 2000000 2>/dev/null"
/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void DjiTest_ProcessRawH264WithAUD(uint8_t* data, size_t length);
static void * DjiTest_H264StreamControlTask(void* arg);
static void DjiTest_PauseStreaming(void);
static void DjiTest_ResumeStreaming(void);
static void *DjiTest_RaspberryPiCameraTask(void *arg);
static T_DjiReturnCode DjiTest_CameraTakePhotoImpl(const char *filename);
static void DjiTest_ProcessSingleNALUnit(uint8_t* nal_data, size_t nal_length);
static size_t DjiTest_ProcessCompleteNALUnits(uint8_t* data, size_t length);
static void DjiTest_ProcessRawH264WithBuffer(uint8_t* data, size_t length);

/* Private variables -------------------------------------------------------------*/
static int s_photoCount = 0;
static bool s_recordingFlag = false;
static const uint8_t s_frameAudData[VIDEO_FRAME_AUD_LENGTH] = {0x00, 0x00, 0x00, 0x01, 0x09, 0x10};
static T_DjiMutexHandle s_cameraMutex;
static bool camera_init_flag = false;
static T_DjiTaskHandle s_camerLiveviewThread;
static bool s_recording = false;
static bool s_streaming_paused = false;
static FILE *s_output_file = NULL;
static FILE *s_camera_stream = NULL;
static bool s_cameraTaskRunningFlag = false;
static uint8_t *s_nal_buffer = NULL;
static size_t s_nal_buffer_size = 0;
static size_t s_nal_buffer_capacity = 0;
static bool s_liveviewCameraState = true;
static T_DjiMutexHandle s_liveviewStateMutex;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_RaspberryPiCameraInit() {
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    s_photoCount = 0;
    s_recordingFlag = false;

    s_nal_buffer_capacity = VIDEO_BUFFER_SIZE;
    s_nal_buffer = malloc(s_nal_buffer_capacity);
    if (!s_nal_buffer) {
        USER_LOG_ERROR("malloc nal buffer error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    s_nal_buffer_size = 0;

    if(osalHandler->MutexCreate(&s_cameraMutex)!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create mutex error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if(osalHandler->MutexCreate(&s_liveviewStateMutex)!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create mutex error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    s_cameraTaskRunningFlag = true;

    returnCode = osalHandler->TaskCreate("user_camera_media_task", DjiTest_RaspberryPiCameraTask, 2048,
                                            NULL, &s_camerLiveviewThread);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("raspberry pi camera video task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    camera_init_flag = true;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void DjiTest_RaspberryPiCameraTakePhoto()
{
    if(!camera_init_flag) {
        USER_LOG_ERROR("camera not init");
        return;
    }
    DjiPlatform_GetOsalHandler()->MutexLock(s_cameraMutex);
    s_photoCount ++;
    DjiPlatform_GetOsalHandler()->MutexUnlock(s_cameraMutex);
}

void DjiTest_RaspberryPiRecordingAction(bool action)
{
    if(!camera_init_flag) {
        USER_LOG_ERROR("camera not init");
        return;
    }
    DjiPlatform_GetOsalHandler()->MutexLock(s_cameraMutex);
    s_recordingFlag = action;
    DjiPlatform_GetOsalHandler()->MutexUnlock(s_cameraMutex);
}

void DjiTest_raspberryPiCameraDeinit(void) {
    s_cameraTaskRunningFlag = false;

    if (s_nal_buffer) {
        free(s_nal_buffer);
        s_nal_buffer = NULL;
    }

    if (s_cameraMutex) {
        DjiPlatform_GetOsalHandler()->MutexDestroy(s_cameraMutex);
        s_cameraMutex = NULL;
    }

    if (s_liveviewStateMutex) {
        DjiPlatform_GetOsalHandler()->MutexDestroy(s_liveviewStateMutex);
        s_liveviewStateMutex = NULL;
    }

    USER_LOG_INFO("Raspberry Pi Camera deinitialized");
}

void DjiTest_RaspberryPiCameraLiveviewStatSet(bool liveviewState) {
    DjiPlatform_GetOsalHandler()->MutexLock(s_liveviewStateMutex);
    s_liveviewCameraState = liveviewState;
    DjiPlatform_GetOsalHandler()->MutexUnlock(s_liveviewStateMutex);
}

void DjiTest_RaspberryPiCameraLiveviewStatGet(bool *liveviewState) {
    DjiPlatform_GetOsalHandler()->MutexLock(s_liveviewStateMutex);
    *liveviewState = s_liveviewCameraState;
    DjiPlatform_GetOsalHandler()->MutexUnlock(s_liveviewStateMutex);
}

/* Private functions definition-----------------------------------------------*/
static void *DjiTest_RaspberryPiCameraTask(void *arg) {
    T_DjiReturnCode returnCode;
    T_DjiTaskHandle streamControllerHandler = NULL;
    static bool local_photo_flag = false;
    static bool local_recording_flag = false;
    static bool last_recording_flag = false;
    char current_h264_path[256] = {0};

    returnCode = DjiPlatform_GetOsalHandler()->TaskCreate("264_stream_contrller", DjiTest_H264StreamControlTask, 2048,
                                            NULL, &streamControllerHandler);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create stream controller task error.");
        return NULL;
    }

    DjiPlatform_GetOsalHandler()->TaskSleepMs(1000);

    while(s_cameraTaskRunningFlag) {
        T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
        osalHandler->MutexLock(s_cameraMutex);
        local_photo_flag = s_photoCount > 0;
        local_recording_flag = s_recordingFlag;
        osalHandler->MutexUnlock(s_cameraMutex);

        if(local_photo_flag) {
            USER_LOG_INFO("raspberry pi camera pause streaming");
            DjiTest_PauseStreaming();

            char full_path[256];
            time_t now = time(NULL);
            snprintf(full_path, sizeof(full_path), "%s/photo_%ld.jpg", RSP_MEDIA_FILE_STORE_PATH, now);
            USER_LOG_INFO("raspberry pi camera take photo ......");
            if (DjiTest_CameraTakePhotoImpl(full_path) == 0) {
                USER_LOG_INFO("photo saved as: %s\n", full_path);
            } else {
                USER_LOG_ERROR("failed to take photo\n");
            }

            DjiTest_ResumeStreaming();
            osalHandler->MutexLock(s_cameraMutex);
            if (s_photoCount > 0) {
                s_photoCount--;
            }
            osalHandler->MutexUnlock(s_cameraMutex);
            USER_LOG_INFO("raspberry pi camera resume streaming");
        }

       if(last_recording_flag != local_recording_flag) {
            if(local_recording_flag) {
                s_recording = true;
                time_t now = time(NULL);
                snprintf(current_h264_path, sizeof(current_h264_path), "%s/video_%ld.h264", RSP_MEDIA_FILE_STORE_PATH, now);
                s_output_file = fopen(current_h264_path, "wb");
                if (!s_output_file) {
                    perror("fopen");
                    s_recording = false;
                    USER_LOG_ERROR("create recording file failed");
                } else {
                    USER_LOG_INFO("raspberry pi camera start recording: %s", current_h264_path);
                }

            } else {
                USER_LOG_INFO("raspberry pi camera stop recording");
                s_recording = false;
                if (s_output_file) {
                    fclose(s_output_file);
                    s_output_file = NULL;

                    if (strlen(current_h264_path) > 0) {
                        char mp4_path[256];
                        snprintf(mp4_path, sizeof(mp4_path), "%s", current_h264_path);
                        char *ext = strrchr(mp4_path, '.');
                        if (ext && strcmp(ext, ".h264") == 0) {
                            strcpy(ext, ".mp4");
                            char cmd[512];
                            snprintf(cmd, sizeof(cmd), "ffmpeg -framerate 25 -i \"%s\" -c copy -y \"%s\" 2>/dev/null",
                                     current_h264_path, mp4_path);

                            USER_LOG_INFO("Converting video: %s", cmd);
                            int result = system(cmd);

                            if (result == 0) {
                                USER_LOG_INFO("Video converted successfully: %s", mp4_path);
                                if (remove(current_h264_path) == 0) {
                                    USER_LOG_INFO("Deleted temporary file: %s", current_h264_path);
                                } else {
                                    USER_LOG_ERROR("Failed to delete temporary file: %s", current_h264_path);
                                }
                            } else {
                                USER_LOG_ERROR("Failed to convert video file");
                            }
                        }
                        memset(current_h264_path, 0, sizeof(current_h264_path));
                    }
                }
            }
            last_recording_flag = local_recording_flag;
        }
       osalHandler->TaskSleepMs(10);
    }

    USER_LOG_INFO("raspberry pi camera shut down");
    if (s_output_file) {
        fclose(s_output_file);
    }

    if (s_camera_stream) {
        pclose(s_camera_stream);
        s_camera_stream = NULL;
    }

    if (s_camera_stream) pclose(s_camera_stream);

    if (streamControllerHandler) {
        DjiPlatform_GetOsalHandler()->TaskDestroy(streamControllerHandler);
    }

    return NULL;
}

static void DjiTest_PauseStreaming() {
    s_streaming_paused = true;
    if (s_camera_stream) {
        pclose(s_camera_stream);
        s_camera_stream = NULL;
    }
    DjiPlatform_GetOsalHandler()->TaskSleepMs(100);
    USER_LOG_INFO("Streaming paused");
}

static void DjiTest_ResumeStreaming() {

    s_camera_stream = popen(START_VIDEO_STREAM_CMD, "r");
    if (!s_camera_stream) {
        perror("popen libcamera-vid");
        return;
    }

    s_streaming_paused = false;
    USER_LOG_INFO("Streaming resumed");
}

static void * DjiTest_H264StreamControlTask(void* arg)
{
    unsigned char buffer[VIDEO_BUFFER_SIZE];
    size_t bytes_read;

    s_camera_stream = popen(START_VIDEO_STREAM_CMD, "r");
    if (!s_camera_stream) {
        perror("popen libcamera-vid");
        exit(EXIT_FAILURE);
    }

    USER_LOG_INFO("H.264 stream control task started");

    while (1) {
        if (s_streaming_paused) {
            DjiPlatform_GetOsalHandler()->TaskSleepMs(10);
            continue;
        }

        if (s_camera_stream && !feof(s_camera_stream)) {

            bytes_read = fread(buffer, 1, sizeof(buffer), s_camera_stream);
            if (bytes_read > 0) {
                DjiTest_ProcessRawH264WithBuffer(buffer, bytes_read);
            }
        } else {
            DjiPlatform_GetOsalHandler()->TaskSleepMs(10);
        }

        if (!s_cameraTaskRunningFlag) {
            break;
        }
    }

    if (s_camera_stream) {
        pclose(s_camera_stream);
        s_camera_stream = NULL;
    }

    USER_LOG_INFO("H.264 stream control task terminated");
    return NULL;
}

static size_t DjiTest_ProcessCompleteNALUnits(uint8_t* data, size_t length)
{
    if (!data || length < 4) return 0;

    uint8_t* current_pos = data;
    size_t remaining = length;
    size_t processed_bytes = 0;

    while (remaining >= 4) {
        // find nal start
        if (current_pos[0] == 0x00 && current_pos[1] == 0x00 &&
            current_pos[2] == 0x00 && current_pos[3] == 0x01) {

            uint8_t* nal_start = current_pos;
            uint8_t* next_start = NULL;

            // find the next nal start
            for (size_t i = 4; i < remaining - 3; i++) {
                if (current_pos[i] == 0x00 && current_pos[i+1] == 0x00 &&
                    current_pos[i+2] == 0x00 && current_pos[i+3] == 0x01) {
                    next_start = current_pos + i;
                    break;
                }
            }

            if (next_start) {
                size_t nal_size = next_start - nal_start;
                DjiTest_ProcessSingleNALUnit(nal_start, nal_size);
                processed_bytes = next_start - data;
                current_pos = next_start;
                remaining = length - processed_bytes;
            } else {
                break;
            }
        } else {
            current_pos++;
            remaining--;
            processed_bytes++;
        }
    }

    return processed_bytes;
}

static void DjiTest_ProcessSingleNALUnit(uint8_t* nal_data, size_t nal_length)
{
    bool local_recording = false;
    bool liveviewState = true;
    uint8_t* frame_with_aud = NULL;

    if (!nal_data || nal_length == 0) return;
    // add aud
    size_t total_frame_size = nal_length + VIDEO_FRAME_AUD_LENGTH;
    frame_with_aud = malloc(total_frame_size);

    if (!frame_with_aud) {
        USER_LOG_ERROR("Failed to allocate frame buffer");
        return;
    }

    memcpy(frame_with_aud, nal_data, nal_length);
    memcpy(frame_with_aud + nal_length, s_frameAudData, VIDEO_FRAME_AUD_LENGTH);

    DjiPlatform_GetOsalHandler()->MutexLock(s_cameraMutex);
    local_recording = s_recording;
    DjiPlatform_GetOsalHandler()->MutexUnlock(s_cameraMutex);

    if (local_recording && s_output_file && total_frame_size > 0) {
        size_t total_fwritten = 0;
        while (total_fwritten < total_frame_size) {
            size_t fw = fwrite(frame_with_aud + total_fwritten, 1, total_frame_size - total_fwritten, s_output_file);
            if (fw > 0) {
                total_fwritten += fw;
                continue;
            }
            if (ferror(s_output_file)) {
                perror("fwrite error");
                clearerr(s_output_file);
                break;
            }
        }
        fflush(s_output_file);
    }

    if (total_frame_size > 60000) {
        size_t offset = 0;
        while (offset < total_frame_size) {
            size_t send_length = (total_frame_size - offset > 6000) ? 6000 : (total_frame_size - offset);
            DjiTest_RaspberryPiCameraLiveviewStatGet(&liveviewState);
            if(!liveviewState) goto free;
            T_DjiReturnCode returnCode = DjiPayloadCamera_SendVideoStream(
                frame_with_aud + offset,
                send_length
            );

            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("send fragmented video stream failed: 0x%08llX", returnCode);
            }

            offset += send_length;
        }
    } else {
        DjiTest_RaspberryPiCameraLiveviewStatGet(&liveviewState);
        if(!liveviewState) goto free;
        T_DjiReturnCode returnCode = DjiPayloadCamera_SendVideoStream(
            frame_with_aud,
            total_frame_size
        );

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("send video stream failed: 0x%08llX", returnCode);
        }
    }

free:
    free(frame_with_aud);
}

static void DjiTest_ProcessRawH264WithBuffer(uint8_t* data, size_t length)
{
    if (!data || length == 0) return;

    size_t required_capacity = s_nal_buffer_size + length;
    if (s_nal_buffer_capacity < required_capacity) {
        size_t new_capacity = required_capacity * 2;
        uint8_t *new_buffer = realloc(s_nal_buffer, new_capacity);
        if (!new_buffer) {
            USER_LOG_ERROR("realloc failed");
            return;
        }
        s_nal_buffer = new_buffer;
        s_nal_buffer_capacity = new_capacity;
    }

    memcpy(s_nal_buffer + s_nal_buffer_size, data, length);
    s_nal_buffer_size += length;

    size_t processed_bytes = DjiTest_ProcessCompleteNALUnits(s_nal_buffer, s_nal_buffer_size);

    if (processed_bytes < s_nal_buffer_size) {
        memmove(s_nal_buffer, s_nal_buffer + processed_bytes, s_nal_buffer_size - processed_bytes);
        s_nal_buffer_size -= processed_bytes;
    } else {
        s_nal_buffer_size = 0;
    }
}

static T_DjiReturnCode DjiTest_CameraTakePhotoImpl(const char *filename) {
    if (!filename) {
        USER_LOG_ERROR("Invalid filename");
        return -1;
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd),
            "libcamera-still -o %s --width 1280 --height 720 --quality 90 --timeout 100 --framerate 25 --verbose 0 2>/dev/null",
            filename);

    USER_LOG_INFO("Taking photo: %s", filename);
    int result = system(cmd);

    FILE *test = fopen(filename, "rb");
    if (test) {
        fclose(test);
        USER_LOG_INFO("Photo captured successfully: %s", filename);
        return 0;
    } else {
        USER_LOG_ERROR("Failed to capture photo: %s", filename);
        return -1;
    }
}

//#endif

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/