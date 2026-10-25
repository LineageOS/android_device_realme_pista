/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "CameraOemlayerWrapper"

#include <dlfcn.h>
#include <errno.h>
#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <hardware/hardware.h>
#include <log/log.h>
#include <system/camera_metadata.h>

#include <mutex>
#include <unordered_map>
#include <vector>

struct alignas(void*) ExtendedCameraModule {
    union {
        camera_module_t base;
        uint8_t raw[512];
    };
};

static int configure_streams_wrapper(const struct camera3_device *dev,
        camera3_stream_configuration_t *stream_list);
static int device_close_wrapper(struct hw_device_t* device);
static int open_wrapper(const struct hw_module_t* module, const char* id,
        struct hw_device_t** device);
static int get_camera_info_wrapper(int camera_id, struct camera_info *info);

static hw_module_methods_t g_methods_wrapper = {
    .open = open_wrapper,
};

extern "C" {
ExtendedCameraModule HAL_MODULE_INFO_SYM = {
    .base = {
        .common = {
            .tag = HARDWARE_MODULE_TAG,
            .module_api_version = CAMERA_MODULE_API_VERSION_2_5,
            .hal_api_version = HARDWARE_HAL_API_VERSION,
            .id = CAMERA_HARDWARE_MODULE_ID,
            .name = "Oplus Camera HAL Extension",
            .author = "LineageOS",
            .methods = &g_methods_wrapper,
            .dso = nullptr,
            .reserved = {0},
        },
        .get_number_of_cameras = nullptr,
        .get_camera_info = get_camera_info_wrapper,
        .set_callbacks = nullptr,
        .get_vendor_tag_ops = nullptr,
        .open_legacy = nullptr,
        .set_torch_mode = nullptr,
        .init = nullptr,
        .get_physical_camera_info = nullptr,
        .is_stream_combination_supported = nullptr,
        .notify_device_state_change = nullptr,
        .reserved = {nullptr, nullptr},
    },
};
}

static camera_module_t* g_real_module = nullptr;
static void* g_real_handle = nullptr;
static std::mutex g_lock;
static std::unordered_map<int, camera_metadata_t*> g_modified_metadata;

static std::unordered_map<camera3_device_t*, camera3_device_ops_t*> g_device_ops_map;
static std::unordered_map<camera3_device_t*, int (*)(struct hw_device_t*)> g_device_close_map;

static int (*g_real_open)(const struct hw_module_t* module, const char* id,
                          struct hw_device_t** device) = nullptr;
static int (*g_real_get_camera_info)(int camera_id, struct camera_info *info) = nullptr;

static bool ensure_real_module() {
    std::lock_guard<std::mutex> lock(g_lock);
    if (g_real_module != nullptr) {
        return true;
    }

    const char* paths[] = {
        "/odm/lib64/hw/camera.oemlayer.so",
        "/vendor/lib64/hw/camera.oemlayer.so",
        nullptr
    };
    for (int i = 0; paths[i] != nullptr; i++) {
        g_real_handle = dlopen(paths[i], RTLD_NOW);
        if (g_real_handle != nullptr) {
            g_real_module = reinterpret_cast<camera_module_t*>(dlsym(g_real_handle, HAL_MODULE_INFO_SYM_AS_STR));
            if (g_real_module != nullptr) {
                ALOGI("%s: Loaded real camera module directly from %s", __FUNCTION__, paths[i]);

                // Copy all 400 bytes of the real OEM module so all proprietary extension
                // methods (e.g. at offset 0x148 which calls SetOplusCameraInfoCallback) are preserved.
                memcpy(&HAL_MODULE_INFO_SYM, g_real_module, 400);

                if (g_real_module->common.methods != nullptr) {
                    g_real_open = g_real_module->common.methods->open;
                }
                g_real_get_camera_info = g_real_module->get_camera_info;

                // Hook open and get_camera_info
                g_methods_wrapper.open = open_wrapper;
                HAL_MODULE_INFO_SYM.base.common.methods = &g_methods_wrapper;
                HAL_MODULE_INFO_SYM.base.get_camera_info = get_camera_info_wrapper;

                ALOGI("%s: oemlayer module cast successfully", __FUNCTION__);
                return true;
            }
            dlclose(g_real_handle);
            g_real_handle = nullptr;
        }
    }

    ALOGE("%s: Failed to load camera.oemlayer.so", __FUNCTION__);
    return false;
}

__attribute__((constructor))
static void constructor_wrapper() {
    ensure_real_module();
}

static int get_camera_info_wrapper(int camera_id, struct camera_info *info) {
    if (!ensure_real_module() || g_real_get_camera_info == nullptr) {
        return -ENODEV;
    }

    int ret = g_real_get_camera_info(camera_id, info);
    if (ret != 0 || info == nullptr || info->static_camera_characteristics == nullptr) {
        return ret;
    }

    camera_metadata_ro_entry_t entry;
    bool is_logical = false;
    if (find_camera_metadata_ro_entry(info->static_camera_characteristics,
            ANDROID_REQUEST_AVAILABLE_CAPABILITIES, &entry) == 0) {
        for (size_t i = 0; i < entry.count; i++) {
            if (entry.data.u8[i] == ANDROID_REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA) {
                is_logical = true;
                break;
            }
        }
    }

    // Filter capabilities for logical multi-camera or primary back camera (camera 0)
    if (is_logical || camera_id == 0) {
        std::lock_guard<std::mutex> lock(g_lock);
        auto it = g_modified_metadata.find(camera_id);
        if (it != g_modified_metadata.end()) {
            info->static_camera_characteristics = it->second;
            return 0;
        }

        size_t entry_capacity = get_camera_metadata_entry_count(info->static_camera_characteristics) + 16;
        size_t data_capacity = get_camera_metadata_data_count(info->static_camera_characteristics) + 1024;
        camera_metadata_t* copy = allocate_camera_metadata(entry_capacity, data_capacity);
        if (copy != nullptr && append_camera_metadata(copy, info->static_camera_characteristics) == 0) {
            // Remove Ultra HDR (JPEG_R) configurations to avoid capture freeze caused by SnapAlloc format 0x36/0x21
            const uint32_t jpegr_tags[] = {
                ANDROID_JPEGR_AVAILABLE_JPEG_R_STREAM_CONFIGURATIONS,
                ANDROID_JPEGR_AVAILABLE_JPEG_R_MIN_FRAME_DURATIONS,
                ANDROID_JPEGR_AVAILABLE_JPEG_R_STALL_DURATIONS,
            };
            for (uint32_t tag : jpegr_tags) {
                camera_metadata_ro_entry_t jpegr_entry;
                if (find_camera_metadata_ro_entry(copy, tag, &jpegr_entry) == 0) {
                    delete_camera_metadata_entry(copy, jpegr_entry.index);
                    ALOGI("%s: Removed JPEG_R tag 0x%x for camera %d", __FUNCTION__, tag, camera_id);
                }
            }

            g_modified_metadata[camera_id] = copy;
            info->static_camera_characteristics = copy;
        } else {
            if (copy != nullptr) {
                free_camera_metadata(copy);
            }
        }
    }

    return 0;
}

static int configure_streams_wrapper(const struct camera3_device *dev,
        camera3_stream_configuration_t *stream_list) {
    if (dev == nullptr || stream_list == nullptr) {
        return -EINVAL;
    }

    camera3_device_ops_t* orig_ops = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_lock);
        auto it = g_device_ops_map.find(const_cast<camera3_device_t*>(dev));
        if (it != g_device_ops_map.end()) {
            orig_ops = it->second;
        }
    }

    if (orig_ops == nullptr || orig_ops->configure_streams == nullptr) {
        return -EINVAL;
    }

    // Strip GRALLOC_USAGE_HW_CAMERA_ZSL (0x60000) from any stream to prevent ChiX
    // from attaching offline ZSLSnapshotYUVHAL pipeline that deadlocks on SAT lens switch.
    for (size_t i = 0; i < stream_list->num_streams; i++) {
        camera3_stream_t *s = stream_list->streams[i];
        if (s != nullptr && (s->usage & 0x60000) == 0x60000) {
            ALOGW("%s: Stripping 0x60000 ZSL usage from stream %zu (%dx%d, format 0x%x)",
                    __FUNCTION__, i, s->width, s->height, s->format);
            s->usage &= ~0x60000;
        }
    }

    return orig_ops->configure_streams(dev, stream_list);
}

static int device_close_wrapper(struct hw_device_t* device) {
    if (device == nullptr) return 0;
    camera3_device_t *cam3_dev = reinterpret_cast<camera3_device_t*>(device);

    camera3_device_ops_t* orig_ops = nullptr;
    int (*orig_close)(struct hw_device_t*) = nullptr;
    camera3_device_ops_t* ops_wrapper = cam3_dev->ops;

    {
        std::lock_guard<std::mutex> lock(g_lock);
        auto it_ops = g_device_ops_map.find(cam3_dev);
        if (it_ops != g_device_ops_map.end()) {
            orig_ops = it_ops->second;
            g_device_ops_map.erase(it_ops);
        }
        auto it_close = g_device_close_map.find(cam3_dev);
        if (it_close != g_device_close_map.end()) {
            orig_close = it_close->second;
            g_device_close_map.erase(it_close);
        }
    }

    if (orig_ops != nullptr) {
        cam3_dev->ops = orig_ops;
    }
    delete ops_wrapper;

    if (orig_close != nullptr) {
        return orig_close(device);
    }
    return 0;
}

static int open_wrapper(const struct hw_module_t* /*module*/, const char* id,
        struct hw_device_t** device) {
    if (!ensure_real_module() || g_real_open == nullptr) {
        return -ENODEV;
    }

    int ret = g_real_open(reinterpret_cast<const hw_module_t*>(g_real_module), id, device);
    if (ret != 0 || device == nullptr || *device == nullptr) {
        return ret;
    }

    camera3_device_t *cam3_dev = reinterpret_cast<camera3_device_t*>(*device);
    if (cam3_dev->ops != nullptr) {
        std::lock_guard<std::mutex> lock(g_lock);
        camera3_device_ops_t *new_ops = new camera3_device_ops_t(*cam3_dev->ops);
        g_device_ops_map[cam3_dev] = cam3_dev->ops;
        g_device_close_map[cam3_dev] = cam3_dev->common.close;

        new_ops->configure_streams = configure_streams_wrapper;
        cam3_dev->ops = new_ops;
        cam3_dev->common.close = device_close_wrapper;
        ALOGI("%s: Hooked camera3_device_ops for camera id %s", __FUNCTION__, id ? id : "null");
    }

    return 0;
}
