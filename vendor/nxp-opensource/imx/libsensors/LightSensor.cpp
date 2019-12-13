/*
 * Copyright (C) 2008 The Android Open Source Project
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "Sensors"
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <cutils/properties.h>
#include <string.h>

#include "LightSensor.h"

#define SENSOR_DEBUG

#ifdef SENSOR_DEBUG
#define DEBUG(format, ...) ALOGD((format), ## __VA_ARGS__)
#else
#define DEBUG(format, ...)
#endif

/*****************************************************************************/
LightSensor::LightSensor()
    : SensorBase("/dev/ltr559_als", "ltr559_als"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false),
      mThresholdLux(10)
{
    char  buffer[PROPERTY_VALUE_MAX];

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_LIGHT;
    mPendingEvent.type = SENSOR_TYPE_LIGHT;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd >= 0) {
		//ALOGE("orangeyang>>>>>> LightSensor::LightSensor data_fd >= 0");
        property_get("ro.hardware.lightsensor", buffer, "0");
        strcpy(ls_sysfs_path, buffer);
        ls_sysfs_path_len = strlen(ls_sysfs_path);
        enable(0, 1, 0);
    }

    /* Default threshold lux is 10 if ro.lightsensor.threshold
       isn't set */
    property_get("ro.lightsensor.threshold", buffer, "10");
    mThresholdLux = atoi(buffer);
}

LightSensor::~LightSensor() {
    if (mEnabled) {
        enable(0, 0, 0);
    }
}

int LightSensor::setDelay(int32_t handle, int64_t ns)
{
    //dummy due to not support in driver....
    return 0;
}

int LightSensor::getWhatFromHandle(int32_t handle)
{
	return 0;
}

int LightSensor::enable(int32_t handle, int en, int  __attribute__((unused))type)
{
    char buf[2];
    int n;
    int flags = en ? 1 : 0;

	//ALOGE("orangeyang>>>>>>111 LightSensor::enable mEnabled=%d, flags=%d", mEnabled, flags);

    mPreviousLight = -1;
    if (flags != mEnabled) {
        FILE *fd = NULL;
        strcpy(&ls_sysfs_path[ls_sysfs_path_len], "alsmodesetup");
        fd = fopen(ls_sysfs_path, "w");
		//ALOGE("orangeyang>>>>>>222 LightSensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
        if (fd) {
			//ALOGE("orangeyang>>>>>>333 LightSensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
            memset(buf, 0, 2);
            if (flags)
                snprintf(buf, 2, "%d", 1);
            else
                snprintf(buf, 2, "%d", 0);
            n = fwrite(buf, 1, 1, fd);
            fclose(fd);
            mEnabled = flags;
			//ALOGE("orangeyang>>>>>>444 LightSensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
            if (flags)
                setIntLux();
            return 0;
        }
        return -1;
    }
    return 0;
}

int LightSensor::setIntLux()
{
    FILE *fd = NULL;
    char buf[6];
    int n, lux, int_ht_lux, int_lt_lux;

    /* Read current lux value firstly, then change Delta value */
    strcpy(&ls_sysfs_path[ls_sysfs_path_len], "als_adc");
    if ((fd = fopen(ls_sysfs_path, "r")) == NULL) {
        ALOGE("Unable to open %s\n", ls_sysfs_path);
        return -1;
    }
    memset(buf, 0, 6);
    if ((n = fread(buf, 1, 6, fd)) < 0) {
        ALOGE("Unable to read %s\n", ls_sysfs_path);
        fclose(fd);
	return -1;
    }
    fclose(fd);

    lux = atoi(buf);
    int_ht_lux = lux + mThresholdLux;
    int_lt_lux = lux - mThresholdLux;

    if (int_lt_lux < 0)
	    int_lt_lux = 0;

    DEBUG("Current light is %d lux\n", lux);

    /* Set low lux and high interrupt lux for polling */
    strcpy(&ls_sysfs_path[ls_sysfs_path_len], "setalslothrerange");
    fd = fopen(ls_sysfs_path, "w");
    if (fd) {
        memset(buf, 0, 6);
        snprintf(buf, 6, "%d", int_lt_lux);
        n = fwrite(buf, 1, 6, fd);
        fclose(fd);
    } else
        ALOGE("Couldn't open %s file\n", ls_sysfs_path);
    strcpy(&ls_sysfs_path[ls_sysfs_path_len], "setalshithrerange");
    fd = fopen(ls_sysfs_path, "w");
    if (fd) {
        memset(buf, 0, 6);
        snprintf(buf, 6, "%d", int_ht_lux);
        n = fwrite(buf, 1, 6, fd);
        fclose(fd);
    } else
        ALOGE("Couldn't open %s file\n", ls_sysfs_path);

    return 0;
}
bool LightSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int LightSensor::getTimestamp()
{
	struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == ABS_MISC) {
                mPendingEvent.light = event->value;
				//ALOGE("orangeyang>>>>>> event->value = %d", event->value);
                setIntLux();
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
			//ALOGE("orangeyang>>>>>> mEnabled = %d, mPendingEvent.light = %d, mPreviousLight = %d", mEnabled, mPendingEvent.light, mPreviousLight);
            if (mEnabled && (mPendingEvent.light != mPreviousLight)) {
				//ALOGE("orangeyang>>>>>> mEnabled = %d", mEnabled);
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
                mPreviousLight = mPendingEvent.light;
            }
        } else {
            ALOGE("LightSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

void LightSensor::processEvent(int code, int value)
{
}
