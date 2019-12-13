/*
 * Copyright (C) 2008 The Android Open Source Project
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
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <string.h>
#include <cutils/log.h>
#include <cutils/properties.h>

#include "ProximitySensor.h"

#define PROXIMITY_THRESHOLD_GP2A (0.5f)

/*****************************************************************************/

ProximitySensor::ProximitySensor()
    : SensorBase("/dev/ltr559_ps", "ltr559_ps"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
	char  buffer[PROPERTY_VALUE_MAX];
	
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_PROXIMITY;
    mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd) {
		//ALOGE("orangeyang>>>>>><<<<<<000000 ProximitySensor::ProximitySensor data_fd=%d", data_fd);
        property_get("ro.hardware.proximitysensor", buffer, "0");
        strcpy(input_sysfs_path, buffer);
        input_sysfs_path_len = strlen(input_sysfs_path);
        enable(0, 1, 0);
    }
}

ProximitySensor::~ProximitySensor() {
    if (mEnabled) {
        enable(0, 0, 0);
    }
}

//int ProximitySensor::setInitialState() {
//    struct input_absinfo absinfo;
//    if (!ioctl(data_fd, EVIOCGABS(SENSOR_TYPE_PROXIMITY), &absinfo)) {
//		ALOGE("orangeyang>>>>>><<<<<<888888 roximitySensor::setInitialState=%d", mHasPendingEvent);
//        // make sure to report an event immediately
//        mHasPendingEvent = true;
//        mPendingEvent.distance = indexToValue(absinfo.value);
//    }
//	mHasPendingEvent = false;
//	ALOGE("orangeyang>>>>>><<<<<<777777 roximitySensor::setInitialState=%d", mHasPendingEvent);
//    return 0;
//}

int ProximitySensor::enable(int32_t, int en, int  __attribute__((unused))type) {
	char buf[2];
	int n;
    int flags = en ? 1 : 0;
	
	mPreviousProximity = -1;
	
	//ALOGE("orangeyang>>>>>><<<<<<111111 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
    if (flags != mEnabled) {
        //int fd;
		FILE *fd = NULL;
		//ALOGE("orangeyang>>>>>><<<<<<222222 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
        strcpy(&input_sysfs_path[input_sysfs_path_len], "psmodesetup");
        //fd = open(input_sysfs_path, O_RDWR);
		//ALOGE("orangeyang>>>>>><<<<<<333333 ProximitySensor::enable input_sysfs_path=%s", input_sysfs_path);
		fd = fopen(input_sysfs_path, "w");
 #if 1       
		if (fd) {
			//ALOGE("orangeyang>>>>>><<<<<<444444 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
            memset(buf, 0, 2);
            if (flags)
                snprintf(buf, 2, "%d", 1);
            else
                snprintf(buf, 2, "%d", 0);
            n = fwrite(buf, 1, 1, fd);
            fclose(fd);
            mEnabled = flags;
			//ALOGE("orangeyang>>>>>><<<<<<555555 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
#else
		if (fd >= 0) {
			//ALOGE("orangeyang>>>>>><<<<<<444444 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
            char buf[2];
            buf[1] = 0;
            if (flags) {
                buf[0] = '1';
            } else {
                buf[0] = '0';
            }
            write(fd, buf, sizeof(buf));
            close(fd);
            mEnabled = flags;
			//ALOGE("orangeyang>>>>>><<<<<<555555 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
#endif
            //setInitialState();
            return 0;
        }
		//ALOGE("orangeyang>>>>>><<<<<<666666 ProximitySensor::enable mEnabled=%d, flags=%d", mEnabled, flags);
        return -1;
    }
    return 0;
}

bool ProximitySensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int ProximitySensor::getTimestamp()
{
	struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}

int ProximitySensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

	//LOGE("orangeyang>>>>>><<<<<<999999 ProximitySensor::enable mEnabled=%d", mEnabled);

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
			//ALOGE("orangeyang>>>>>><<<<<<1000000 ProximitySensor::readEvents mEnabled=%d", mEnabled);
            if (event->code == ABS_DISTANCE) {
				//ALOGE("orangeyang>>>>>><<<<<<1111111 ProximitySensor::readEvents mEnabled=%d", mEnabled);
                if (event->value != -1) {
                    // FIXME: not sure why we're getting -1 sometimes
					//ALOGE("orangeyang>>>>>><<<<<<2 ProximitySensor::readEvents mPendingEvent.distance=%d, event->value=%d", mPendingEvent.distance, event->value);
                    mPendingEvent.distance = event->value;//indexToValue(event->value);
					//ALOGE("orangeyang>>>>>><<<<<<1222222 ProximitySensor::readEvents mPendingEvent.distance=%d, event->value=%d", mPendingEvent.distance, event->value);
                }
            }
        } else if (type == EV_SYN) {
			//ALOGE("orangeyang>>>>>><<<<<<1333333 ProximitySensor::readEvents mEnabled=%d", mEnabled);
            mPendingEvent.timestamp = timevalToNano(event->time);
			//ALOGE("orangeyang>>>>>><<<<<<3 ProximitySensor::readEvents event->time=%d", event->time);
            if (mEnabled && (mPendingEvent.distance != mPreviousProximity) ) {
				//ALOGE("orangeyang>>>>>><<<<<<1444444 ProximitySensor::readEvents mEnabled=%d", mEnabled);
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
				mPreviousProximity = mPendingEvent.distance;
            }
        } else {
            ALOGE("ProximitySensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
		//ALOGE("orangeyang>>>>>><<<<<<1444444 ProximitySensor::readEvents mEnabled=%d", mEnabled);
    }

    return numEventReceived;
}

float ProximitySensor::indexToValue(size_t index) const
{
    return index * PROXIMITY_THRESHOLD_GP2A;
}

void ProximitySensor::processEvent(int code, int value)
{
}

int ProximitySensor::getWhatFromHandle(int32_t handle)
{
	return 0;
}