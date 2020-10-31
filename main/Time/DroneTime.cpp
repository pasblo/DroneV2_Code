#include "DroneTime.h"

#define referenceLeapYear 2020 // The best year ever, sarcasm if you didn't get it

timeUs_t DroneTime::gpsCoordinatedMicros = 0;
timeUs_t DroneTime::lastGpsCoordination = 0;

void DroneTime::obtainDroneTimestamp(droneDateTime_t *dtFormat, timeUs_t timestamp){
	microsecondsToDateTime(dtFormat, gpsCoordinatedMicros + (timestamp - lastGpsCoordination)); 
}

void DroneTime::microsecondsToDateTime(droneDateTime_t *dtFormat, timeUs_t time){

	dtFormat->time.rawMicroseconds = time;

	dtFormat->time.microsecond = time % 1000;
	uint64_t leftoverTime = time / 1000;

	dtFormat->time.rawMilliseconds = leftoverTime;

	dtFormat->time.millisecond = leftoverTime % 1000;
	leftoverTime /= 1000;

	dtFormat->time.second = leftoverTime % 60;
	leftoverTime /= 60;

	dtFormat->time.minute = leftoverTime % 60;
	leftoverTime /= 60;

	dtFormat->time.hour = leftoverTime % 24;
	leftoverTime /= 24;

	uint32_t years = leftoverTime / (365 * 4 + 1) * 4;
    leftoverTime %= 365 * 4 + 1;

    uint32_t year;
    for (year = 3; year > 0; year--) {
        if (leftoverTime >= days[year][0]) {
            break;
        }
    }

    uint32_t month;
    for (month = 11; month > 0; month--) {
        if (leftoverTime >= days[year][month]) {
            break;
        }
    }

    dtFormat->date.year = years + year + referenceLeapYear;
    dtFormat->date.month = month + 1;
    dtFormat->date.dayOfTheMonth = leftoverTime - days[year][month] + 1;
}

/*void DroneTime::updateGpsCoordinatedDate(uint32_t date){
	gpsCoordinatedDate.year = (date % 100) + 2000;
    gpsCoordinatedDate.month = (date / 100) % 100;
    gpsCoordinatedDate.dayOfTheMonth = (date / 10000) % 100;
}

void DroneTime::updateGpsCoordinatedTime(uint32_t time){
    gpsCoordinatedTime.hour = (time / 1000000) % 100;
    gpsCoordinatedTime.minute = (time / 10000) % 100;
    gpsCoordinatedTime.second = (time / 100) % 100;
    gpsCoordinatedTime.millisecond = (time & 100) * 10;
}*/