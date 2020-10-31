#ifndef DRONE_TIME
#define DRONE_TIME

// Micros passes away with 70 minutes, make an alarm at about 60minutes

#include "Arduino.h"

//#define USE_64BIT_TIME

// With 64 bit mode the time in us can be used up to 586549 years
#ifdef USE_64BIT_TIME
	typedef uint64_t timeUs_t;
	#define USTIMEBYTES 4
// Maybe add other bitrates tiem models, like 48bit? : TODO
// With 32 bit mode the time in us can be used up to 1.19 hours
#else
	typedef uint32_t timeUs_t;
	#define USTIMEBYTES 2
#endif

// With 32 bit mode the time in ms can be used up to 49.7 days
typedef uint32_t timeMs_t;

// Structure for all the data related to the date
typedef struct {
	uint16_t year; // 2020 - 65536
	uint8_t month; // 1 - 12
	uint16_t dayOfTheYear; // 1 - 366
	uint8_t dayOfTheMonth; // 1 - 31

} droneDate_t;

// Conversion to years
#define usInYear 31536000000000
#define usToYears(us) (round(us/usInYear))
#define usToYearsRest(us) (round(us%usInYear))

// Conversion to months
#define usInMonth 1036800000000
#define usToMonths(us) (round(us/usInMonth))
#define usToMonthsRest(us) (round(us%usInMonth))

// Conversion to days
#define usInDay 86400000000
#define usToDays(us) (round(us/usInDay))
#define usToDaysRest(us) (round(us%usInDay))

// Structure for all the data related to the time
typedef struct {
	uint8_t hour; // 0 - 23
	uint8_t minute; // 0 - 59
	uint8_t second; // 0 - 59
	uint16_t millisecond; // 0 - 999
	uint16_t microsecond; // 0 - 999
	timeMs_t rawMilliseconds;
	timeUs_t rawMicroseconds;

} droneTime_t;

// Conversion to hours
#define usInHour 3600000000
#define usToHours(us) (round(us/usInHour))
#define usToHoursRest(us) (round(us/usInHour))

// Conversion to minutes
#define usInMinute 60000000
#define usToMinutes(us) (round(us/usInMinute))
#define usToMinutesRest(us) (round(us%usInMinute))

// Conversion to seconds
#define usInSecond 1000000
#define usToSeconds(us) (round(us/usInSecond))
#define usToSecondsRest(us) (round(us/usInSecond))

// Conversion to milliseconds
#define usInMillisecond 1000
#define usToMilliseconds(us) (round(us/usInMillisecond))
#define usToMillisecondsRest(us) (round(us%usInMillisecond))

// Structure that joins the date structure and the time structure
typedef struct {
	droneDate_t date;
	droneTime_t time;

} droneDateTime_t;

static const uint16_t days[4][12] = {
    {   0,  31,     60,     91,     121,    152,    182,    213,    244,    274,    305,    335},
    { 366,  397,    425,    456,    486,    517,    547,    578,    609,    639,    670,    700},
    { 731,  762,    790,    821,    851,    882,    912,    943,    974,    1004,   1035,   1065},
    {1096,  1127,   1155,   1186,   1216,   1247,   1277,   1308,   1339,   1369,   1400,   1430}};

class DroneTime{
	public:

		static void obtainDroneTimestamp(droneDateTime_t *dtFormat, timeUs_t timestamp);

		static droneDateTime_t getActualTimeDT(); // Returns the actual time in dt format

		static timeUs_t getActualTimeUS(); // Returns the actual time in us format

		//static void updateGpsCoordinatedDate(uint32_t date); // date as ddmmyy
		//static void updateGpsCoordinatedTime(uint32_t time); // time as hhmmsscc

	private:

		//static droneTime_t microsecondsToTime(timeUs_t time);
		//static droneTime_t millisecondsToTime(timeMs_t time);

		static void microsecondsToDateTime(droneDateTime_t *dtFormat, timeUs_t time);

		static void dateTimeToMicroseconds(timeUs_t *usFormat, droneDateTime_t time);

		//static droneDate_t gpsCoordinatedDate;
		//static droneTime_t gpsCoordinatedTime;
		static timeUs_t gpsCoordinatedMicros; // Micros of the gps time starting at 2020/01/01 00:00:00:0000:0000
		static timeUs_t lastGpsCoordination;
};
#endif