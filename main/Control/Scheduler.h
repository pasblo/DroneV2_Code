#ifndef SCHEDULER
#define SCHEDULER7

#include "Arduino.h"

typedef enum {
    TASK_PRIORITY_IDLE = 0, // Task is only executed if there are no other tasks being executed at this tick
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 6, // Task is executed in the tick obligatory does not matter if the limitting time finishes
    TASK_PRIORITY_MAX = 255 // Task is executed first does not matter what
} sTaskPriority_e;

/*typedef struct {
	uint64_t
} sTaskExecutionTimes_t;*/

class Scheduler(){
	public:

		static void init();
};
#endif