// cron-like function to invoke functions at regular intervals

#include "rover.h"
#include <vector>

std::vector<CronJob *> cronJobs;

// right now, cron jobs have to be 'void func()'
// this function makes a heap copy of the CronJob, so the caller is
// free to drop the CronJob they call with.
// the lastRun field of the CronJob passed should normally be 0,
// which means the first run of the cronJob occurs after 1 interval.
// to delay further, pass a non-zero number in the CronJob's lastRun
// field and the first run of the job will be delyed by that many ms.
void registerCronJob(CronJob *cronJob) {
    // malloc a new copy of the cronJob
    CronJob *permanentCronJob = (CronJob *)malloc(sizeof(CronJob));
    permanentCronJob->job = cronJob->job;
    permanentCronJob->interval = cronJob->interval;
    // see comment above about passing in a nonzero lastRun
    permanentCronJob->lastRun = millis() + cronJob->lastRun;
    cronJobs.push_back(permanentCronJob);
}

// call this every loop() to check and run jobs
void checkCronJobs() {
    uint32_t now = millis();
    for (std::vector<CronJob *>::iterator it = cronJobs.begin();
         it != cronJobs.end();
         ++it) {
        // with the delay function, the (now - lastRun) expression can be negative
        int32_t elapsed = ((int32_t)now - (int32_t)(*it)->lastRun);
        if (elapsed > (int32_t)(*it)->interval) {
            (*it)->job();
            (*it)->lastRun = now;
        }
    }
}
