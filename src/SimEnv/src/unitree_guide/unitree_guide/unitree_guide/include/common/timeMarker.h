/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TIMEMARKER_H
#define TIMEMARKER_H

#include <cctype>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <unistd.h>

//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}
//时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}

inline bool shouldLogAbsoluteWaitWarning(long long now){
    static long long lastWarningTime = 0;
    const char *enabledValue = std::getenv("UNITREE_LOG_WAIT_WARNINGS");
    if(enabledValue == NULL || enabledValue[0] == '\0'){
        return false;
    }
    std::string enabled(enabledValue);
    for(char &ch : enabled){
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    if(enabled == "0" || enabled == "false" || enabled == "no" || enabled == "off"){
        return false;
    }

    const char *envValue = std::getenv("UNITREE_WAIT_WARNING_INTERVAL_US");
    long long interval = 1000000;
    if(envValue != NULL && envValue[0] != '\0'){
        char *end = NULL;
        long long parsedInterval = std::strtoll(envValue, &end, 10);
        if(end != envValue && *end == '\0' && parsedInterval >= 0){
            interval = parsedInterval;
        }
    }
    if(interval == 0){
        return true;
    }
    if(now - lastWarningTime >= interval){
        lastWarningTime = now;
        return true;
    }
    return false;
}

//等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long startTime, long long waitTime){
    long long now = getSystemTime();
    long long elapsed = now - startTime;
    if(elapsed > waitTime && shouldLogAbsoluteWaitWarning(now)){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << elapsed << "us." << std::endl;
    }
    while(getSystemTime() - startTime < waitTime){
        usleep(50);
    }
}

#endif //TIMEMARKER_H
