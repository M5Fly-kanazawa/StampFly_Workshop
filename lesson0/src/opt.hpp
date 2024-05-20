#ifndef OPT_HPP
#define OPT_HPP

/*! CPP guard */
//#ifdef __cplusplus
//extern "C" {
//#endif

#include <Arduino.h>
#include <stdint.h>

typedef struct {
    uint8_t chipid;
    uint8_t dipihc;
    

} optconfig_t;

extern optconfig_t optconfig;

uint8_t opt_init(optconfig_t* optconfig);
void readMotionCount(int16_t *deltaX, int16_t *deltaY);


//#ifdef __cplusplus
//}
//#endif /* End of CPP guard */
#endif