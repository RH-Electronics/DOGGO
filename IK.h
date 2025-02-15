#ifndef IK_H
#define IK_H

#include "Arduino.h"
#include <array>

enum class LegType { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT };

class IK {
public:
    IK(); 
    
    // Теперь передаём не только смещения, но и тип ноги
    void begin(float offset_z, float femur, float tibia, float global_x, float global_y, float global_z, LegType legType);

    std::array<float, 3> getLegAnglesLocal(float x, float y, float z);
    std::array<float, 3> getLegAnglesGlobal(float x, float y, float z);

private:
    float _offset_z;
    float _femur;
    float _tibia;
    float _global_x;
    float _global_y;
    float _global_z;
    LegType _legType;

    std::array<float, 3> calculateIK(float x, float y, float z);
};

#endif


