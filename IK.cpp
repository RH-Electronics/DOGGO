#include "IK.h"
#include <math.h>

IK::IK() {}

// Теперь `begin()` принимает `LegType`
void IK::begin(float offset_z, float femur, float tibia, float global_x, float global_y, float global_z, LegType legType) {
    _offset_z = offset_z;
    _femur = femur;
    _tibia = tibia;
    _global_x = global_x;
    _global_y = global_y;
    _global_z = global_z;
    _legType = legType;
}

// Локальные углы считаются без учёта глобального смещения
std::array<float, 3> IK::getLegAnglesLocal(float x, float y, float z) {
    return calculateIK(x, y, z);
}

// Теперь учитываем, где расположена нога
std::array<float, 3> IK::getLegAnglesGlobal(float x, float y, float z) {
    float x_adj = x - _global_x;
    float y_adj = y - _global_y;
    float z_adj = z - _global_z;

    // Для задних ног x инвертируется
    if (_legType == LegType::BACK_LEFT || _legType == LegType::BACK_RIGHT) {
        x_adj = -x_adj;
    }

    // Для правых ног y инвертируется
    if (_legType == LegType::FRONT_RIGHT || _legType == LegType::BACK_RIGHT) {
        y_adj = -y_adj;
    }

    return calculateIK(x_adj, y_adj, z_adj);
}

// Функция рассчета углов серво по координатам
// Получает мм, Возвращает радианы
std::array<float, 3> IK::calculateIK(float x, float y, float z) {
	  // Рассчитываем L (горизонтальная проекция ноги)
    // double L = sqrt(y*y + z*z);
    // Рассчитываем L' (длина от бедра до ступни в X-Z плоскости)
    double L_prime = sqrt((z - _offset_z)*(z - _offset_z) + x*x);
    
	  // Y-Z plane
    float theta1 = atan2(y, z);
    
    // X-Z plane
    // Угол β' (по теореме косинусов для _femur, _tibia и L')
    double cos_beta = (L_prime*L_prime + _femur*_femur - _tibia*_tibia) / (2 * _femur * L_prime);
    float beta_1 = acos(constrain(cos_beta, -1.0, 1.0));
    // Угол β'' (по тангенсу x, z - _offset_z)
    float beta_2 = atan2(x, z - _offset_z);
    // коррекция вычисления на нужный угол поворота
    float theta2 = M_PI_2 - M_PI_4/2.0 + beta_2 - beta_1;
    

    // Угол между бедром и голенью (γ) – коленный сустав
    double cos_gamma = (_femur*_femur + _tibia*_tibia - L_prime*L_prime) / (2 * _femur * _tibia);
    float theta3 = acos(constrain(cos_gamma, -1.0, 1.0));
    // коррекция вычисления на нужный угол поворота
    theta3 = M_PI_2 - M_PI_4/2.0 - theta3;
    
    return {theta3, theta2, theta1};
}
