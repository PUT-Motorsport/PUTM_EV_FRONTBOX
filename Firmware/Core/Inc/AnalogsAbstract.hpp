/*
 * AnalogsAbstract.hpp
 *
 *  Created on: Feb 10, 2024
 *      Author: ketirange
 */

#ifndef INC_ANALOGSABSTRACT_HPP_
#define INC_ANALOGSABSTRACT_HPP_

class Analog{
private:

public:
	const unsigned number_of_samples = 50;
	uint16_t raw_value[50];
};

class SteeringPositionSensor : Analog{
private:

public:

};

class SuspensionSensors : Analog{
private:

public:

};

class TemperatureSenors : Analog{
private:

public:

};




#endif /* INC_ANALOGSABSTRACT_HPP_ */
