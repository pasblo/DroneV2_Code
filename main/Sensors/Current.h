#ifndef CURRENT_H
#define CURRENT_H

const double PCBsensitivity = 10; //100mV per 1A
const double PCBvref = 0.3; //I dont know why

class DroneCurrent{
	public:
		void updateCurrentSensors();

		double getPCBCurrent() {return pcbCurrent; }

		//double getBattCurrent(); //With external sensing
	private:

		double pcbCurrent;
};
extern DroneCurrent DCurrent;
#endif