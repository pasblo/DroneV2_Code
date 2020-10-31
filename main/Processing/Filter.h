#ifndef FILTERS_H
#define FILTERS_H

#define KALAMAN_FILTER UINT8_C(0x00)

struct filterProperties{

	//Kalaman filter variables
	double measurementError;
	double estimateError;
	double processNoise;
};

struct filter{

	//Filter used
	uint8_t filterUsed;

	//All filter properties
	struct filterProperties properties;

	//Kalaman filter variables
	double kalamanGain;
	double lastEstimate;
	double currentEstimate;
};

class Filters{
	public:
		Filters(uint8_t filterType);

		double updateEstimate(double newValue);

		void setFilterProperties(struct filterProperties newProperties) {internalFilter.properties = newProperties; }

	private:
		struct filter internalFilter;

};
#endif