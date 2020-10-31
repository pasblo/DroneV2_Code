#ifndef FILTERS_H
#define FILTERS_H

typedef struct{
	double state;
	double k;
} lowPassFilter_t;

class DroneFilters{
	public:
		lowPassFilter_t createLowPassFilter();
		double applyLowPassFilter(lowPassFilter_t *filter, double input);
	private:

};
extern DroneFilters DFilters;
#endif