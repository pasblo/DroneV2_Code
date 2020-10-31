#include "Filters.h"
DroneFilters DFilters;

lowPassFilter_t DroneFilters::createLowPassFilter(){
	lowPassFilter_t newLPF;
	newLPF.state = 0.0;
	newLPF.k = 0.0;
	return newLPF;
}

double DroneFilters::applyLowPassFilter(lowPassFilter_t *filter, double input){
	filter->state = filter->state + filter->k * (input - filter->state);
	return filter->state;
}