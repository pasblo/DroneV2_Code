#include "Filters.h"
#include <math.h>

Filters::Filters(uint8_t filterType){
	internalFilter.filterUsed = filterType;
}

double Filters::updateEstimate(double newValue){

	//Calculating the current estimate and other variables related to the filter used
	if(internalFilter.filterUsed == KALAMAN_FILTER){
		internalFilter.kalamanGain = internalFilter.properties.estimateError / (internalFilter.properties.estimateError + internalFilter.properties.measurementError);
		internalFilter.currentEstimate = internalFilter.lastEstimate + internalFilter.kalamanGain * (newValue - internalFilter.lastEstimate);
		internalFilter.properties.estimateError = (1.0 - internalFilter.kalamanGain) * internalFilter.properties.estimateError + fabs(internalFilter.lastEstimate - internalFilter.currentEstimate) * internalFilter.properties.processNoise;
	}

	internalFilter.lastEstimate = internalFilter.currentEstimate;

	return internalFilter.currentEstimate;
}