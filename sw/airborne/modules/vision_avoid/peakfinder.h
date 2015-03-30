#ifndef PEAKFINDER_H
#define PEAKFINDER_H

//extern void peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle);

extern void cv_flowSum(int *hPos, int *hFlow, int NFlows, int NCols, float *flowSum);
extern void cv_smoothAndNormalizeSum(float *flowSum, int NCols, unsigned char smootherSize, float minFlow);
extern void cv_peakFinder(float *flowSum, int NCols, float threshold, int *np, float *angle, float visualAngle);
#endif