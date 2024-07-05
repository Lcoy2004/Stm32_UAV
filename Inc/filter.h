#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif
double Filter_low_pass_filter(double in, double *out, double hz, double t);
double Filter_limit(double data, double toplimit, double lowerlimit);
#ifdef __cplusplus
}
#endif
#endif