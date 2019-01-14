#pragma once
#ifndef _TSMUTM_H
#define _TSMUTM_H


#include <stdlib.h>
#include <math.h>
#include <iostream>
using namespace std;
#include <string.h>


#ifndef M_PI
#define M_PI        (3.14159265358979323846) 
#endif
#define HALF_PI     (1.57079632679489661923)
#define TWO_PI      (M_PI+M_PI)

#ifndef RADDEG
#define RADDEG      (M_PI/180.0)
#endif

#define EPSILON     ( 0.0000001 )
#define DOUBLE_NULL (-1.7976931348623158e+308) /* minimum double */


#define WGS84_AXIS 6378137.0
#define WGS84_RFLAT 298.257223563

#define _NOTHER	(16)
#define NM				(cnsts->name)
#define P0				(cnsts->p0)
#define SINP0			(cnsts->sinp0)	/* SIN( ABS(P0) ) */
#define COSP0			(cnsts->cosp0)	/* COS( ABS(P0) ) */
#define L0				(cnsts->l0)
#define RADIUS	  		(cnsts->axis)
#define AXIS			(cnsts->axis)
#define ECC1			(cnsts->e)
#define ECC2			(cnsts->e2)
#define WH				(cnsts->which)
#define NCNSTS			(cnsts->ncnsts)
#define K0				(cnsts->k0)
#define AZ0				(cnsts->a0)
#define STD1			(cnsts->p1)		
#define STD2			(cnsts->p2)
#define SLIM			(cnsts->glims[0])
#define WLIM			(cnsts->glims[1])
#define NLIM			(cnsts->glims[2])
#define ELIM			(cnsts->glims[3])
#define EQUATORIAL		(cnsts->which == 0)
#define POLAR			(abs(cnsts->which) == 1)


#define dtcc_allocate_ptr(n)     calloc((n),1)
#define dtcc_free_ptr(aP)        free((void*)(aP))


#define NC				(13)
#define EP2				cnsts->other[1]
#define M0				cnsts->other[2]
#define KA				cnsts->other[3]
#define MDCFS			(cnsts->other+4)
#define RCCFS			(cnsts->other+9)
#define LABEL			"Transverse Mercator"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#define WGS84_A    6378137.0               /* equatorial radius */
#define WGS84_B    6356752.3142            /* polar radius */
#define WGS84_E    0.08181919092890638     /* eccentricity */
#define WGS84_E2   ( WGS84_E * WGS84_E )
#define WGS84_1_E2 ( 1.0 - WGS84_E2 )

#define FN	(0.0)
#define FS	(10000000.0)
#define FE	(500000.0)


typedef struct {
	short  id;
	char   name[32];
	double l0, p0, sinp0, cosp0;
	double k0, a0, p1, p2;
	double axis, e, e2;
	double glims[4];
	short  which, ncnsts;
	double other[_NOTHER];
} proj_dfn;


//static
void   _convertSin2468(double* u);
double _fSin2468(double sphi, const double* u);
double poly(double x, int n, double* c);


double constN(double e);
double eccentricity2(double rf);
double check_PI(double a);
double check_180(double a);
double meridian_distance(double phi, double sphi, const double* mdcoefs);
double vertical_radius(double e, double sphi);


void set_inverse_rectifying(double e, double* fpcoefs);
void inverse_rectifying_lat(double R, double sinR, const double* fpcoefs, double* phi);
void set_meridian_distance(double e, double* mdcoefs);
void sincostan(double a, double* sina, double* cosa, double* tana);
void free_projection(const void* dfnP);


short _init_std_pars(proj_dfn* cnsts, int k, const char* name,
	double a, double rf, double lat0, double lon0);
short init_transverse_mercator(void* cnstsP, const char* opt_name, double a,
	double rf, double lat0, double lon0, double k0);
short init_utm(void* cnstsP, double a, double rf);
short utm_to_geo(const void* tcnsts, int zone, double easting,
	double northing, double* lat, double* lon);
short geo_to_utm(const void* tcnsts, double lat, double lon,
	int* izone, double* east, double* north);


int projection_limit_check(const void *cnstsP, double lat, double lon);
int geo_to_transverse_mercator(const void* cnstsP, double lat, double lon,
	double* x, double* y);
int transverse_mercator_to_geo(const void* cnstsP, double x, double y,
	double* lat, double* lon);
int transverse_mercator_limits(const void* cnstsP, double* mnlat,
	double* mnlon, double* mxlat, double* mxlon);
int utm_limits(const void* cnstsP, double* mnlat, double* mnlon,
	double* mxlat, double* mxlon);
int tsmLatLongToUTM(double lat, double lon, int *zone,
	double *easting, double *northing);
int tsmUTMToLatLong(int zone, double easting, double northing,
	double *lat, double *lon);


const void* set_transverse_mercator(const char* opt_name, double a, double rf,
	double lat0, double lon0, double k0);
const void* set_utm(double a, double rf);

#endif