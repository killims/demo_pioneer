/***************************************************/
/* Last Revised: 
$Id: Scan.h,v 1.1 2001/10/07 06:28:38 jminguez Exp jminguez $
*/
/***************************************************/

#ifndef Scan
#define Scan

#include "TData.h"

//#define MHOUGHp 201 /* Impares !!! para que la celda central caiga en el centro !! */
//#define MHOUGHt 201 /* Esto creo que mejorara la estabilidad del algoritmo */

//#define incHOUGHx 0.0005
//#define incHOUGHt 0.001

typedef struct{
  float Bw;
  float Br;
  float error_th;
  float error_r;
  int MaxIter;
}TSMparams;

/* typedef struct{ */
/*   int i,j,k; */
/* }Tindice; */


void InitSM(TSMparams *params);

int scanMatchingLumilios(Tsc *sistemaSalida, 
			 Tscan *ptosNew, 
			 Tscan *ptosRef,  
			 Tsc *estimacion,
			 TSMparams *params);

#endif 
