/*******************************************************************/
/* Fichier entete pour utilisation carte advantech PCM-3712 PC-104 */
/* version : 1.0                                                   */
/* auteurs : RK                                                    */
/*******************************************************************/

#ifndef __3712__
#define __3712__

#include<asm/io.h>

#define BASE_3712 0x300
#define PCM3712_DA0 BASE_3712
#define PCM3712_DA1 (BASE_3712 +2)
#define PCM3712_SYNC (BASE_3712 +4)
#define PCM3712_OUT (BASE_3712 +5)


//int initPCM3712(void);

 void PCM3712setda0(unsigned value);

 void PCM3712setda1(unsigned value);

unsigned PCM3712VBitoValue(float vref, float V);
unsigned PCM3712VUnitoValue(float vref, float V);
#endif
