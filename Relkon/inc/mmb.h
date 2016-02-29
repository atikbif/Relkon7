#ifndef MMB_H_
#define MMB_H_

typedef struct{
    unsigned int D1[32];
    unsigned int D2[32];
    unsigned int D3[32];
    unsigned int D4[32];
}mmb_ain;//_ADC;

typedef struct{
    unsigned int D1[32];
    unsigned int D2[32];
}mmb_dac;//_DAC;

void mmb_work(void);

#endif /* MMB_H_ */
