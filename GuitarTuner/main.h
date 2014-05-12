#define DECIMATION_FACTOR       64
#define OUT_FREQ                32000
#define PDM_Input_Buffer_SIZE   ((OUT_FREQ/1000)*DECIMATION_FACTOR/8)
#define PCM_Output_Buffer_SIZE  (OUT_FREQ/1000)

#define SPECTRUM_BG_COLOR       Black
#define SPECTRUM_FREQ_S_kHz     32.0
#define SPECTRUM_HEIGHT         150
#define SPECTRUM_NR_SAMPLES     512
#define SPECTRUM_X_LABEL        "[kHz]"
