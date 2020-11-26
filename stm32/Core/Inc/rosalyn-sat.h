#ifndef ROSALYN_SAT_H__
#define ROSALYN_SAT_H__

#include "nvdata.h"
#include "system.h"
#include "sx1268.h"


extern "C" SPI_HandleTypeDef hspi1;

class TRosalynSat
{
public:
  TRosalynSat();

  void Loop();
  void Setup();
  void RadioEvent( TRadioEvent const Event );
  void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin );

private:
  bool Failsafe;
  bool RadioFlag;
  TSx1268 Radio;
  TNvData NvData;
};

#endif // ROSALYN_SAT_H__
