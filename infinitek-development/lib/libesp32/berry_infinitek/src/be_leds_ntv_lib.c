/********************************************************************
 * Berry class `Leds_ntv`
 * 
 *******************************************************************/

#include "be_constobj.h"

#ifdef USE_WS2812

#if __has_include("InfinitekLED.h")
#include "InfinitekLED.h"
#else
#include "infinitekLED.h"
#endif

extern int be_Infinitekled_call_native(bvm *vm);
extern int be_leds_blend_color(bvm *vm);
extern int be_leds_apply_bri_gamma(bvm *vm);

/* @const_object_info_begin
class be_class_Leds_ntv (scope: global, name: Leds_ntv, strings: weak) {
  _p, var

  WS2812_GRB, int(ws2812_grb)
  SK6812_GRBW, int(sk6812_grbw)
  SK6812_GRB, int(sk6812_grb)

  RMT, int(InfinitekLed_RMT)
  SPI, int(InfinitekLed_SPI)
  I2S, int(InfinitekLed_I2S)

  call_native, func(be_Infinitekled_call_native)

  blend_color, static_func(be_leds_blend_color)
  apply_bri_gamma, static_func(be_leds_apply_bri_gamma)
}
@const_object_info_end */

#include "be_fixed_be_class_Leds_ntv.h"

#endif // USE_WS2812
