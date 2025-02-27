#include <Arduino.h>

class APM2RCOutput {
public:
    /* No init argument required */
    void     init();

    /* Output freq (1/period) control */
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);

    /* Output, either single channel or bulk array of channels */
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);

    /* Read back current output state, as either single channel or
     * array of channels starting at 0. */
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    uint16_t _timer_period(uint16_t speed_hz);
};
