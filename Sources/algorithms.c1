// uint/int?

static uint16_t rms = 2.5;
static uint32_t sum_rms_squares = 1UL * 2.5 * 2.5 * 16;

// Newton's method
// Sample number is 16
// " / 16" == " >> 4", " / 2" == " >> 1"
uint16_t Algorithm_RMS(int16_t sample)
{
  // Subtract one sample
  sum_rms_squares -= sum_rms_squares >> 4;
  // Add new rms square
  sum_rms_squares += (uint32_t) sample * sample;
  
  // Avoid divide by 0
  if (rms == 0)
    rms = 1; 
  
  // New rms
  rms = (rms + (sum_rms_squares >> 4) / rms) >> 1;
  
  return rms;
}
