// ESP32-S3 8-lane 16-bit dot product for the Superia sharpen vertical pass.
//
// void usmVblurS3(const int16_t* ring, uint8_t* out, const int16_t* coeffs, int n)
//   ring   : n groups of 8 int16 (16 bytes each, 16-byte aligned)
//   out    : n bytes
//   coeffs : 8 int16 (16 bytes, 16-byte aligned), kernel rotated into ring-slot
//            order and zero padded
//
// Each group holds the 2R+1 horizontally blurred values for one (channel, x)
// in ring-slot order. ee.vmulas.s16.accx multiplies the eight 16-bit lanes by
// the eight coefficients and sums them, so a whole 7-tap column costs one
// vector MAC instead of seven scalar ones. The scalar reference is
// usmVblurRowT() in jpeg_filter.h; values are 0..255 and coefficients sum to
// 256, so the 32-bit accumulator cannot overflow.
    .text
    .align 4
    .global usmVblurS3
    .type usmVblurS3,@function
usmVblurS3:
    entry           a1, 16
    ee.vld.128.ip   q1, a4, 0          // 8 coefficients
    movi            a8, 128            // rounding before >> 8
    movi.n          a6, 0
    loopnez         a5, .Lloop
        ee.vld.128.ip       q0, a2, 16 // 8 taps for one pixel
        wur.accx_0          a6
        wur.accx_1          a6
        ee.vmulas.s16.accx  q0, q1
        rur.accx_0          a7
        add                 a7, a7, a8
        srai                a7, a7, 8
        s8i                 a7, a3, 0
        addi.n              a3, a3, 1
    .Lloop:
    retw.n
