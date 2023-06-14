#include <fast/fast.h>
#include <fast/corner_9.h>
#include <fast/faster_corner_utilities.h>

#ifndef __ARM_NEON__
#  error "This file requires NEON support. Check your compiler flags."
#else
#  include <arm_neon.h>
#endif

namespace fast
{



template <bool Aligned> void faster_corner_detect_9(const fast_byte* img, int imgWidth, int imgHeight, int widthStep,
                                                    short barrier, std::vector<fast_xy>& corners)
{
   const int stride3 = 3*widthStep;

   // The compiler refuses to reserve a register for this
   register const uint8x16_t barriers = vdupq_n_u8((unsigned char)barrier);
   // cf. neon_test.m for more information about this mask
   register const uint8x16_t magic_mask = vcombine_u8(vcreate_u8(0x8040201008040201llu),
                                                      vcreate_u8(0x8040201008040201llu));

   int xend = imgWidth - 3;
   xend -= (imgWidth-3) % 16;

   for(int y=3; y < imgHeight - 3; ++y)
   {
      for(int x=3; x < 16; ++x)
      {
         if(is_corner_9<Less>((const unsigned char*)img + y*widthStep + x, widthStep, barrier) ||
            is_corner_9<Greater>((const unsigned char*)img + y*widthStep + x, widthStep, barrier))
            corners.push_back(fast_xy(x, y));
      }

      for(int x=16; x < xend; x+=16)
      {
         const unsigned char* p = (const unsigned char*)img + y*widthStep + x;
         uint8x16_t lo, hi;
         {
            const uint8x16_t here = load_ui128<Aligned>((const uint8_t*)(p));
            lo = vqsubq_u8(here, barriers);
            hi = vqaddq_u8(here, barriers);
         }
         unsigned int ans_0, ans_8, possible;
         {
            uint8x16_t top = load_ui128<Aligned>((const uint8_t*)(p-stride3));
            uint8x16_t bottom = load_ui128<Aligned>((const uint8_t*)(p+stride3));

            CHECK_BARRIER(lo, hi, top, ans_0);
            CHECK_BARRIER(lo, hi, bottom, ans_8);
            possible = ans_0 | ans_8;
            if (!possible)
               continue;
         }

         unsigned int ans_15, ans_1;
         {
            uint8x16_t a = vld1q_u8((const uint8_t*)(p-1-stride3));
            // FIXXXME: very smart trick, since both a and c are in the same row
            //uint8x16_t c = _mm_insert_epi16(_mm_srli_si128(a,2), *(const unsigned short*)(p+15-stride3), 7);
            uint8x16_t c = vld1q_u8((const uint8_t*)(p+1-stride3));
            CHECK_BARRIER(lo, hi, a, ans_15);
            CHECK_BARRIER(lo, hi, c, ans_1);
            possible &= ans_8 | (ans_15 & ans_1);
            if (!possible)
               continue;
         }

         unsigned int ans_9, ans_7;
         {
            uint8x16_t d = vld1q_u8((const uint8_t*)(p-1+stride3));
            // FIXXXME: very smart trick, since both d and f are in the same row
            //uint8x16_t f = _mm_insert_epi16(_mm_srli_si128(d,2), *(const unsigned short*)(p+15+stride3), 7);
            uint8x16_t f = vld1q_u8((const uint8_t*)(p+1+stride3));
            CHECK_BARRIER(lo, hi, d, ans_9);
            CHECK_BARRIER(lo, hi, f, ans_7);
            possible &= ans_9 | (ans_0 & ans_1);
            possible &= ans_7 | (ans_15 & ans_0);
            if (!possible)
               continue;
         }

         unsigned int ans_12, ans_4;
         {
            uint8x16_t left = vld1q_u8((const uint8_t*)(p-3));
            uint8x16_t right = vld1q_u8((const uint8_t*)(p+3));
            CHECK_BARRIER(lo, hi, left, ans_12);
            CHECK_BARRIER(lo, hi, right, ans_4);
            possible &= ans_12 | (ans_4 & (ans_1 | ans_7));
            possible &= ans_4 | (ans_12 & (ans_9 | ans_15));
            if (!possible)
               continue;
         }

         unsigned int ans_14, ans_6;
         {
            uint8x16_t ul = vld1q_u8((const uint8_t*)(p-2-2*widthStep));
            uint8x16_t lr = vld1q_u8((const uint8_t*)(p+2+2*widthStep));
            CHECK_BARRIER(lo, hi, ul, ans_14);
            CHECK_BARRIER(lo, hi, lr, ans_6);
            {
               const unsigned int ans_6_7 = ans_6 & ans_7;
               possible &= ans_14 | (ans_6_7 & (ans_4 | (ans_8 & ans_9)));
               possible &= ans_1 | (ans_6_7) | ans_12;
            }
            {
               const unsigned int ans_14_15 = ans_14 & ans_15;
               possible &= ans_6 | (ans_14_15 & (ans_12 | (ans_0 & ans_1)));
               possible &= ans_9 | (ans_14_15) | ans_4;
            }
            if (!possible)
               continue;
         }

         unsigned int ans_10, ans_2;
         {
            uint8x16_t ll = vld1q_u8((const uint8_t*)(p-2+2*widthStep));
            uint8x16_t ur = vld1q_u8((const uint8_t*)(p+2-2*widthStep));
            CHECK_BARRIER(lo, hi, ll, ans_10);
            CHECK_BARRIER(lo, hi, ur, ans_2);
            {
               const unsigned int ans_1_2 = ans_1 & ans_2;
               possible &= ans_10 | (ans_1_2 & ((ans_0 & ans_15) | ans_4));
               possible &= ans_12 | (ans_1_2) | (ans_6 & ans_7);
            }
            {
               const unsigned int ans_9_10 = ans_9 & ans_10;
               possible &= ans_2 | (ans_9_10 & ((ans_7 & ans_8) | ans_12));
               possible &= ans_4 | (ans_9_10) | (ans_14 & ans_15);
            }
            possible &= ans_8 | ans_14 | ans_2;
            possible &= ans_0 | ans_10 | ans_6;
            if (!possible)
               continue;
         }

         unsigned int ans_13, ans_5;
         {
            uint8x16_t g = vld1q_u8((const uint8_t*)(p-3-widthStep));
            uint8x16_t l = vld1q_u8((const uint8_t*)(p+3+widthStep));
            CHECK_BARRIER(lo, hi, g, ans_13);
            CHECK_BARRIER(lo, hi, l, ans_5);
            const unsigned int ans_15_0 = ans_15 & ans_0;
            const unsigned int ans_7_8 = ans_7 & ans_8;
            {
               const unsigned int ans_12_13 = ans_12 & ans_13;
               possible &= ans_5 | (ans_12_13 & ans_14 & ((ans_15_0) | ans_10));
               possible &= ans_7 | (ans_1 & ans_2) | (ans_12_13);
               possible &= ans_2 | (ans_12_13) | (ans_7_8);
            }
            {
               const unsigned int ans_4_5 = ans_4 & ans_5;
               const unsigned int ans_9_10 = ans_9 & ans_10;
               possible &= ans_13 | (ans_4_5 & ans_6 & ((ans_7_8) | ans_2));
               possible &= ans_15 | (ans_4_5) | (ans_9_10);
               possible &= ans_10 | (ans_4_5) | (ans_15_0);
               possible &= ans_15 | (ans_9_10) | (ans_4_5);
            }

            possible &= ans_8 | (ans_13 & ans_14) | ans_2;
            possible &= ans_0 | (ans_5 & ans_6) | ans_10;
            if (!possible)
               continue;
         }


         unsigned int ans_11, ans_3;
         {
            uint8x16_t ii = vld1q_u8((const uint8_t*)(p-3+widthStep));
            uint8x16_t jj = vld1q_u8((const uint8_t*)(p+3-widthStep));
            CHECK_BARRIER(lo, hi, ii, ans_11);
            CHECK_BARRIER(lo, hi, jj, ans_3);
            {
               const unsigned int ans_2_3 = ans_2 & ans_3;
               possible &= ans_11 | (ans_2_3 & ans_4 & ((ans_0 & ans_1) | (ans_5 & ans_6)));
               possible &= ans_13 | (ans_7 & ans_8) | (ans_2_3);
               possible &= ans_8 | (ans_2_3) | (ans_13 & ans_14);
            }
            {
               const unsigned int ans_11_12 = ans_11 & ans_12;
               possible &= ans_3 | (ans_10 & ans_11_12 & ((ans_8 & ans_9) | (ans_13 & ans_14)));
               possible &= ans_1 | (ans_11_12) | (ans_6 & ans_7);
               possible &= ans_6 | (ans_0 & ans_1) | (ans_11_12);
            }
            {
               const unsigned int ans_3_4 = ans_3 & ans_4;
               possible &= ans_9 | (ans_3_4) | (ans_14 & ans_15);
               possible &= ans_14 | (ans_8 & ans_9) | (ans_3_4);
            }
            {
               const unsigned int ans_10_11 = ans_10 & ans_11;
               possible &= ans_5 | (ans_15 & ans_0) | (ans_10_11);
               possible &= ans_0 | (ans_10_11) | (ans_5 & ans_6);
            }
            if (!possible)
               continue;

         }

         possible |= (possible >> 16);

         //if(possible & 0x0f) //Does this make it faster?
         {
            if(possible & (1<< 0))
               corners.push_back(fast_xy(x + 0, y));
            if(possible & (1<< 1))
               corners.push_back(fast_xy(x + 1, y));
            if(possible & (1<< 2))
               corners.push_back(fast_xy(x + 2, y));
            if(possible & (1<< 3))
               corners.push_back(fast_xy(x + 3, y));
            if(possible & (1<< 4))
               corners.push_back(fast_xy(x + 4, y));
            if(possible & (1<< 5))
               corners.push_back(fast_xy(x + 5, y));
            if(possible & (1<< 6))
               corners.push_back(fast_xy(x + 6, y));
            if(possible & (1<< 7))
               corners.push_back(fast_xy(x + 7, y));
         }
         //if(possible & 0xf0) //Does this mak( ,  fast)r?
         {
            if(possible & (1<< 8))
               corners.push_back(fast_xy(x + 8, y));
            if(possible & (1<< 9))
               corners.push_back(fast_xy(x + 9, y));
            if(possible & (1<<10))
               corners.push_back(fast_xy(x +10, y));
            if(possible & (1<<11))
               corners.push_back(fast_xy(x +11, y));
            if(possible & (1<<12))
               corners.push_back(fast_xy(x +12, y));
            if(possible & (1<<13))
               corners.push_back(fast_xy(x +13, y));
            if(possible & (1<<14))
               corners.push_back(fast_xy(x +14, y));
            if(possible & (1<<15))
               corners.push_back(fast_xy(x +15, y));
         }
      }

      for(int x=xend; x < imgWidth - 3; x++)
      {
         if(is_corner_9<Less>((const unsigned char*)img + y*widthStep + x, widthStep, barrier) ||
            is_corner_9<Greater>((const unsigned char*)img + y*widthStep + x, widthStep, barrier))
            corners.push_back(fast_xy(x, y));
      }
   }
}

void fast_corner_detect_9_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep,
                               short barrier, std::vector<fast_xy>& corners)
{
   if(imgWidth < 7 || imgHeight < 7)
   {
      return;
   }
   else if(imgWidth < 22)
   {
      fast_corner_detect_9(img, imgWidth, imgHeight, widthStep, barrier, corners);
      return;
   }
   else
   {
      // FIXME: why was unaligned version removed?
      // corners.reserve(512);
      // if (is_aligned<16>(img) && is_aligned<16>(img + widthStep))
      //    faster_corner_detect_9<true>(img, imgWidth, imgHeight, widthStep, barrier, corners);
      // else
      faster_corner_detect_9<false>(img, imgWidth, imgHeight, widthStep, barrier, corners);
   }
}

}
