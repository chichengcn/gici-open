#include <cstdlib>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>
#include <fast/fast.h>

namespace fast
{

short corner_score_test(const fast_byte* img, const int *pointer_dir, short barrier, bool *type)
{
   /*The score for a positive feature is sum of the difference between the pixels
     and the barrier if the difference is positive. Negative is similar.
     The score is the max of those two.

      B = {x | x = points on the Bresenham circle around c}
      Sp = { I(x) - t | x E B , I(x) - t > 0 }
      Sn = { t - I(x) | x E B, t - I(x) > 0}

      Score = max sum(Sp), sum(Sn)
   */

   const short pix = *img;
   const short cb = pix + barrier;
   const short c_b = pix - barrier;
   short sp = 0;
   short sn = 0;

   for(short i=0; i<16; ++i)
   {
      const short p = img[pointer_dir[i]];

      if(p > cb)
      {
         sp += p - pix;
      }
      else if(p < c_b)
      {
         sn += pix - p;
      }
   }

   if(sp > sn)
   {
      *type = true;   // dark-center-on-bright-background corner
      return sp;
   }
   else
   {
      *type = false;  // bright-center-on-dark-background corner
      return sn;
   }
}

// Non-maximum suppression in a 5x5 window (instead of the original 3x3 window).
void fast_nonmax_5x5(
      const fast_byte* img,
      int /*imgWidth*/,
      int imgHeight,
      int widthStep,
      const std::vector<fast_xy>& corners,
      short barrier,
      DarkCorners& darkCorners,
      BrightCorners& brightCorners
      )
{
   /*Create a list of integer pointer offstes, corresponding to the */
   /*direction offsets in dir[]*/
   const int numCorners = static_cast<int>(corners.size());
   int pointer_dir[16];
   int* row_start = (int*) std::malloc(imgHeight * sizeof(int));
   short* scores    = (short*) std::malloc(numCorners * sizeof(short));
   bool* types     = (bool*) std::malloc(numCorners * sizeof(bool));

   // We assume that we roughly have the same amount of dark and bright
   // corners, and that we drop more than 50% of the detected corners
   const int estNumNonmaxCorners = numCorners/4;
   darkCorners.reserve(estNumNonmaxCorners);
   brightCorners.reserve(estNumNonmaxCorners);

   int prev_row = -1;
   int i;
   int j;
   int point_above = 0;
   int point_below = 0;

   /* Directions. */
   pointer_dir[0] = 0 + 3 * widthStep;
   pointer_dir[1] = 1 + 3 * widthStep;
   pointer_dir[2] = 2 + 2 * widthStep;
   pointer_dir[3] = 3 + 1 * widthStep;
   pointer_dir[4] = 3 + 0 * widthStep;
   pointer_dir[5] = 3 + -1 * widthStep;
   pointer_dir[6] = 2 + -2 * widthStep;
   pointer_dir[7] = 1 + -3 * widthStep;
   pointer_dir[8] = 0 + -3 * widthStep;
   pointer_dir[9] = -1 + -3 * widthStep;
   pointer_dir[10] = -2 + -2 * widthStep;
   pointer_dir[11] = -3 + -1 * widthStep;
   pointer_dir[12] = -3 + 0 * widthStep;
   pointer_dir[13] = -3 + 1 * widthStep;
   pointer_dir[14] = -2 + 2 * widthStep;
   pointer_dir[15] = -1 + 3 * widthStep;


   /* Compute the score for each detected corner, and find where each row begins*/
   /* (the corners are output in raster scan order). A beginning of -1 signifies*/
   /* that there are no corners on that row.*/

   for(i=0; i < imgHeight; ++i)
      row_start[i] = -1;

   /* Calculate scores and orientations. */
   for(i=0; i < numCorners; ++i)
   {
      if(corners[i].y != prev_row)
      {
         row_start[corners[i].y] = i;
         prev_row = corners[i].y;
      }

      scores[i] = corner_score(
            img + corners[i].x + corners[i].y * widthStep,
            pointer_dir,
            barrier, types + i);
   }

   /*Point above points (roughly) to the pixel above the one of interest, if there*/
   /*is a feature there.*/

   for(i=0; i < numCorners; ++i)
   {
      short score = scores[i];
      fast_xy pos = corners[i];

      //Check left
      if(i > 0 &&
         scores[i-1] >= score &&
         corners[i-1].y == pos.y &&
         corners[i-1].x >= pos.x-2)
         continue;

      //Check left
      if(i > 1 &&
         scores[i-2] >= score &&
         corners[i-2].y == pos.y &&
         corners[i-2].x >= pos.x-2)
         continue;

      //Check right
      if(i < (numCorners - 2) &&
         scores[i+2] > score &&
         corners[i+2].y == pos.y &&
         corners[i+2].x <= pos.x+2)
         continue;

      //Check right
      if(i < (numCorners - 1) &&
         scores[i+1] > score &&
         corners[i+1].y == pos.y &&
         corners[i+1].x <= pos.x+2)
         continue;

      //Check two lines above (if this row is valid)
      if(pos.y > 1 && (point_above = row_start[pos.y - 2]) != -1)
      {
         //Make point_above point to the first of the pixels above the current point,
         //if it exists.
         for(; corners[point_above].y < pos.y-1 && corners[point_above].x < pos.x-2; point_above++)
         {}

         for(j=point_above; corners[j].y < pos.y-1 && corners[j].x <= pos.x+2; j++)
         {
            if(scores[j] >= score && corners[j].x >= pos.x-2) // implicit via loop condition "&& x <= pos.x+2)"
               goto cont;
         }
      }

      //Check directly above (if this row is valid)
      if(pos.y > 0 && (point_above = row_start[pos.y - 1]) != -1)
      {
         //Make point_above point to the first of the pixels above the current point,
         //if it exists.
         for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x-2; point_above++)
         {}

         for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x+2; j++)
         {
            if(scores[j] >= score && corners[j].x >= pos.x-2) // implicit via loop condition "&& x <= pos.x+2)"
               goto cont;
         }
      }

      //Check directly below (if there is anything below)
      if(pos.y < imgHeight-1 && (point_below = row_start[pos.y + 1]) != -1)
      {
         // Make point below point to one of the pixels belowthe current point, if it
         // exists.
         for(; point_below < numCorners && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x-2; point_below++)
         {}

         for(j=point_below; j < numCorners && corners[j].y == pos.y+1 && corners[j].x <= pos.x+2; j++)
         {
            if(scores[j] > score && corners[j].x >= pos.x-2) // implicit via loop condition "&& x <= pos.x+2"
               goto cont;
         }
      }

      //Check two lines below (if this line is valid)
      if(pos.y < imgHeight-2 && (point_below = row_start[pos.y + 2]) != -1)
      {
         // Make point below point to one of the pixels belowthe current point, if it
         // exists.
         for(; point_below < numCorners && corners[point_below].y == pos.y+2 && corners[point_below].x < pos.x-2; point_below++)
         {}

         for(j=point_below; j < numCorners && corners[j].y == pos.y+2 && corners[j].x <= pos.x+2; j++)
         {
            if(scores[j] > score && corners[j].x >= pos.x-2) // implicit via loop condition "&& x <= pos.x+2"
               goto cont;
         }
      }

      // Update the respective output vector.
      if(types[i])
      {
         darkCorners.push_back(DarkCorner(corners[i], scores[i]));
      }
      else
      {
         brightCorners.push_back(BrightCorner(corners[i], scores[i]));
      }

      cont: // skip the point, a better one is within our 5x5 search window
      ;
   }

   std::free(row_start);
   std::free(scores);
   std::free(types);
}

} // namespace CVD
