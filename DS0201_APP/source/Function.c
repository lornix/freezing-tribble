/*******************************************************************************
File Name: Function.c   
*******************************************************************************/
/* modified for FFT by Paul Veldhuijzen van Zanten */
/* modified to use the DSP library from ST by Wilton Lacerda Silva */

#include "Function.h"
#include "Menu.h"
#include "Lcd.h"
#include "Calculate.h"
#include "stm32f10x_lib.h"
#include "HW_V1_Config.h"
#include "ASM_Function.h"
#include "string.h"
#include "Files.h"
#include "stm32_dsp.h"
#include <math.h>

//-----------------------------------------------------------------------------

volatile unsigned short  Scan_Buffer[BUFFER_SIZE]; // sampling buffer
unsigned char   Signal_Buffer[300]; // signal data buffer
unsigned char   View_Buffer[300]; // view buffer
unsigned char   Erase_Buffer[300]; // erase buffer
unsigned char   Ref_Buffer[304]; // reference waveform buffer

// -----------------------------------------------------------------------------

volatile unsigned char   ScanSegment, ScanMode;

unsigned short  X1_Counter, X2_Counter,
                Wait_CNT, t0, t0_scan, tp_to_abs, tp_to_rel;
unsigned char   Toggle, Sync, SyncSegment;

unsigned char MeFr, MeDC;   // flag variable to indicate if frequency/DC related parameters are up to date
int      Frequency, Duty, Vpp, Vrms, Vavg, Vdc, Vmin, Vmax;

unsigned const short Ks[22] =   // interpolation coefficient of the horizontal scanning interval
 {29860, 14930, 5972, 2986, 1493, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};

// ------------ For FFT ---------------------------------------------------

#define NP        256   // Number of FFT points: 64, 256 or 1024

#define X_OFFSET  3     // Offset from the right of the screen (in pixels)
#define SPLIT_X   (1 + X_OFFSET)  // left = fft, right = wave

int FFT_in[NP]; // input array
int FFT_out[NP + 2];  // one extra element
unsigned const char F_Unit_DUPLICATE[4][4] = {"Hz ", "Hz ", "kHz", "MHz"}; // REMOVE: already present in menu.c

void Calculate_FFT( void );
void Erase_FFT(void);
void Draw_FFT(void);

/* From the ST DSP library */
void powerMag(int nfill, char* strPara);
void onesided(int nfill);

/* Multiplication factors for Hanning window
 * Hanning[i] = 0.5 - 0.5 * cos(2 * PI * i/(64 - 1))
 * Use only half (The windows are symmetric) */		
unsigned const short Hanning64[64] =
    {
      0x0000, 0x00A3, 0x028A, 0x05B0, 0x0A0D, 0x0F96, 0x163E, 0x1DF2,
      0x26A0, 0x3031, 0x3A8E, 0x459B, 0x513D, 0x5D55, 0x69C6, 0x766F,
      0x8331, 0x8FEA, 0x9C7C, 0xA8C4, 0xB4A5, 0xC000, 0xCAB8, 0xD4B2,
      0xDDD5, 0xE608, 0xED39, 0xF353, 0xF848, 0xFC0B, 0xFE92, 0xFFD7,
    };

unsigned const short Hanning256[256] =
   {
      0x0000, 0x000A, 0x0028, 0x0059, 0x009F, 0x00F8, 0x0165, 0x01E6, 
      0x027B, 0x0322, 0x03DE, 0x04AC, 0x058E, 0x0683, 0x078A, 0x08A5, 
      0x09D2, 0x0B11, 0x0C62, 0x0DC6, 0x0F3B, 0x10C2, 0x125A, 0x1403, 
      0x15BD, 0x1787, 0x1961, 0x1B4C, 0x1D46, 0x1F50, 0x2168, 0x238F, 
      0x25C5, 0x2809, 0x2A5A, 0x2CB9, 0x2F24, 0x319C, 0x3421, 0x36B1, 
      0x394C, 0x3BF3, 0x3EA4, 0x415F, 0x4424, 0x46F2, 0x49C9, 0x4CA9, 
      0x4F90, 0x527F, 0x5575, 0x5872, 0x5B75, 0x5E7E, 0x618B, 0x649E, 
      0x67B5, 0x6ACF, 0x6DED, 0x710E, 0x7431, 0x7755, 0x7A7B, 0x7DA2, 
      0x80CA, 0x83F1, 0x8718, 0x8A3D, 0x8D61, 0x9083, 0x93A2, 0x96BF, 
      0x99D7, 0x9CEC, 0x9FFC, 0xA307, 0xA60D, 0xA90D, 0xAC07, 0xAEF9, 
      0xB1E5, 0xB4C8, 0xB7A3, 0xBA76, 0xBD40, 0xC000, 0xC2B6, 0xC562, 
      0xC803, 0xCA99, 0xCD23, 0xCFA1, 0xD213, 0xD478, 0xD6D0, 0xD91B, 
      0xDB58, 0xDD86, 0xDFA6, 0xE1B7, 0xE3B9, 0xE5AB, 0xE78E, 0xE960, 
      0xEB22, 0xECD4, 0xEE74, 0xF004, 0xF182, 0xF2EE, 0xF449, 0xF591, 
      0xF6C7, 0xF7EB, 0xF8FC, 0xF9FA, 0xFAE5, 0xFBBD, 0xFC82, 0xFD34, 
      0xFDD2, 0xFE5D, 0xFED4, 0xFF37, 0xFF86, 0xFFC2, 0xFFEA, 0xFFFE
      };

unsigned const short Hanning1024[1024] =
{
     0x0000, 0x0001, 0x0002, 0x0006, 0x000A, 0x000F, 0x0016, 0x001E, 0x0028, 0x0032, 
     0x003E, 0x004B, 0x0059, 0x0068, 0x0079, 0x008B, 0x009E, 0x00B2, 0x00C8, 0x00DF, 
     0x00F7, 0x0110, 0x012B, 0x0146, 0x0163, 0x0182, 0x01A1, 0x01C2, 0x01E3, 0x0206, 
     0x022B, 0x0250, 0x0277, 0x029F, 0x02C8, 0x02F2, 0x031E, 0x034A, 0x0378, 0x03A8, 
     0x03D8, 0x0409, 0x043C, 0x0470, 0x04A5, 0x04DC, 0x0513, 0x054C, 0x0586, 0x05C1, 
     0x05FD, 0x063A, 0x0679, 0x06B9, 0x06FA, 0x073C, 0x077F, 0x07C4, 0x0809, 0x0850, 
     0x0898, 0x08E1, 0x092B, 0x0977, 0x09C3, 0x0A11, 0x0A60, 0x0AB0, 0x0B01, 0x0B53, 
     0x0BA6, 0x0BFB, 0x0C50, 0x0CA7, 0x0CFF, 0x0D58, 0x0DB2, 0x0E0D, 0x0E69, 0x0EC6, 
     0x0F25, 0x0F84, 0x0FE5, 0x1046, 0x10A9, 0x110D, 0x1172, 0x11D8, 0x123F, 0x12A7, 
     0x1310, 0x137A, 0x13E6, 0x1452, 0x14BF, 0x152D, 0x159D, 0x160D, 0x167F, 0x16F1, 
     0x1765, 0x17D9, 0x184F, 0x18C5, 0x193D, 0x19B5, 0x1A2F, 0x1AA9, 0x1B25, 0x1BA1, 
     0x1C1E, 0x1C9D, 0x1D1C, 0x1D9C, 0x1E1D, 0x1EA0, 0x1F23, 0x1FA7, 0x202C, 0x20B2, 
     0x2138, 0x21C0, 0x2249, 0x22D2, 0x235D, 0x23E8, 0x2474, 0x2501, 0x258F, 0x261E, 
     0x26AE, 0x273E, 0x27D0, 0x2862, 0x28F5, 0x2989, 0x2A1E, 0x2AB4, 0x2B4A, 0x2BE2, 
     0x2C7A, 0x2D13, 0x2DAC, 0x2E47, 0x2EE2, 0x2F7E, 0x301B, 0x30B9, 0x3157, 0x31F6, 
     0x3296, 0x3337, 0x33D8, 0x347A, 0x351D, 0x35C1, 0x3665, 0x370A, 0x37B0, 0x3856, 
     0x38FD, 0x39A5, 0x3A4D, 0x3AF7, 0x3BA0, 0x3C4B, 0x3CF6, 0x3DA2, 0x3E4E, 0x3EFB, 
     0x3FA9, 0x4057, 0x4106, 0x41B6, 0x4266, 0x4316, 0x43C8, 0x447A, 0x452C, 0x45DF, 
     0x4693, 0x4747, 0x47FC, 0x48B1, 0x4967, 0x4A1D, 0x4AD4, 0x4B8B, 0x4C43, 0x4CFB, 
     0x4DB4, 0x4E6D, 0x4F27, 0x4FE1, 0x509C, 0x5157, 0x5213, 0x52CF, 0x538C, 0x5448, 
     0x5506, 0x55C4, 0x5682, 0x5740, 0x57FF, 0x58BF, 0x597E, 0x5A3F, 0x5AFF, 0x5BC0, 
     0x5C81, 0x5D43, 0x5E05, 0x5EC7, 0x5F89, 0x604C, 0x610F, 0x61D3, 0x6296, 0x635A, 
     0x641F, 0x64E3, 0x65A8, 0x666D, 0x6732, 0x67F8, 0x68BE, 0x6984, 0x6A4A, 0x6B11, 
     0x6BD7, 0x6C9E, 0x6D65, 0x6E2C, 0x6EF4, 0x6FBB, 0x7083, 0x714B, 0x7213, 0x72DB, 
     0x73A3, 0x746B, 0x7534, 0x75FD, 0x76C5, 0x778E, 0x7857, 0x7920, 0x79E9, 0x7AB2, 
     0x7B7B, 0x7C44, 0x7D0D, 0x7DD7, 0x7EA0, 0x7F69, 0x8032, 0x80FC, 0x81C5, 0x828E, 
     0x8357, 0x8420, 0x84EA, 0x85B3, 0x867C, 0x8745, 0x880E, 0x88D6, 0x899F, 0x8A68, 
     0x8B30, 0x8BF9, 0x8CC1, 0x8D89, 0x8E51, 0x8F19, 0x8FE1, 0x90A9, 0x9170, 0x9237, 
     0x92FE, 0x93C5, 0x948C, 0x9553, 0x9619, 0x96DF, 0x97A5, 0x986B, 0x9930, 0x99F5, 
     0x9ABA, 0x9B7F, 0x9C43, 0x9D08, 0x9DCC, 0x9E8F, 0x9F52, 0xA015, 0xA0D8, 0xA19A, 
     0xA25C, 0xA31E, 0xA3E0, 0xA4A1, 0xA561, 0xA622, 0xA6E1, 0xA7A1, 0xA860, 0xA91F, 
     0xA9DD, 0xAA9B, 0xAB59, 0xAC16, 0xACD3, 0xAD8F, 0xAE4B, 0xAF06, 0xAFC1, 0xB07C, 
     0xB136, 0xB1EF, 0xB2A8, 0xB361, 0xB419, 0xB4D1, 0xB588, 0xB63E, 0xB6F4, 0xB7AA, 
     0xB85F, 0xB913, 0xB9C7, 0xBA7A, 0xBB2D, 0xBBDF, 0xBC91, 0xBD42, 0xBDF2, 0xBEA2, 
     0xBF51, 0xC000, 0xC0AE, 0xC15B, 0xC208, 0xC2B4, 0xC360, 0xC40A, 0xC4B5, 0xC55E, 
     0xC607, 0xC6AF, 0xC756, 0xC7FD, 0xC8A3, 0xC949, 0xC9ED, 0xCA91, 0xCB34, 0xCBD7, 
     0xCC79, 0xCD1A, 0xCDBA, 0xCE59, 0xCEF8, 0xCF96, 0xD034, 0xD0D0, 0xD16C, 0xD207, 
     0xD2A1, 0xD33A, 0xD3D2, 0xD46A, 0xD501, 0xD597, 0xD62C, 0xD6C1, 0xD754, 0xD7E7, 
     0xD879, 0xD90A, 0xD99A, 0xDA29, 0xDAB8, 0xDB45, 0xDBD2, 0xDC5E, 0xDCE9, 0xDD73, 
     0xDDFC, 0xDE84, 0xDF0B, 0xDF92, 0xE017, 0xE09B, 0xE11F, 0xE1A2, 0xE223, 0xE2A4, 
     0xE324, 0xE3A3, 0xE420, 0xE49D, 0xE519, 0xE594, 0xE60E, 0xE687, 0xE6FF, 0xE776, 
     0xE7EC, 0xE861, 0xE8D5, 0xE948, 0xE9BA, 0xEA2B, 0xEA9B, 0xEB0A, 0xEB78, 0xEBE4, 
     0xEC50, 0xECBB, 0xED25, 0xED8D, 0xEDF5, 0xEE5B, 0xEEC1, 0xEF25, 0xEF88, 0xEFEB, 
     0xF04C, 0xF0AC, 0xF10B, 0xF169, 0xF1C5, 0xF221, 0xF27C, 0xF2D5, 0xF32D, 0xF385, 
     0xF3DB, 0xF430, 0xF484, 0xF4D6, 0xF528, 0xF579, 0xF5C8, 0xF616, 0xF663, 0xF6AF, 
     0xF6FA, 0xF744, 0xF78C, 0xF7D4, 0xF81A, 0xF85F, 0xF8A3, 0xF8E5, 0xF927, 0xF967, 
     0xF9A6, 0xF9E4, 0xFA21, 0xFA5D, 0xFA97, 0xFAD1, 0xFB09, 0xFB40, 0xFB75, 0xFBAA, 
     0xFBDD, 0xFC0F, 0xFC40, 0xFC70, 0xFC9F, 0xFCCC, 0xFCF8, 0xFD23, 0xFD4D, 0xFD75, 
     0xFD9D, 0xFDC3, 0xFDE8, 0xFE0B, 0xFE2E, 0xFE4F, 0xFE6F, 0xFE8E, 0xFEAB, 0xFEC8, 
     0xFEE3, 0xFEFD, 0xFF15, 0xFF2D, 0xFF43, 0xFF58, 0xFF6C, 0xFF7E, 0xFF8F, 0xFF9F, 
     0xFFAE, 0xFFBC, 0xFFC8, 0xFFD3, 0xFFDD, 0xFFE6, 0xFFED, 0xFFF3, 0xFFF8, 0xFFFC, 
     0xFFFF, 0xFFFF};

/*******************************************************************************
 Function Name : Mark_Trig
 Description : mark the trigger point and setup for post scan
 Para :     trigger point (absolute position within buffer)
*******************************************************************************/
void     Mark_Trig(unsigned short tp, unsigned char stop_scan)
{
    SyncSegment = tp / SEGMENT_SIZE;
    Sync = 2; // indicate trigger marked
    if (stop_scan) ScanMode = 3; // start DMA post fetch

    if (SyncSegment == 0) {
      tp_to_abs = SEGMENT_SIZE * 2;
      tp_to_rel = SEGMENT_SIZE;
    } else if (SyncSegment == 1) {
      tp_to_abs = 0;
      tp_to_rel = 0;
    } else {
      tp_to_abs = SEGMENT_SIZE;
      tp_to_rel = SEGMENT_SIZE * 2;
    }
    
    t0 = (tp + tp_to_rel);
    if (t0 >= BUFFER_SIZE) t0 -= BUFFER_SIZE;
    
    // reset display x pointers
    X1_Counter = X2_Counter = 0;
}

/*******************************************************************************
 Function Name : GetScanPos
 Description : find and reurn index of current point in scan buffer
*******************************************************************************/
unsigned short GetScanPos(void)
{
   unsigned short t;
   unsigned char  ss;

   do {
     ss = ScanSegment;
     t = (ss + 1) * SEGMENT_SIZE;
     t = t - DMA_CNDTR1;
   } while (ss != ScanSegment);
   if (t >= BUFFER_SIZE) t -= BUFFER_SIZE;
   return t;
}

/*******************************************************************************
 Function Name : Find_Trig
 Description :find the first point in sampling buffer which meets trigger condition 
*******************************************************************************/
void     Find_Trig(void)
{
   int            th1, th2;
   bool           trig = FALSE;
   unsigned short t = GetScanPos(); // get current absolute position in scan buffer

   // calculate trigger thresholds
   th1 = SigToAdc(Item_Index[VT] - Item_Index[TRIG_SENSITIVITY]);
   th2 = SigToAdc(Item_Index[VT] + Item_Index[TRIG_SENSITIVITY]);

   // search for trigger
   while (t0 != t) {
      if (Item_Index[TRIG_SLOPE] == 0)
      { 
         if ((Sync == 0) && (Scan_Buffer[t0] > th1))
           Sync = 1;  // below trigger threshold

         if ((Sync == 1) && (Scan_Buffer[t0] < th2))
           trig = TRUE; // above trigger threshold
      } else {  // trigger slope is descending edge
         if ((Sync == 0) && (Scan_Buffer[t0] <= th2))
           Sync = 1;  // above trigger threshold

         if ((Sync == 1) && (Scan_Buffer[t0] >= th1))
           trig = TRUE; // below trigger threshold
      }
 
      if (trig) {
        Mark_Trig(t0, 1);
        break;
      }
      
      t0 = (t0 + 1);
      if (t0 >= BUFFER_SIZE) t0 = 0;
   }
}

/*******************************************************************************
 Function Name : AdcToSig
 Description : scale ADC reading to screen
*******************************************************************************/
int AdcToSig(int adc)
{
  int sig;

  sig = Km[Item_Index[Y_SENSITIVITY]] * (2048 - adc) / 4096 + 120 + (Item_Index[CALIBRATE_OFFSET] - 100);
  sig += sig * (Item_Index[CALIBRATE_RANGE] - 100) / 200;
  return sig;
}

/*******************************************************************************
 Function Name : SigToAdc
 Description : scale signal to ADC value
*******************************************************************************/
int SigToAdc(int sig)
{
  int adc;
 
  adc = 2048 - (sig - 120 - (Item_Index[CALIBRATE_OFFSET] - 100)) * 4096 / Km[Item_Index[Y_SENSITIVITY]];
  sig -= sig * (Item_Index[CALIBRATE_RANGE] - 100) / 200;
  return adc;
}

/*******************************************************************************
 Function Name : Process_Wave
 Description : process sampling buffer and put results in signal buffer
*******************************************************************************/
void    Process_Wave(void)
{
   int             p, q;
   int             Vs;
   unsigned short  t;  // relative position of last capture

   if (ScanMode == 0) { // capture complete
     t = BUFFER_SIZE;
     Sync = 4;  // no more data to process
   } else {
     t = GetScanPos() + tp_to_rel;  // get current relative position in scan buffer
     if (t >= BUFFER_SIZE) t -= BUFFER_SIZE;
   }

   p = t0;
   if ((ScanMode != 1) && (ScanMode != 2))  // ignore trigger/x offset when in continues scan mode
     p += BUFFER_SIZE - Item_Index[TP] - 150;

   for (; X2_Counter < X_SIZE; X2_Counter++)
   {
      q = p + (X2_Counter * 1024) / Ks[Item_Index[X_SENSITIVITY]];
      if (q < 0)
      {
        Erase_Wave(X1_Counter, X2_Counter + 1);
        X1_Counter = X2_Counter;
        continue;
      }

      if (q >= t)
         break;

      // find absolute q position in buffer      
      q = (q + tp_to_abs);
      if (q >= BUFFER_SIZE) q -= BUFFER_SIZE;

      Vs = AdcToSig(Scan_Buffer[q]);  // scale to screen
      if (Vs > MAX_Y) Vs = MAX_Y;
      else if (Vs < MIN_Y) Vs = MIN_Y;
      Signal_Buffer[X2_Counter] = Vs;
      
      Sync = 3; // new values in signal buffer
   }

   if ((Sync >= 3) && (X2_Counter < X_SIZE)) {
      if ((ScanMode == 1) || (ScanMode == 2))
        Erase_Wave(X2_Counter, X2_Counter + 25); // leave 1 div gap in scan mode
      else if ((Sync >= 4) || (Item_Index[X_SENSITIVITY] >= 12))
        Erase_Wave(X2_Counter, X_SIZE);
   }
}

/*******************************************************************************
 Function Name : Erase_Reference
 Description : Erase reference waveform
*******************************************************************************/
void        Erase_Reference(void)
{
   unsigned short j, i;
   
   for (j = i = 0; j < X_SIZE; j++) {
      Erase_SEG(j, Ref_Buffer[i], Ref_Buffer[j], REF_COLOR); // erase reference wave
      i = j;
   }
}

/*******************************************************************************
 Function Name : Draw_Reference
 Description : Draw reference waveform
*******************************************************************************/
void        Draw_Reference(void)
{
   unsigned short j, i;
   
   for (j = i = 0; j < X_SIZE; j++) {
      Draw_SEG(j, Ref_Buffer[i], Ref_Buffer[j], REF_COLOR); // erase reference wave
      i = j;
   }
}

/*******************************************************************************
 Function Name : Erase_Wave
 Description : Erase waveform from t1 to t2 - 1 inclusive
*******************************************************************************/
void        Erase_Wave(unsigned short t1, unsigned short t2)
{
   unsigned short j;
   
   if (t2 > X_SIZE) t2 = X_SIZE;
   for (j = t1; j < t2; j++)
   {
     if ( j > SPLIT_X ) // Do not display left of screen (FFT)
     {
      Erase_SEG(j, Erase_Buffer[j], View_Buffer[j], WAV_COLOR); // erase previous signal
      View_Buffer[j] = 0xff;
      Erase_Buffer[j] = 0xff;
     }
   }
   MeFr = MeDC = 0;
}

/*******************************************************************************
 Function Name : Redraw_Wave
 Description : Process signal buffer and redraw waveform
*******************************************************************************/
void        Redraw_Wave(void)
{
  if ((ScanMode == 0) || (Sync >= 2)) {
    X1_Counter = X2_Counter = 0;
    Sync = 2; // process wave
  } else Erase_Wave(0, X_SIZE);
}

/*******************************************************************************
 Function Name : Draw_Wave
 Description :erase the reference and view wave then draw new waveform
*******************************************************************************/
void        Draw_Wave(void)
{
   unsigned short  i = (X1_Counter > 0) ? X1_Counter - 1 : 0;

   for (; X1_Counter < X2_Counter; X1_Counter++)
   {
     if ( X1_Counter > SPLIT_X ) // Do not display left of screen (FFT)
     {
      Erase_SEG(X1_Counter, Erase_Buffer[X1_Counter], View_Buffer[X1_Counter], WAV_COLOR); // erase previous signal
      Draw_SEG(X1_Counter, Signal_Buffer[i], Signal_Buffer[X1_Counter], WAV_COLOR); // draw new signal
      View_Buffer[X1_Counter] = Signal_Buffer[X1_Counter];
      Erase_Buffer[X1_Counter] = Signal_Buffer[i];
     }
     i = X1_Counter;
   }

   if (Sync == 4) // scan complete
   { 
      Measure_Wave();   // do waveform measurements
      Sync = 0;
   } else (Sync = 2); // further processing needed
}


/*******************************************************************************
 Function Name : Stop_Wave
 Description : stop DMA ADC sampling forcefully and clear screen
               use for mode change
*******************************************************************************/
void    Stop_Wave(void)
{
  ADC_Stop();
  Erase_Wave(0, X_SIZE);
  Sync = 0;
}

/*******************************************************************************
 Function Name : Scan_Wave
 Description : process the synchronous scan,display waveform based on the synchronous mode
 NOTE: there are four modes, AUTO, NORM, SING, FIT 
*******************************************************************************/
void    Scan_Wave(void) 
{
    //--------------------Common-------------------------
    if (Item_Index[RUNNING_STATUS] == RUN)
    {
       if ((Sync <= 1) && (ScanMode == 0)) { // we must restart sampling
          Sync = 0;
          t0 = SEGMENT_SIZE; // start to look for trigger in first position, second segment
          t0_scan = 0;
          ADC_Start();
          Refresh_Counter = 100;  // keep waveform for 100ms
       }
    }
    
   //--------------------SCAN-----------------------------
    if (Item_Index[SYNC_MODE] == 3) { // 3:SCAN
      if ((Sync <= 1) && (ScanMode >= 1)) {
         Mark_Trig(t0_scan, 0);
      }
      if (Sync == 2)
         Process_Wave(); 
  
      if (Sync >= 3) {
         Draw_Wave(); 
         if ((ScanMode == 1) || (ScanMode == 2)) {
           if (X2_Counter >= X_SIZE) {
             t0_scan += X_SIZE;   // advance one frame
             if (t0_scan >= BUFFER_SIZE) t0_scan -= BUFFER_SIZE;
             Sync = 0;
           }
         }
      }

      if ((Item_Index[RUNNING_STATUS] != RUN) && ((ScanMode == 1) || (ScanMode == 2))) {
        t0_scan += 150;
        if (t0_scan >= BUFFER_SIZE) t0_scan -= BUFFER_SIZE;
        Mark_Trig(t0_scan, 1); // force trig at screen center position
        Draw_Ti_Line(Tp, ERASE, CH2_COLOR);
        Draw_Ti_Mark(Tp, ERASE, CH2_COLOR);
        Erase_Trig_Pos();
        Item_Index[TP] = BUFFER_SIZE; // reset X position to center of screen
        Update[CURSORS] = 1;
     }
   }
   
   //--------------------AUTO, FIT------------------------
   if ((Item_Index[SYNC_MODE] == 0) || (Item_Index[SYNC_MODE] == 4))// 0:AUTO, 5:FIT
   {
      if ((Sync <= 1) && (ScanMode >= 2)) {
       if (Refresh_Counter == 0) {  // force trigger after 100ms
         Mark_Trig(GetScanPos(), 1); // trig at current scan position
       } else
         Find_Trig(); 
      }
      if (Sync == 2)
         Process_Wave(); 
  
      if ((Sync >= 3) && (Refresh_Counter == 0))
         Draw_Wave(); 
   }

   //--------------------NORM-----------------------------
   if (Item_Index[SYNC_MODE] == 1)//1 :NORM
   {
      if ((Sync <= 1) && (ScanMode >= 2))
        Find_Trig(); 

      if (Sync == 2)
       Process_Wave(); 

      if ((Sync >= 3) && (Refresh_Counter == 0))
         Draw_Wave(); 
   }

   //-------------------- SING--------------------------
   if (Item_Index[SYNC_MODE] == 2)//2 :SING
   {
      if ((Sync <= 1) && (ScanMode >= 2))
         Find_Trig(); 

      if (Sync == 2)
         Process_Wave();

      if ((Sync >= 3) && (Refresh_Counter == 0)) {
         Draw_Wave(); 
         Item_Index[RUNNING_STATUS] = HOLD;
         Update[SYNC_MODE] = 1;
      }
   }
}

/*******************************************************************************
 Function Name : Measure_Wave
 Description :  calculate the frequency,cycle,duty, Vpp(peak-to-peak value),Vavg(average of alternating voltage),
                Vrms (effective value of alternating voltage), DC voltage
*******************************************************************************/
void      Measure_Wave(void)
{
   unsigned short  i, j, t_max = 0xffff, t_min = 0, Trig = 0;
   unsigned int    Threshold0, Threshold1, Threshold2, Threshold3;
   int             Vk = 0, Vn, Vm, Vp, Vq, Tmp1, Tmp2;
   unsigned short  Edge, First_Edge, Last_Edge;
   
   Edge = 0,
   First_Edge = 0;
   Last_Edge = 0;
   Threshold0 = SigToAdc(Item_Index[V0]);
   Threshold1 = SigToAdc(Item_Index[VT] - Item_Index[TRIG_SENSITIVITY]);
   Threshold2 = SigToAdc(Item_Index[VT] + Item_Index[TRIG_SENSITIVITY]);
   Threshold3 = SigToAdc(Item_Index[VT]);
   
   for (i = 0; i < BUFFER_SIZE; i++)
   {
      j = (i + tp_to_abs);
      if (j >= BUFFER_SIZE) j -= BUFFER_SIZE;

      Vk += Scan_Buffer[j];
      if ((i >= t0) && (i < t0 + 300))
      {
         if (Scan_Buffer[j] < t_max)
            t_max = Scan_Buffer[j];
         if (Scan_Buffer[j] > t_min)
            t_min = Scan_Buffer[j];
      }
      if ((Trig == 0) && (Scan_Buffer[j] > Threshold1))
         Trig = 1; 

      if ((Trig == 1) && (Scan_Buffer[j] < Threshold2))
      {
         Trig = 0;
         if (First_Edge == 0)
         {
            First_Edge = i;
            Last_Edge = i;
            Edge = 0;
         } else {
            Last_Edge = i;
            Edge++;
         }
      }
   }
   Vk = Vk / BUFFER_SIZE;

   MeFr = 0;
   if (Edge != 0)
   {
      MeFr = 1; // true
      Vm = 0;
      Vq = 0;
      Vn = 0;
      for (i = First_Edge; i < Last_Edge; i++)
      { 
         j = (i + tp_to_abs);
         if (j >= BUFFER_SIZE) j -= BUFFER_SIZE;
      
         if (Scan_Buffer[j] < Threshold3) Vm++;
         
         Vp = (4096 - Scan_Buffer[j]) - Threshold0;
         Vn += (Vp * Vp) / 8; 

         if (Scan_Buffer[j] < Threshold0)
            Vq += (Threshold0 - Scan_Buffer[j]);
         else
            Vq += (Scan_Buffer[j] - Threshold0);
      }
      if (Item_Index[X_SENSITIVITY] < 5)
        Frequency = (Edge * (1000000000 / 1167) / (Last_Edge - First_Edge)) * 1000; // ??? suspicious, check
      else
        Frequency = (Edge * (1000000000 / T_Scale[Item_Index[X_SENSITIVITY]]) / (Last_Edge - First_Edge)) * 1000;
      
      //Cycle = ((Last_Edge - First_Edge) * T_Scale[Item_Index[X_SENSITIVITY]]) / Edge;
      //Tlow = ((Last_Edge - First_Edge - Vm) * T_Scale[Item_Index[X_SENSITIVITY]]) / Edge;
      //Thigh = (Vm * T_Scale[Item_Index[X_SENSITIVITY]]) / Edge;
      Duty = 100000 * Vm / (Last_Edge - First_Edge);
      
      Vrms = ((Km[Item_Index[Y_SENSITIVITY]] * sqrt32(Vn / (Last_Edge - First_Edge) * 8)) / 4096)
              * V_Scale[Item_Index[Y_SENSITIVITY]];
      Vrms = Vrms + Vrms * (Item_Index[CALIBRATE_RANGE] - 100) / 200;
      Vavg = ((Km[Item_Index[Y_SENSITIVITY]] * (Vq / (Last_Edge - First_Edge))) / 4096)
              * V_Scale[Item_Index[Y_SENSITIVITY]];
      Vavg = Vavg + Vavg * (Item_Index[CALIBRATE_RANGE] - 100) / 200;
      
   }
    
   if (t_min < t_max) t_min = t_max;

   Tmp1 = AdcToSig(t_min);
   Vmin = (Tmp1 - Item_Index[V0]) * V_Scale[Item_Index[Y_SENSITIVITY]];
   Tmp2 = AdcToSig(t_max);
   Vmax = (Tmp2 - Item_Index[V0]) * V_Scale[Item_Index[Y_SENSITIVITY]];
   Tmp1 = Tmp2 - Tmp1;
   Vpp = Tmp1 * V_Scale[Item_Index[Y_SENSITIVITY]];
   Tmp2 = AdcToSig(Vk);
   Vdc = (Tmp2 - Item_Index[V0]) * V_Scale[Item_Index[Y_SENSITIVITY]];

   MeDC = 1;
   Update[MEASURE_KIND] = 1;
 
   if ((Item_Index[SYNC_MODE] == 4) && (Wait_CNT == 0))
   {
      if ((Edge < 20) && (Item_Index[X_SENSITIVITY] < 14))
      {
         Item_Index[X_SENSITIVITY]++;
         Update[X_SENSITIVITY] = 1;
      }
      if ((Edge > 60) && (Item_Index[X_SENSITIVITY] > 0))
      {
         Item_Index[X_SENSITIVITY]--;
         Update[X_SENSITIVITY] = 1;

      }
      if ((Tmp1 < 50) && (Item_Index[INPUT_ATTENUATOR] == 0) && (Item_Index[Y_SENSITIVITY] > 0))
      {
         Item_Index[Y_SENSITIVITY]--;
         Update[Y_SENSITIVITY] = 1;
      }
      if ((Tmp1 < 50) && (Item_Index[INPUT_ATTENUATOR] == 1) && (Item_Index[Y_SENSITIVITY] > 11))
      {
         Item_Index[Y_SENSITIVITY]--;
         Update[Y_SENSITIVITY] = 1;
      }
      if ((Tmp1 > 150) && (Item_Index[INPUT_ATTENUATOR] == 0) && (Item_Index[Y_SENSITIVITY] < 9))
      {
         Item_Index[Y_SENSITIVITY]++;
         Update[Y_SENSITIVITY] = 1;
      }
      if ((Tmp1 > 150) && (Item_Index[INPUT_ATTENUATOR] == 1) && (Item_Index[Y_SENSITIVITY] < 18))
      {
         Item_Index[Y_SENSITIVITY]++;
         Update[Y_SENSITIVITY] = 1;
      }
      Erase_Sensitivity();
      Item_Index[VT] = Tmp2 - Item_Index[VS] * 2;
      Update[TRIG_LEVEL] = 1;
      Update[CURSORS] = 1;
   }
   if (Wait_CNT > 5) Wait_CNT = 0;
   else Wait_CNT++;

   Calculate_FFT();
}


/*******************************************************************************
 Function Name : Calculate_FFT
 Description :  compute FFT with 64, 256 or 1024 bins

*******************************************************************************/
void Calculate_FFT( void )
{
  unsigned short i, iback;
  int FFT_Peakfreq, nyquist_freq;
  I32STR_RES res; // Needed for string conversion
  unsigned short factor;
  
  // STEP 0 : Erase previous fft screen
  Erase_FFT();

  
  // STEP 1 : Get data from Scan_Buffer
  for ( i = 0; i < NP; i++ )
  {
    FFT_in[ i ] = Scan_Buffer[ t0 + i ]; // No need to scale 12-bit input value
  }

  // STEP 2 : Hanning window function
  // The multiplication factors are symmetrical
  iback = NP - 1;
  for ( i = 0; i < NP / 2; i++)
  {
    // Hanning value between 0..65535 instead of 0..1
    if (NP == 64) factor = Hanning64[i]; 
    if (NP == 256) factor = Hanning256[i]; 
    if (NP == 1024) factor = Hanning1024[i];     
    FFT_in[i]     = ( FFT_in[i]     * factor ) >> 16;
    FFT_in[iback] = ( FFT_in[iback] * factor ) >> 16;
    iback--;
  }

  // STEP 3 : Call the ST library 
  for (i=0; i < NP; i++)
  {
    FFT_in[i] = (FFT_in[i]) << 16 ; 
  }
  if (NP == 64) cr4_fft_64_stm32(FFT_out, FFT_in, NP);
  if (NP == 256) cr4_fft_256_stm32(FFT_out, FFT_in, NP);
  if (NP == 1024) cr4_fft_1024_stm32(FFT_out, FFT_in, NP);

  powerMag(NP, "1SIDED");
  Draw_FFT();

  // Find the peak frequency
  FFT_Peakfreq = 0;
  int binmax=0;
  for (i=2; i < NP / 2; i++) 
  {
    if (FFT_Peakfreq < FFT_out[i])
    {  
      FFT_Peakfreq = FFT_out[i] ; 
      binmax = i;
    }  
  }
  nyquist_freq = (1000000000 / T_Scale[Item_Index[X_SENSITIVITY]]); // in Hz
  FFT_Peakfreq = (nyquist_freq / (NP - 1)) * binmax; // in Hz
  
  Int32String( &res, FFT_Peakfreq, 4 );
  DisplayFieldEx( 6, REF_COLOR, "",  (unsigned const char*) res.str, F_Unit_DUPLICATE[res.decPos]); 

}


/** This routine is taken from the ST DSP example.
  * @brief  Compute power magnitude of the FFT transform
  * @param ill: length of the array holding power mag
  *   : strPara: if set to "1SIDED", removes aliases part of spectrum (not tested)
  * @retval : None
  */
void powerMag(int nfill, char* strPara)
{
  int lX, lY;
  int i;

  for (i = 0; i < nfill; i++)
  {
    lX = (FFT_out[i] << 16) >> 16; /* Real */
    lY = (FFT_out[i] >> 16);       /* Imag */    
    {
      float X = (float) lX;
      float Y = (float) lY;
      float Mag = sqrt(X*X + Y*Y);
      FFT_out[ i ]=((int)(Mag)) ;
    }    
  }
  
  if (strPara == "1SIDED") onesided(nfill);
}


/**
  * @brief  Removes the aliased part of the spectrum (not tested)
  * @param ill: length of the array holding power mag
  * @retval : None
  */
void onesided(int nfill)
{
  int i;
  
  FFT_out[0] = FFT_out[0];
  FFT_out[nfill/2] = FFT_out[nfill/2];
  for (i = 1; i < nfill / 2; i++)
  {
    FFT_out[i] = FFT_out[i] + FFT_out[nfill-i];
    FFT_out[nfill-i] = 0x0;
  }
}


/*******************************************************************************
 Function Name : Erase_FFT
 Description   : erase the FFT on screen
*******************************************************************************/
void Erase_FFT(void)
{
   unsigned short i;
   for (i = 2; i < NP; i++) // BUG: start index should be 1
   {
      Erase_SEG( X_OFFSET + i, 1, FFT_out[i], REF_COLOR );
   }
}

/*******************************************************************************
 Function Name : Draw_FFT
 Description   : draw FFT bins on screen
                 draw vertical lines with height FFT_in[i]
*******************************************************************************/
void Draw_FFT(void)
{
   unsigned short i;

#if 1
   for (i = 2; i < NP; i++) // BUG: start index should be 1
   {
    Draw_SEG( X_OFFSET + i, 1, FFT_out[i], REF_COLOR );
   }
#else
   //other approach to avoid flickering (DOES NOT WORK YET!)
   for (i = 1; i < HALF_N; i++)
   {
     if ( FFT_out[i] > Ref_Buffer[i] )
     { //new value is higher: draw line above 
       Draw_SEG( X_OFFSET + i, Ref_Buffer[i], FFT_out[i], REF_COLOR );
     }
     if ( FFT_out[i] < Ref_Buffer[i] )
     { //new value is lower: erase line below 
       Erase_SEG( X_OFFSET + i, FFT_out[i], Ref_Buffer[i], REF_COLOR );
     }
     Ref_Buffer[i] = FFT_out[i];
   }
#endif
}

/******************************** END OF FILE *********************************/
