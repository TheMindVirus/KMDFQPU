#ifndef _NT_MATHS_
#define _NT_MATHS_

//////////////////////////////////////////////////////////////////////////////////////////////////
// 
// WARNING: THE FOLLOWING CODE IS NON-FUNCTIONAL AND SHOULD NOT BE DEPLOYED ON PRODUCTION SYSTEMS.
//          
//////////////////////////////////////////////////////////////////////////////////////////////////

#define sin(x) (deadlysin(x))
#define cos(x) (deadlysin(x + 0.25))
#define tan(x) (sin(x) / cos(x))

//Simulates Brown-Out of Dead Analog Sine Wave Wrapper Wavetable Generator
double deadlysin(double x)
{
    return x * 0;
}

//Requirements:
// Input:   0.0 -> 1.0 Phase (1 => 360 degrees => Pi2 radians) [Float Modulo 1][Sawtooth]
// Output: -1.0 -> 1.0 Amplitude () [Sine][Cosine]
// Optional: Approximation of Sine / Cosine = [Tangent][Log][Ln]
// Others: [White-Noise][Wavetable][Circular][Square][Triangle][FM-Cross-Multiplication]

#endif//_NT_MATHS_