#ifndef _FPREAL_H_
#define _FPREAL_H_

#include <stdio.h>
#include <cstdint>
#include <cstdlib>

namespace mke {
namespace dd {
namespace types {

/** \addtogroup mkedd
 *  @{
 */    

template <typename T, typename TD, typename UT, int width = 16, bool isrounded = false, bool issaturated = false>
class FPReal {
  private:
   T value;  

 public:
    // Constructors
    FPReal() : value(T(0)) {}

    FPReal(int v, bool verbatim = false) {
      value = (verbatim) ? T(v) : T(v) << width;
    }

    FPReal(unsigned char v) {
      value = T(v) << width;
    }
  
    FPReal(double v) {
       double tmp = v * one();

       if (isrounded)
         tmp += (tmp >= 0) ? 0.5f : -0.5f;

       value = (T) tmp;
    }  

    FPReal(float v) {
       float tmp = v * one();

       if (isrounded)
         tmp += (tmp >= 0) ? 0.5f : -0.5f;

       value = (T) tmp;
    }
  
    // Conversion
    
    explicit inline operator int() const {
      if (isrounded)
        {
          if (value >= 0) 
            return int((value + (one() >> 1)) / one());
          else
            return int((value - (one() >> 1)) / one());
        }
      else
        {        
          return int(value >> width);
        }
    }

    explicit inline operator float() const {
      return float(value) / one();
    }

    explicit inline operator double() const {
      return double(value) / one();
    }

    explicit inline operator bool() const {
      return (bool) value;
    }
    

    // Assignment
    inline FPReal & operator= (int v) {
      value = T(v) << width; 
      return *this; 
    }

    inline FPReal & operator= (unsigned char v) {
      value = T(v) << width; 
      return *this; 
    }

    inline FPReal & operator= (double v) {
      double tmp = v * one();

      if (isrounded)
        tmp += (tmp >= 0) ? 0.5f : -0.5f;

      value = (T) tmp;
      return *this;
    }

    inline FPReal & operator=( float v) {
      float tmp = v * one();

      if (isrounded)
        tmp += (tmp >= 0) ? 0.5f : -0.5f;

      value = (T) tmp;
      return *this;
    }
 
    inline void assign (T v) {
      value = v;
    }
    
    // Access
    const T& getValue() const { return value;}

    // Comparison
    inline bool operator== (const FPReal & fpf) const {
      return fpf.getValue() == value;
    }

    inline bool operator!= (const FPReal & fpf) const {
      return fpf.getValue() != value;
    }
    
    inline bool operator< (const FPReal & fpf) const {
      return value < fpf.getValue();
    }

    inline bool operator<= (const FPReal & fpf) const {
      return value <= fpf.getValue();
    }

    inline bool operator> (const FPReal & fpf) const {
      return value > fpf.getValue();
    }

    inline bool operator>= (const FPReal & fpf) const {
      return value >= fpf.getValue();
    }

    inline bool operator! (void) const {
      return !value;
    }
    

    // Arithmetics
    inline FPReal & operator += (const FPReal & fpf) {
      FPReal tmp = *this + fpf;
      value = tmp.getValue();      
      return *this;
    }

    inline FPReal & operator -= (const FPReal & fpf) {
      FPReal tmp = *this - fpf;
      value = tmp.getValue();
      return *this;
    }
    
    inline FPReal & operator *= (const FPReal & fpf) {
      FPReal tmp = *this * fpf;
      value = tmp.getValue();
      return *this;
    }    

    inline FPReal & operator /= (const FPReal & fpf) {
      FPReal tmp = *this / fpf;
      value = tmp.getValue();
      return *this;
    }    

    inline FPReal & operator >>= (const int nbits) {
      value = value >> nbits;
      return *this;
    }    

    inline FPReal & operator <<= (const int nbits) {
      value = value << nbits;
      return *this;
    }    
    
    static inline T one(void) { return T(1) << width; }
    static inline UT sign(void) { return T(1) << (nbits() - 1); }
    static inline T max(void) { return T(~UT(0) >> 1); }
    static inline T min(void) { return T(1) << (nbits() - 1); }
    static inline T msbyte(void) { return T(0xF) << (nbits() - 4); }
    static inline T msbytes3(void) { return T(0xFFF) << (nbits() - 12); }
    static inline size_t nbits(void) { return sizeof(UT) * 8; }
    static inline size_t iwidth(void) { return nbits() - width; }
    static inline size_t ndsign(void) { return 2 * nbits() - (width + iwidth() * 2 - 1); }

    //static const uint32_t magic_uint32[][3];
    //static const uint32_t magic_int32[][2];

    static inline size_t clz(UT x) {
    #ifdef __GNUC__
      return __builtin_clzl(x) - 8 * (sizeof(unsigned long long) - sizeof(UT));
    #else
      size_t result = 0;
      if (x == 0) 
        return nbits();

       while (!(x & msbyte())) 
         { 
           result += 4; 
           x <<= 4; 
         }

       while (!(x & sign())) 
         { 
           result += 1; 
           x <<= 1; 
         }
       return result;
    #endif
    }   
};


// Typedefs

//typedef double fpreal16;
typedef FPReal<int32_t, int64_t, uint32_t, 16, false, false> fpreal16;
typedef FPReal<int32_t, int64_t, uint32_t, 16, true, false> fpreal16r;
typedef FPReal<int32_t, int64_t, uint32_t, 8, false, false> fpreal8;
typedef FPReal<uint32_t, uint64_t, uint32_t, 16, false, false> fpureal16;
typedef FPReal<uint16_t, uint32_t, uint16_t, 3, false, false> fpureal3;
typedef FPReal<uint16_t, uint32_t, uint16_t, 3, true, false> fpureal3r;

typedef FPReal<int16_t, int32_t, int16_t, 1, true, false> fhreal1r;
typedef FPReal<int16_t, int32_t, int16_t, 2, true, false> fhreal2r;
typedef FPReal<int16_t, int32_t, int16_t, 3, true, false> fhreal3r;
typedef FPReal<int16_t, int32_t, int16_t, 4, true, false> fhreal4r;


// Templated operators

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator+ (const FPReal<T1, T2, T3, T4, T5, T6> &fpf) {
  return fpf;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator<< (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const int &nbits) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  tmp.assign(fpf1.getValue() << nbits);
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator>> (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const int &nbits) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  tmp.assign(fpf1.getValue() >> nbits);
  return tmp;
}

#ifdef __nios2_arch__
template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6>
inline FPReal<T1, T2, T3, T4, T5, T6> operator<< (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const int32_t &nbits) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  tmp.assign(fpf1.getValue() << nbits);
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6>
inline FPReal<T1, T2, T3, T4, T5, T6> operator>> (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const int32_t &nbits) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  tmp.assign(fpf1.getValue() >> nbits);
  return tmp;
}
#endif

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator/ (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const FPReal<T1, T2, T3, T4, T5, T6> &fpf2) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;

  tmp.assign((T2(fpf1.getValue()) << T4) / T2(fpf2.getValue()));
  return tmp;
/*
  if (fpf2.getValue() == 0)
      return FPFloat<T1, T2, T3, T4, T5, T6>::min();
  
  T3 remainder = (fpf1.getValue() >= 0) ? fpf1.getValue() : (-fpf1.getValue());
  T3 divider = (fpf2.getValue() >= 0) ? fpf2.getValue() : (-fpf2.getValue());
  T3 quotient = 0;
  int bit_pos = T4 + 1;
  
  if (divider & FPFloat<T1, T2, T3, T4, T5, T6>::msbytes3())
    {
      T3 shifted_div = ((divider >> (T4 + 1)) + 1);
      quotient = remainder / shifted_div;
      remainder -= ((T2)quotient * divider) >> (T4 + 1);
    }

  while (!(divider & 0xF) && (bit_pos >= 4))
    {
      divider >>= 4;
      bit_pos -= 4;
    }

  while (remainder && (bit_pos >= 0))
    {
      size_t shift = FPFloat<T1, T2, T3, T4, T5, T6>::clz(remainder);

      if (shift > bit_pos) 
        shift = bit_pos;
    
      remainder <<= shift;
      bit_pos -= shift;
    
      T3 div = remainder / divider;
      remainder = remainder % divider;
      quotient += div << bit_pos;

      if (T6)
        {
          if (div & ~(~T3(0) >> bit_pos))
            {
              if ((fpf1.getValue() >= 0) == (fpf2.getValue() >= 0))
                tmp.assign(FPFloat<T1, T2, T3, T4, T5, T6>::max());
              else
                tmp.assign(FPFloat<T1, T2, T3, T4, T5, T6>::min());  
              return tmp;              
            }
        }
    
      remainder <<= 1;
      bit_pos--;
    }
  
  quotient++;
  T1 result = quotient >> 1;
  
  if ((fpf1.getValue() ^ fpf2.getValue()) & FPFloat<T1, T2, T3, T4, T5, T6>::sign())
  {
    if (T6)
      {
        if (result == FPFloat<T1, T2, T3, T4, T5, T6>::min())
          {
            if ((fpf1.getValue() >= 0) == (fpf2.getValue() >= 0))
              tmp.assign(FPFloat<T1, T2, T3, T4, T5, T6>::max());
            else
              tmp.assign(FPFloat<T1, T2, T3, T4, T5, T6>::min());  
            return tmp;
	  }
      }

    result = -result;
  }
  
  tmp.assign(result);  
  return tmp;
*/
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6>
inline FPReal<T1, T2, T3, T4, T5, T6> operator* (const T1 &i1, const FPReal<T1, T2, T3, T4, T5, T6> &fpf2) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;

  tmp.assign(i1 * fpf2.getValue());
  return tmp;
}

template <typename T, typename T1, typename T2, typename T3, int T4, bool T5, bool T6>
inline FPReal<T1, T2, T3, T4, T5, T6> operator* (const FPReal<T1, T2, T3, T4, T5, T6> &fpf2, const T &i1) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;

  tmp.assign(T1(i1 * fpf2.getValue()));
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator* (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const FPReal<T1, T2, T3, T4, T5, T6> &fpf2) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  
  T3 upper;
  T2 prod = T2(fpf1.getValue()) * T2(fpf2.getValue());
  
  if (T6)
    upper = T3(prod >> (2 * FPReal<T1, T2, T3, T4, T5, T6>::nbits() - FPReal<T1, T2, T3, T4, T5, T6>::ndsign()));
    
  if (prod < 0)
    {
      if (T6)
        {
          if (~upper)
            {
              if ((fpf1.getValue() >= 0) == (fpf2.getValue() >= 0))
                tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::max());
              else
                tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::min());
              return tmp;              
            }
        }
      
      if (T5)
        prod--; 
    }
  else
    {
      if (T6)
        {
          if (upper)
            {
              if ((fpf1.getValue() >= 0) == (fpf2.getValue() >= 0))
                tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::max());
              else
                tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::min());
              return tmp;
            }
        }
    }
  
  if (T5)
    {
      T1 res = T1(prod >> T4);
      res += (prod & (FPReal<T1, T2, T3, T4, T5, T6>::one() >> 1) >> 15);
      tmp.assign(res);      
    }
  else
    {
      tmp.assign(T1(prod >> T4));
    }

  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator+ (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const FPReal<T1, T2, T3, T4, T5, T6> &fpf2) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  
  if (!T6)
    {
      tmp.assign(fpf1.getValue() + fpf2.getValue());
    }
  else
    {
      T3 v1 = T3(fpf1.getValue());
      T3 v2 = T3(fpf2.getValue());
      T3 sum = v1 + v2;
      
      if (!((v1 ^ v2) & FPReal<T1, T2, T3, T4, T5, T6>::sign()) &&
           ((v1 ^ sum) & FPReal<T1, T2, T3, T4, T5, T6>::sign()))
        {
          if (fpf1.getValue() >= 0)
            tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::max());
          else
            tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::min());
        }
      else
        {
          tmp.assign(T1(sum));
        }
    }
      
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator- (const FPReal<T1, T2, T3, T4, T5, T6> &fpf1, const FPReal<T1, T2, T3, T4, T5, T6> &fpf2) {
  FPReal<T1, T2, T3, T4, T5, T6> tmp;
  
  if (!T6)
    {
      tmp.assign(fpf1.getValue() - fpf2.getValue());
    }
  else
    {
      T3 v1 = T3(fpf1.getValue());
      T3 v2 = T3(fpf2.getValue());
      T3 diff = v1 - v2;
      
      if (((v1 ^ v2) & FPReal<T1, T2, T3, T4, T5, T6>::sign()) &&
           ((v1 ^ diff) & FPReal<T1, T2, T3, T4, T5, T6>::sign()))
        {
          if (fpf1.getValue() >= 0)
            tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::max());
          else
            tmp.assign(FPReal<T1, T2, T3, T4, T5, T6>::min());
        }
      else
        {
          tmp.assign(T1(diff));
        }
    }
      
  return tmp;
}


template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> operator- (const FPReal<T1, T2, T3, T4, T5, T6> &fpf) {
  FPReal<T1, T2, T3, T4, T5> tmp;
  tmp.assign(-fpf.getValue());
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> abs(const FPReal<T1, T2, T3, T4, T5, T6> &fpf) {
  FPReal<T1, T2, T3, T4, T5> tmp;
  tmp.assign(std::abs(fpf.getValue()));
  return tmp;
}

template <typename T1, typename T2, typename T3, int T4, bool T5, bool T6> 
inline FPReal<T1, T2, T3, T4, T5, T6> floor(const FPReal<T1, T2, T3, T4, T5, T6> &fpf) {
  FPReal<T1, T2, T3, T4, T5> tmp;
  tmp.assign((fpf.getValue() >> T4) << T4);
  return tmp;
}


///template <typename int32_t, typename int64_t, typename uint32_t, int T4, bool T5, bool T6> 
template <typename int32_t, typename int64_t, typename uint32_t, int T4> 
inline FPReal<int32_t, int64_t, uint32_t, T4, false, false> operator/ (const FPReal<int32_t, int64_t, uint32_t, T4, false, false> &fpf, const int32_t &divider) {
  static const uint32_t magic_int32[][2] = {
	{0x00000000, 0}, {0xFFFFFFFF, 0}, {0x80000001, 0}, {0x55555556, 0}, 
	{0x80000001, 1}, {0x66666667, 1}, {0x2AAAAAAB, 0}, {0x92492493, 2}, 
	{0x80000001, 2}, {0x38E38E39, 1}, {0x66666667, 2}, {0x2E8BA2E9, 1}, 
	{0x2AAAAAAB, 1}, {0x4EC4EC4F, 2}, {0x92492493, 3}, {0x88888889, 3}, 
	{0x80000001, 3}, {0x78787879, 3}, {0x38E38E39, 2}, {0x6BCA1AF3, 3}, 
	{0x66666667, 3}, {0x30C30C31, 2}, {0x2E8BA2E9, 2}, {0xB21642C9, 4}, 
	{0x2AAAAAAB, 2}, {0x51EB851F, 3}, {0x4EC4EC4F, 3}, {0x4BDA12F7, 3}, 
	{0x92492493, 4}, {0x8D3DCB09, 4}, {0x88888889, 4}, {0x84210843, 4}, 
	{0x80000001, 4}, {0x3E0F83E1, 3}, {0x78787879, 4}, {0xEA0EA0EB, 5}, 
	{0x38E38E39, 3}, {0xDD67C8A7, 5}, {0x6BCA1AF3, 4}, {0xD20D20D3, 5}, 
	{0x66666667, 4}, {0x63E7063F, 4}, {0x30C30C31, 3}, {0x2FA0BE83, 3}, 
	{0x2E8BA2E9, 3}, {0xB60B60B7, 5}, {0xB21642C9, 5}, {0xAE4C415D, 5}, 
	{0x2AAAAAAB, 3}, {0x5397829D, 4}, {0x51EB851F, 4}, {0xA0A0A0A1, 5}, 
	{0x4EC4EC4F, 4}, {0x4D4873ED, 4}, {0x4BDA12F7, 4}, {0x094F2095, 1}, 
	{0x92492493, 5}, {0x8FB823EF, 5}, {0x8D3DCB09, 5}, {0x22B63CBF, 3}, 
	{0x88888889, 5}, {0x4325C53F, 4}, {0x84210843, 5}, {0x82082083, 5}, 
	{0x80000001, 5}, {0x7E07E07F, 5}, {0x3E0F83E1, 4}, {0x07A44C6B, 1}, 
	{0x78787879, 5}, {0x76B981DB, 5}, {0xEA0EA0EB, 6}, {0xE6C2B449, 6}, 
	{0x38E38E39, 4}, {0xE070381D, 6}, {0xDD67C8A7, 6}, {0x1B4E81B5, 3}, 
	{0x6BCA1AF3, 5}, {0x3531DEC1, 4}, {0xD20D20D3, 6}, {0x67B23A55, 5}, 
	{0x66666667, 5}, {0x1948B0FD, 3}, {0x63E7063F, 5}, {0x3159721F, 4}, 
	{0x30C30C31, 4}, {0x60606061, 5}, {0x2FA0BE83, 4}, {0x2F149903, 4}, 
	{0x2E8BA2E9, 4}, {0xB81702E1, 6}, {0xB60B60B7, 6}, {0xB40B40B5, 6}, 
	{0xB21642C9, 6}, {0x2C0B02C1, 4}, {0xAE4C415D, 6}, {0xAC769185, 6}, 
	{0x2AAAAAAB, 4}, {0x151D07EB, 3}, {0x5397829D, 5}, {0xA57EB503, 6}, 
	{0x51EB851F, 5}, {0x288DF0CB, 4}, {0xA0A0A0A1, 6}, {0x13E22CBD, 3}, 
	{0x4EC4EC4F, 5}, {0x9C09C09D, 6}, {0x4D4873ED, 5}, {0x4C8F8D29, 5}, 
	{0x4BDA12F7, 5}, {0x964FDA6D, 6}, {0x094F2095, 2}, {0x939A85C5, 6}, 
	{0x92492493, 6}, {0x487EDE05, 5}, {0x8FB823EF, 6}, {0x473C1AB7, 5}, 
	{0x8D3DCB09, 6}, {0x8C08C08D, 6}, {0x22B63CBF, 4}, {0x44D72045, 5}, 
	{0x88888889, 6}, {0x043B3D5B, 1}, {0x4325C53F, 5}, {0x214D0215, 4}, 
	{0x84210843, 6}, {0x10624DD3, 3}, {0x82082083, 6}, {0x81020409, 6}, 
	{0x80000001, 6}, {0x0FE03F81, 3}, {0x7E07E07F, 6}, {0x3E88CB3D, 5}, 
	{0x3E0F83E1, 5}, {0xF6603D99, 7}, {0x07A44C6B, 2}, {0xF2B9D649, 7}, 
	{0x78787879, 6}, {0x077975B9, 2}, {0x76B981DB, 6}, {0x75DED953, 6}, 
	{0xEA0EA0EB, 7}, {0x3A196B1F, 5}, {0xE6C2B449, 7}, {0x3949660B, 5}, 
	{0x38E38E39, 5}, {0xE1FC780F, 7}, {0xE070381D, 7}, {0x6F74AE27, 6}, 
	{0xDD67C8A7, 7}, {0x0DBEB61F, 3}, {0x1B4E81B5, 4}, {0x36406C81, 5}, 
	{0x6BCA1AF3, 6}, {0xD62B80D7, 7}, {0x3531DEC1, 5}, {0x1A6D01A7, 4}, 
	{0xD20D20D3, 7}, {0x342DA7F3, 5}, {0x67B23A55, 6}, {0x19C2D14F, 4}, 
	{0x66666667, 6}, {0xCB8727C1, 7}, {0x1948B0FD, 4}, {0x0C907DA5, 3}, 
	{0x63E7063F, 6}, {0x634C0635, 6}, {0x3159721F, 5}, {0x621B97C3, 6}, 
	{0x30C30C31, 5}, {0x60F25DEB, 6}, {0x60606061, 6}, {0xBFA02FE9, 7}, 
	{0x2FA0BE83, 5}, {0x17AD2209, 4}, {0x2F149903, 5}, {0x5D9F7391, 6}, 
	{0x2E8BA2E9, 5}, {0xB92143FB, 7}, {0xB81702E1, 7}, {0xB70FBB5B, 7}, 
	{0xB60B60B7, 7}, {0x2D4279A3, 5}, {0xB40B40B5, 7}, {0xB30F6353, 7}, 
	{0xB21642C9, 7}, {0xB11FD3B9, 7}, {0x2C0B02C1, 5}, {0xAF3ADDC7, 7}, 
	{0xAE4C415D, 7}, {0xAD602B59, 7}, {0xAC769185, 7}, {0xAB8F69E3, 7}, 
	{0x2AAAAAAB, 5}, {0x15390949, 4}, {0x151D07EB, 4}, {0xA80A80A9, 7}, 
	{0x5397829D, 6}, {0x532AE21D, 6}, {0xA57EB503, 7}, {0x5254E78F, 6}, 
	{0x51EB851F, 6}, {0x028C1979, 1}, {0x288DF0CB, 5}, {0xA16B312F, 7}, 
	{0xA0A0A0A1, 7}, {0x4FEC04FF, 6}, {0x13E22CBD, 4}, {0x27932B49, 5}, 
	{0x4EC4EC4F, 6}, {0x9CC8E161, 7}, {0x9C09C09D, 7}, {0x04DA637D, 2}, 
	{0x4D4873ED, 6}, {0x99D722DB, 7}, {0x4C8F8D29, 6}, {0x4C346405, 6}, 
	{0x4BDA12F7, 6}, {0x97012E03, 7}, {0x964FDA6D, 7}, {0x95A02569, 7}, 
	{0x094F2095, 3}, {0x94458095, 7}, {0x939A85C5, 7}, {0x92F11385, 7}, 
	{0x92492493, 7}, {0x91A2B3C5, 7}, {0x487EDE05, 6}, {0x24168E19, 5}, 
	{0x8FB823EF, 7}, {0x478BBCED, 6}, {0x473C1AB7, 6}, {0x8DDA5203, 7}, 
	{0x8D3DCB09, 7}, {0x8CA29C05, 7}, {0x8C08C08D, 7}, {0x8B70344B, 7}, 
	{0x22B63CBF, 5}, {0x8A42F871, 7}, {0x44D72045, 6}, {0x891AC73B, 7}, 
	{0x88888889, 7}, {0x10FEF011, 4}, {0x043B3D5B, 2}, {0x86D90545, 7}, 
	{0x4325C53F, 6}, {0x42DF9BB1, 6}, {0x214D0215, 5}, {0x84A9F9C9, 7}, 
	{0x84210843, 7}, {0x83993053, 7}, {0x10624DD3, 4}, {0x828CBFBF, 7}, 
	{0x82082083, 7}, {0x81848DA9, 7}, {0x81020409, 7}, {0x80808081, 7}, 
	{0x80000001, 7}, {0x7F807F81, 7}, {0x0FE03F81, 4}, {0x7E8472A9, 7}, 
	{0x7E07E07F, 7}, {0x7D8C42B3, 7}, {0x3E88CB3D, 6}, {0x7C97D911, 7}, 
	{0x3E0F83E1, 6}, {0x3DD38FF1, 6}, {0xF6603D99, 8}, {0x7ABA01EB, 7}, 
	{0x07A44C6B, 3}, {0x79D06A97, 7}, {0xF2B9D649, 8}, {0x0F1D48BD, 4}, 
	{0x78787879, 7}, {0xF00F00F1, 8}, {0x077975B9, 3}, {0x77280773, 7}, 
	{0x76B981DB, 7}, {0x764BC88D, 7}, {0x75DED953, 7}, {0x3AB95901, 6}, 
	{0xEA0EA0EB, 8}, {0x0749CB29, 3}, {0x3A196B1F, 6}, {0xE79372E3, 8}, 
	{0xE6C2B449, 8}, {0xE5F36CB1, 8}, {0x3949660B, 6}, {0x1C8B265B, 5}, 
	{0x38E38E39, 6}, {0x71625345, 7}, {0xE1FC780F, 8}, {0x709AD4E5, 7}, 
	{0xE070381D, 8}, {0xDFAC1F75, 8}, {0x6F74AE27, 7}, {0xDE27EB2D, 8}, 
	{0xDD67C8A7, 8}, {0x6E5478AD, 7}, {0x0DBEB61F, 4}, {0x6D978B8F, 7}, 
	{0x1B4E81B5, 5}, {0x1B37484B, 5}, {0x36406C81, 6}, {0x0D84A599, 4}, 
	{0x6BCA1AF3, 7}, {0x6B6FA1FF, 7}, {0xD62B80D7, 8}, {0xD578E97D, 8}, 
	{0x3531DEC1, 6}, {0x6A0B9945, 7}, {0x1A6D01A7, 5}, {0x34AE820F, 6}, 
	{0xD20D20D3, 8}, {0xD161543F, 8}, {0x342DA7F3, 6}, {0xD00D00D1, 8}, 
	{0x67B23A55, 7}, {0x33AF3E2F, 6}, {0x19C2D14F, 5}, {0x335C49D5, 6}, 
	{0x66666667, 7}, {0x6614BC37, 7}, {0xCB8727C1, 8}, {0x06572EC3, 3}, 
	{0x1948B0FD, 5}, {0x64D319FF, 7}, {0x0C907DA5, 4}, {0xC86A7891, 8}, 
	{0x63E7063F, 7}, {0x18E6527B, 5}, {0x634C0635, 7}, {0x317F9D01, 6}, 
	{0x3159721F, 6}, {0xC4CE07B1, 8}, {0x621B97C3, 7}, {0xC3A13DE7, 8}, 
	{0x30C30C31, 6}, {0x309E0185, 6}, {0x60F25DEB, 7}, {0xC152500D, 8}, 
	{0x60606061, 7}, {0x300C0301, 6}, {0xBFA02FE9, 8}, {0x2FC44AA3, 6}, 
	{0x2FA0BE83, 6}, {0x5EFACE49, 7}, {0x17AD2209, 5}, {0x5E6EA9AF, 7}, 
	{0x2F149903, 6}, {0x5DE42047, 7}, {0x5D9F7391, 7}, {0xBAB65611, 8}, 
	{0x2E8BA2E9, 6}, {0xB9A7862B, 8}, {0xB92143FB, 8}, {0x5C4DE1B7, 7}, 
	{0xB81702E1, 8}, {0x16F26017, 5}, {0xB70FBB5B, 8}, {0xB68D3135, 8}, 
	{0xB60B60B7, 8}, {0x5AC5242B, 7}, {0x2D4279A3, 6}, {0xB48A39D5, 8}, 
	{0xB40B40B5, 8}, {0xB38CF9B1, 8}, {0xB30F6353, 8}, {0x59493E15, 7}, 
	{0xB21642C9, 8}, {0xB19AB5C5, 8}, {0xB11FD3B9, 8}, {0x5852CDA1, 7}, 
	{0x2C0B02C1, 6}, {0x57D990D1, 7}, {0xAF3ADDC7, 8}, {0x057619F1, 3}, 
	{0xAE4C415D, 8}, {0xADD5E633, 8}, {0xAD602B59, 8}, {0x567587C5, 7}, 
	{0xAC769185, 8}, {0x2B00AC03, 6}, {0xAB8F69E3, 8}, {0x2AC72F75, 6}, 
	{0x2AAAAAAB, 6}, {0x551C979B, 7}, {0x15390949, 5}, {0x54ABFD5B, 7}, 
	{0x151D07EB, 5}, {0xA8791709, 8}, {0xA80A80A9, 8}, {0x14F38F63, 5}, 
	{0x5397829D, 7}, {0x14D843BF, 5}, {0x532AE21D, 7}, {0x52F4FB77, 7}, 
	{0xA57EB503, 8}, {0x2944FF5B, 6}, {0x5254E78F, 7}, {0xA4402911, 8}, 
	{0x51EB851F, 7}, {0x28DB9C69, 6}, {0x028C1979, 2}, {0xA29ECF17, 8}, 
	{0x288DF0CB, 6}, {0x50E89CC3, 7}, {0xA16B312F, 8}, {0x28416A4D, 6}, 
	{0xA0A0A0A1, 8}, {0xA03C1689, 8}, {0x4FEC04FF, 7}, {0x4FBA3D0B, 7}, 
	{0x13E22CBD, 5}, {0x4F576647, 7}, {0x27932B49, 6}, {0x4EF58365, 7}, 
	{0x4EC4EC4F, 7}, {0x274A4871, 6}, {0x9CC8E161, 8}, {0x271A45A7, 6}, 
	{0x9C09C09D, 8}, {0x9BAADE8F, 8}, {0x04DA637D, 3}, {0x4D77397F, 7}, 
	{0x4D4873ED, 7}, {0x134679AD, 5}, {0x99D722DB, 8}, {0x265EB9DB, 6}, 
	{0x4C8F8D29, 7}, {0x13187759, 5}, {0x4C346405, 7}, {0x980E4157, 8}, 
	{0x4BDA12F7, 7}, {0x0975A751, 4}, {0x97012E03, 8}, {0x4B542805, 7}, 
	{0x964FDA6D, 8}, {0x95F7CC73, 8}, {0x95A02569, 8}, {0x12A91C93, 5}, 
	{0x094F2095, 4}, {0x4A4DC96F, 7}, {0x94458095, 8}, {0x49F7E8E3, 7}, 
	{0x939A85C5, 8}, {0x93459BE7, 8}, {0x92F11385, 8}, {0x929CEBF5, 8}, 
	{0x92492493, 8}, {0x91F5BCB9, 8}, {0x91A2B3C5, 8}, {0x48A8048B, 7}, 
	{0x487EDE05, 7}, {0x90ABCC03, 8}, {0x24168E19, 6}, {0x90090091, 8}, 
	{0x8FB823EF, 8}, {0x23D9E879, 6}, {0x478BBCED, 7}, {0x4763D59D, 7}, 
	{0x473C1AB7, 7}, {0x47148BF1, 7}, {0x8DDA5203, 8}, {0x02362F8D, 2}, 
	{0x8D3DCB09, 8}, {0x08CF008D, 4}, {0x8CA29C05, 8}, {0x8C55841D, 8}, 
	{0x8C08C08D, 8}, {0x8BBC50C9, 8}, {0x8B70344B, 8}, {0x11648D51, 5}, 
	{0x22B63CBF, 6}, {0x04546E69, 3}, {0x8A42F871, 8}, {0x44FC3A35, 7}, 
	{0x44D72045, 7}, {0x089645C5, 4}, {0x891AC73B, 8}, {0x4468C067, 7}, 
	{0x88888889, 8}, {0x883FDDF1, 8}, {0x10FEF011, 5}, {0x43D7B7EB, 7}, 
	{0x043B3D5B, 3}, {0x872032AD, 8}, {0x86D90545, 8}, {0x43491159, 7}, 
	{0x4325C53F, 7}, {0x86053C35, 8}, {0x42DF9BB1, 7}, {0x42BCBDC9, 7}, 
	{0x214D0215, 6}, {0x42776E9B, 7}, {0x84A9F9C9, 8}, {0x21195767, 6}, 
	{0x84210843, 8}, {0x41EE7CA7, 7}, {0x83993053, 8}, {0x20D56B39, 6}, 
	{0x10624DD3, 5}, {0x20B3DD41, 6}, {0x828CBFBF, 8}, {0x824A4E61, 8}, 
	{0x82082083, 8}, {0x81C635BD, 8}, {0x81848DA9, 8}, {0x2050C9F9, 6}, 
	{0x81020409, 8}, {0x80C121B3, 8}, {0x80808081, 8}, {0x80402011, 8}, 
	{0x80000001, 8}, {0x7FC01FF1, 8}, {0x7F807F81, 8}, {0x7F411E53, 8}, 
	{0x0FE03F81, 5}, {0x1FB0C611, 6}, {0x7E8472A9, 8}, {0x7E460ADB, 8}, 
	{0x7E07E07F, 8}, {0x3EE4F99D, 7}, {0x7D8C42B3, 8}, {0x07D4ECE9, 4}, 
	{0x3E88CB3D, 7}, {0x7CD49A17, 8}, {0x7C97D911, 8}, {0xF8B6A623, 9}, 
	{0x3E0F83E1, 7}, {0x7BE2F6CF, 8}, {0x3DD38FF1, 7}, {0x7B6B82A7, 8}, 
	{0xF6603D99, 9}, {0x7AF4F3FF, 8}, {0x7ABA01EB, 8}, {0x3D3FA421, 7}, 
	{0x07A44C6B, 4}, {0x7A0A7CE7, 8}, {0x79D06A97, 8}, {0x079968F7, 4}, 
	{0xF2B9D649, 9}, {0x3C91BEB3, 7}, {0x0F1D48BD, 5}, {0x78B1445D, 8}, 
	{0x78787879, 8}, {0xF07FC3E1, 9}, {0xF00F00F1, 9}, {0x3BE7A9E3, 7}, 
	{0x077975B9, 4}, {0x775F978D, 8}, {0x77280773, 8}, {0x3B78557D, 7}, 
	{0x76B981DB, 8}, {0xED05179D, 9}, {0x764BC88D, 8}, {0xEC2A6FA1, 9}, 
	{0x75DED953, 8}, {0x075A8ACD, 4}, {0x3AB95901, 7}, {0x753CE8A5, 8}, 
	{0xEA0EA0EB, 9}, {0xE9A3D25F, 9}, {0x0749CB29, 4}, {0x3A33D62B, 7}, 
	{0x3A196B1F, 7}, {0x0E7FC601, 5}, {0xE79372E3, 9}, {0x7395723B, 8}, 
	{0xE6C2B449, 9}, {0x3996B877, 7}, {0xE5F36CB1, 9}, {0x72C62A25, 8}, 
	{0x3949660B, 7}, {0x725F9BED, 8}, {0x1C8B265B, 6}, {0x0E3F388B, 5}, 
	{0x38E38E39, 7}, {0x00E32943, 1}, {0x71625345, 8}, {0xE260630B, 9}, 
	{0xE1FC780F, 9}, {0x070CC729, 4}, {0x709AD4E5, 8}, {0x706962CD, 8}, 
	{0xE070381D, 9}, {0xE00E00E1, 9}, {0xDFAC1F75, 9}, {0xDF4A9369, 9}, 
	{0x6F74AE27, 8}, {0x37A21E6D, 7}, {0xDE27EB2D, 9}, {0xDDC7B04D, 9}, 
	{0xDD67C8A7, 9}, {0x6E8419E7, 8}, {0x6E5478AD, 8}, {0x6E25006F, 8}, 
	{0x0DBEB61F, 5}, {0x1B71A285, 6}, {0x6D978B8F, 8}, {0x36B45A9B, 7}, 
	{0x1B4E81B5, 6}, {0x6D0B8037, 8}, {0x1B37484B, 6}, {0x00D95DD3, 1}, 
	{0x36406C81, 7}, {0x00D8A5DF, 1}, {0x0D84A599, 5}, {0x6BF790A9, 8}, 
	{0x6BCA1AF3, 8}, {0x6B9CCB75, 8}, {0x6B6FA1FF, 8}, {0x6B429E61, 8}, 
	{0xD62B80D7, 9}, {0x06AE907F, 4}, {0xD578E97D, 9}, {0xD5200D53, 9}, 
	{0x3531DEC1, 7}, {0xD46F3235, 9}, {0x6A0B9945, 8}, {0xD3BF7BA9, 9}, 
	{0x1A6D01A7, 6}, {0x34C439B7, 7}, {0x34AE820F, 7}, {0x6931B881, 8}, 
	{0xD20D20D3, 9}, {0x68DB8BAD, 8}, {0xD161543F, 9}, {0x3442F5CB, 7}, 
	{0x342DA7F3, 7}, {0x6830D6E5, 8}, {0xD00D00D1, 9}, {0x33EE2623, 7}, 
	{0x67B23A55, 8}, {0x33C42535, 7}, {0x33AF3E2F, 7}, {0x6734D007, 8}, 
	{0x19C2D14F, 6}, {0x66E1DBD5, 8}, {0x335C49D5, 7}, {0x3347B649, 7}, 
	{0x66666667, 8}, {0x00663D81, 0}, {0x6614BC37, 8}, {0x197B05F9, 6}, 
	{0xCB8727C1, 9}, {0x659B3007, 8}, {0x06572EC3, 4}, {0x32A5641B, 7}, 
	{0x1948B0FD, 6}, {0x64FADF43, 8}, {0x64D319FF, 8}, {0x3255BA01, 7}, 
	{0x0C907DA5, 5}, {0x645C854B, 8}, {0xC86A7891, 9}, {0x640E11FB, 8}, 
	{0x63E7063F, 8}, {0xC78031E1, 9}, {0x18E6527B, 6}, {0x6372990F, 8}, 
	{0x634C0635, 8}, {0xC64B2279, 9}, {0x317F9D01, 7}, {0x62D90063, 8}, 
	{0x3159721F, 7}, {0xC519CAE1, 9}, {0xC4CE07B1, 9}, {0xC4827EA9, 9}, 
	{0x621B97C3, 8}, {0x61F60D03, 8}, {0xC3A13DE7, 9}, {0x61AB4D73, 8}, 
	{0x30C30C31, 7}, {0x6160FF9F, 8}, {0x309E0185, 7}, {0xC22E4507, 9}, 
	{0x60F25DEB, 8}, {0x60CDB521, 8}, {0xC152500D, 9}, {0x6084B67B, 8}, 
	{0x60606061, 8}, {0x0C0784B3, 5}, {0x300C0301, 7}, {0x00BFE803, 1}, 
	{0xBFA02FE9, 9}, {0x017EB125, 2}, {0x2FC44AA3, 7}, {0x5F64FBE7, 8}, 
	{0x2FA0BE83, 7}, {0x2F8F0C43, 7}, {0x5EFACE49, 8}, {0x2F6BCF19, 7}, 
	{0x17AD2209, 6}, {0x2F48C601, 7}, {0x5E6EA9AF, 8}, {0xBC97C21F, 9}, 
	{0x2F149903, 7}, {0xBC0D38EF, 9}, {0x5DE42047, 8}, {0xBB837AB1, 9}, 
	{0x5D9F7391, 8}, {0x5D7D42D5, 8}, {0xBAB65611, 9}, {0xBA725821, 9}, 
	{0x2E8BA2E9, 7}, {0x2E7ABC19, 7}, {0xB9A7862B, 9}, {0x2E591331, 7}, 
	{0xB92143FB, 9}, {0x5C6F35CD, 8}, {0x5C4DE1B7, 8}, {0xB8594B41, 9}, 
	{0xB81702E1, 9}, {0x5BEA750D, 8}, {0x16F26017, 6}, {0x5BA8A345, 8}, 
	{0xB70FBB5B, 9}, {0x5B672F7D, 8}, {0xB68D3135, 9}, {0x5B2618ED, 8}, 
	{0xB60B60B7, 9}, {0xB5CABD9B, 9}, {0x5AC5242B, 8}, {0x5AA5005B, 8}, 
	{0x2D4279A3, 7}, {0x5A64FCD3, 8}, {0xB48A39D5, 9}, {0x5A255375, 8}, 
	{0xB40B40B5, 9}, {0x59E60383, 8}, {0xB38CF9B1, 9}, {0x2CD38621, 7}, 
	{0xB30F6353, 9}, {0xB2D0D9EF, 9}, {0x59493E15, 8}, {0x164A893B, 6}, 
	{0xB21642C9, 9}, {0x58EC3369, 8}, {0xB19AB5C5, 9}, {0x58AE97BB, 8}, 
	{0xB11FD3B9, 9}, {0xB0E2A261, 9}, {0x5852CDA1, 8}, {0x58345F19, 8}, 
	{0x2C0B02C1, 7}, {0x2BFBE063, 7}, {0x57D990D1, 8}, {0xAF76EB19, 9}, 
	{0xAF3ADDC7, 9}, {0x577F7CC1, 8}, {0x057619F1, 4}, {0xAE87AB77, 9}, 
	{0xAE4C415D, 9}, {0x57087FD5, 8}, {0xADD5E633, 9}, {0x0AD9AF4D, 5}, 
	{0xAD602B59, 9}, {0x2B496269, 7}, {0x567587C5, 8}, {0x56585E71, 8}, 
	{0xAC769185, 9}, {0x561E46A5, 8}, {0x2B00AC03, 7}, {0xABC8F9A1, 9}, 
	{0xAB8F69E3, 9}, {0x2AD5802B, 7}, {0x2AC72F75, 7}, {0x5571D09B, 8}, 
	{0x2AAAAAAB, 7}, {0xAA71DA0D, 9}, {0x551C979B, 8}, {0xAA00AA01, 9}, 
	{0x15390949, 6}, {0x54C807F3, 8}, {0x54ABFD5B, 8}, {0xA9200A93, 9}, 
	{0x151D07EB, 6}, {0xA8B098E1, 9}, {0xA8791709, 9}, {0xA841B9AD, 9}, 
	{0xA80A80A9, 9}, {0x14FA6D7B, 6}, {0x14F38F63, 6}, {0x29D96B91, 7}, 
	{0x5397829D, 8}, {0xA6F87FD7, 9}, {0x14D843BF, 6}, {0xA68BDF79, 9}, 
	{0x532AE21D, 8}, {0xA61FCC17, 9}, {0x52F4FB77, 8}, {0xA5B4449D, 9}, 
	{0xA57EB503, 9}, {0x52A4A3FF, 8}, {0x2944FF5B, 7}, {0xA4DED52D, 9}, 
	{0x5254E78F, 8}, {0x0523A759, 4}, {0xA4402911, 9}, {0x0A40B88D, 5}, 
	{0x51EB851F, 8}, {0x51D1569D, 8}, {0x28DB9C69, 7}, {0xA33A575B, 9}, 
	{0x028C1979, 3}, {0x28B4A18D, 7}, {0xA29ECF17, 9}, {0xA26B38C9, 9}, 
	{0x288DF0CB, 7}, {0x05102371, 4}, {0x50E89CC3, 8}, {0x02867895, 3}, 
	{0xA16B312F, 9}, {0xA1385D35, 9}, {0x28416A4D, 7}, {0x2834C543, 7}, 
	{0xA0A0A0A1, 9}, {0xA06E4BD5, 9}, {0xA03C1689, 9}, {0xA00A00A1, 9}, 
	{0x4FEC04FF, 8}, {0x27E98CA1, 7}, {0x4FBA3D0B, 8}, {0x4FA1704B, 8}, 
	{0x13E22CBD, 6}, {0x9EE009EF, 9}, {0x4F576647, 8}, {0x9E7DADA9, 9}, 
	{0x27932B49, 7}, {0x9E1BCAE3, 9}, {0x4EF58365, 8}, {0x276E982F, 7}, 
	{0x4EC4EC4F, 8}, {0x9D596E55, 9}, {0x274A4871, 7}, {0x4E7C7969, 8}, 
	{0x9CC8E161, 9}, {0x13931DAB, 6}, {0x271A45A7, 7}, {0x9C395D11, 9}, 
	{0x9C09C09D, 9}, {0x9BDA4125, 9}, {0x9BAADE8F, 9}, {0x026DEE63, 3}, 
	{0x04DA637D, 4}, {0x4D8EB189, 8}, {0x4D77397F, 8}, {0x9ABF9F49, 9}, 
	{0x4D4873ED, 8}, {0x9A624C97, 9}, {0x134679AD, 6}, {0x9A056A31, 9}, 
	{0x99D722DB, 9}, {0x266A3DD3, 7}, {0x265EB9DB, 7}, {0x994CF321, 9}, 
	{0x4C8F8D29, 8}, {0x98F15CE7, 9}, {0x13187759, 6}, {0x26258CF7, 7}, 
	{0x4C346405, 8}, {0x983B773B, 9}, {0x980E4157, 9}, {0x97E12645, 9}, 
	{0x4BDA12F7, 8}, {0x4BC3A01D, 8}, {0x0975A751, 5}, {0x25CB7117, 7}, 
	{0x97012E03, 9}, {0x096D4B1F, 5}, {0x4B542805, 8}, {0x967C083B, 9}, 
	{0x964FDA6D, 9}, {0x9623C687, 9}, {0x95F7CC73, 9}, {0x2572FB07, 7}, 
	{0x95A02569, 9}, {0x255D1E11, 7}, {0x12A91C93, 6}, {0x4A8EB527, 8}, 
	{0x094F2095, 5}, {0x94C6C187, 9}, {0x4A4DC96F, 8}, {0x0251C1F5, 3}, 
	{0x94458095, 9}, {0x941A9CC9, 9}, {0x49F7E8E3, 8}, {0x49E28FBB, 8}, 
	{0x939A85C5, 9}, {0x24DC0127, 7}, {0x93459BE7, 9}, {0x931B4B91, 9}, 
	{0x92F11385, 9}, {0x92C6F3AD, 9}, {0x929CEBF5, 9}, {0x9272FC49, 9}, 
	{0x92492493, 9}, {0x921F64BF, 9}, {0x91F5BCB9, 9}, {0x91CC2C6D, 9}, 
	{0x91A2B3C5, 9}, {0x917952AF, 9}, {0x48A8048B, 8}, {0x9126D6E5, 9}, 
	{0x487EDE05, 8}, {0x090D4B87, 5}, {0x90ABCC03, 9}, {0x09082F6B, 5}, 
	{0x24168E19, 7}, {0x4818C885, 8}, {0x90090091, 9}, {0x8FE086E3, 9}, 
	{0x8FB823EF, 9}, {0x047C7EBD, 4}, {0x23D9E879, 7}, {0x8F3F82A9, 9}, 
	{0x478BBCED, 8}, {0x4777C3B3, 8}, {0x4763D59D, 8}, {0x8E9FE543, 9}, 
	{0x473C1AB7, 8}, {0x8E509BA9, 9}, {0x47148BF1, 8}, {0x4700D503, 8}, 
	{0x8DDA5203, 9}, {0x8DB30FC7, 9}, {0x02362F8D, 3}, {0x23593317, 7}, 
	{0x8D3DCB09, 9}, {0x468B6F9B, 8}, {0x08CF008D, 5}, {0x8CC947C5, 9}, 
	{0x8CA29C05, 9}, {0x8C7C057D, 9}, {0x8C55841D, 9}, {0x8C2F17D3, 9}, 
	{0x8C08C08D, 9}, {0x45F13F1D, 8}, {0x8BBC50C9, 9}, {0x45CB1C15, 8}, 
	{0x8B70344B, 9}, {0x45A5228D, 8}, {0x11648D51, 6}, {0x22BFA921, 7}, 
	{0x22B63CBF, 7}, {0x8AB355E1, 9}, {0x04546E69, 4}, {0x8A6858AB, 9}, 
	{0x8A42F871, 9}, {0x8A1DAC61, 9}, {0x44FC3A35, 8}, {0x89D3507D, 9}, 
	{0x44D72045, 8}, {0x01131289, 2}, {0x089645C5, 5}, {0x893F87E9, 9}, 
	{0x891AC73B, 9}, {0x111EC347, 6}, {0x4468C067, 8}, {0x44567D77, 8}, 
	{0x88888889, 9}, {0x8864298F, 9}, {0x883FDDF1, 9}, {0x440DD2CF, 8}, 
	{0x10FEF011, 6}, {0x043E9B75, 4}, {0x43D7B7EB, 8}, {0x878B841B, 9}, 
	{0x043B3D5B, 4}, {0x8743E595, 9}, {0x872032AD, 9}, {0x86FC9297, 9}, 
	{0x86D90545, 9}, {0x10D6B155, 6}, {0x43491159, 8}, {0x219BB355, 7}, 
	{0x4325C53F, 8}, {0x218A1689, 7}, {0x86053C35, 9}, {0x21788C29, 7}, 
	{0x42DF9BB1, 8}, {0x859C5061, 9}, {0x42BCBDC9, 8}, {0x10AAD71D, 6}, 
	{0x214D0215, 7}, {0x10A22D39, 6}, {0x42776E9B, 8}, {0x084CC629, 5}, 
	{0x84A9F9C9, 9}, {0x8487A2D1, 9}, {0x21195767, 7}, {0x2110CA87, 7}, 
	{0x84210843, 9}, {0x83FEF803, 9}, {0x41EE7CA7, 8}, {0x10776183, 6}, 
	{0x83993053, 9}, {0x837765F1, 9}, {0x20D56B39, 7}, {0x83340521, 9}, 
	{0x10624DD3, 6}, {0x4178749F, 8}, {0x20B3DD41, 7}, {0x415708EF, 8}, 
	{0x828CBFBF, 9}, {0x4135BF4D, 8}, {0x824A4E61, 9}, {0x104525E1, 6}, 
	{0x82082083, 9}, {0x81E722C3, 9}, {0x81C635BD, 9}, {0x81A55963, 9}, 
	{0x81848DA9, 9}, {0x8163D283, 9}, {0x2050C9F9, 7}, {0x81228DBF, 9}, 
	{0x81020409, 9}, {0x80E18AB3, 9}, {0x80C121B3, 9}, {0x2028323F, 7}, 
	{0x80808081, 9}, {0x80604837, 9}, {0x80402011, 9}, {0x80200803, 9}, 
	{0x80000001, 9}};

  FPReal<int32_t, int64_t, uint32_t, T4, false, false> tmp;
  union {
    int64_t v;
    int32_t p[2];
  } m;

  if ((divider > 1024) || (divider <= 0))
    {
       tmp = fpf / FPReal<int32_t, int64_t, uint32_t, T4, false, false>(divider, false);
    }
  else
    {
      int32_t v = fpf.getValue();
      m.v = (int64_t) v * (int64_t) magic_int32[divider][0];
      m.p[1] = (m.p[1]) >> (int32_t) magic_int32[divider][1]; 
      m.p[1] += v >> 31;
      tmp.assign(m.p[1]);
    }
  return tmp;
}


/*

template <typename int32_t, typename uint64_t, typename uint32_t, int T4> 
inline FPFloat<uint32_t, uint64_t, uint32_t, T4, false, false> operator/ (const FPFloat<uint32_t, uint64_t, uint32_t, T4, false, false> &fpf, const uint32_t &divider) {
  static const uint32_t magic_uint32[][3] = {
	{0x00000000, 0, 0}, {0xFFFFFFFF, 0, 0}, {0x80000000, 0, 0}, {0xAAAAAAAB, 1, 0}, 
	{0x40000000, 0, 0}, {0xCCCCCCCD, 2, 0}, {0xAAAAAAAB, 2, 0}, {0x24924925, 2, 1}, 
	{0x20000000, 0, 0}, {0x38E38E39, 1, 0}, {0xCCCCCCCD, 3, 0}, {0xBA2E8BA3, 3, 0}, 
	{0xAAAAAAAB, 3, 0}, {0x4EC4EC4F, 2, 0}, {0x24924925, 3, 1}, {0x88888889, 3, 0}, 
	{0x10000000, 0, 0}, {0xF0F0F0F1, 4, 0}, {0x38E38E39, 2, 0}, {0xAF286BCB, 4, 1}, 
	{0xCCCCCCCD, 4, 0}, {0x86186187, 4, 1}, {0xBA2E8BA3, 4, 0}, {0xB21642C9, 4, 0}, 
	{0xAAAAAAAB, 4, 0}, {0x51EB851F, 3, 0}, {0x4EC4EC4F, 3, 0}, {0x2F684BDB, 4, 1}, 
	{0x24924925, 4, 1}, {0x8D3DCB09, 4, 0}, {0x88888889, 4, 0}, {0x08421085, 4, 1}, 
	{0x08000000, 0, 0}, {0x3E0F83E1, 3, 0}, {0xF0F0F0F1, 5, 0}, {0xD41D41D5, 5, 1}, 
	{0x38E38E39, 3, 0}, {0xBACF914D, 5, 1}, {0xAF286BCB, 5, 1}, {0xA41A41A5, 5, 1}, 
	{0xCCCCCCCD, 5, 0}, {0xC7CE0C7D, 5, 0}, {0x86186187, 5, 1}, {0x2FA0BE83, 3, 0}, 
	{0xBA2E8BA3, 5, 0}, {0x6C16C16D, 5, 1}, {0xB21642C9, 5, 0}, {0xAE4C415D, 5, 0}, 
	{0xAAAAAAAB, 5, 0}, {0x5397829D, 4, 0}, {0x51EB851F, 4, 0}, {0xA0A0A0A1, 5, 0}, 
	{0x4EC4EC4F, 4, 0}, {0x3521CFB3, 5, 1}, {0x2F684BDB, 5, 1}, {0x29E4129F, 5, 1}, 
	{0x24924925, 5, 1}, {0x1F7047DD, 5, 1}, {0x8D3DCB09, 5, 0}, {0x22B63CBF, 3, 0}, 
	{0x88888889, 5, 0}, {0x4325C53F, 4, 0}, {0x08421085, 5, 1}, {0x04104105, 5, 1}, 
	{0x04000000, 0, 0}, {0xFC0FC0FD, 6, 0}, {0x3E0F83E1, 4, 0}, {0x07A44C6B, 1, 0}, 
	{0xF0F0F0F1, 6, 0}, {0x76B981DB, 5, 0}, {0xD41D41D5, 6, 1}, {0xE6C2B449, 6, 0}, 
	{0x38E38E39, 4, 0}, {0xC0E07039, 6, 1}, {0xBACF914D, 6, 1}, {0x1B4E81B5, 3, 0}, 
	{0xAF286BCB, 6, 1}, {0x3531DEC1, 4, 0}, {0xA41A41A5, 6, 1}, {0xCF6474A9, 6, 0}, 
	{0xCCCCCCCD, 6, 0}, {0xCA4587E7, 6, 0}, {0xC7CE0C7D, 6, 0}, {0x3159721F, 4, 0}, 
	{0x86186187, 6, 1}, {0xC0C0C0C1, 6, 0}, {0x2FA0BE83, 4, 0}, {0x2F149903, 4, 0}, 
	{0xBA2E8BA3, 6, 0}, {0xB81702E1, 6, 0}, {0x6C16C16D, 6, 1}, {0x68168169, 6, 1}, 
	{0xB21642C9, 6, 0}, {0xB02C0B03, 6, 0}, {0xAE4C415D, 6, 0}, {0x58ED2309, 6, 1}, 
	{0xAAAAAAAB, 6, 0}, {0x51D07EAF, 6, 1}, {0x5397829D, 5, 0}, {0xA57EB503, 6, 0}, 
	{0x51EB851F, 5, 0}, {0x446F8657, 6, 1}, {0xA0A0A0A1, 6, 0}, {0x3E22CBCF, 6, 1}, 
	{0x4EC4EC4F, 5, 0}, {0x38138139, 6, 1}, {0x3521CFB3, 6, 1}, {0x323E34A3, 6, 1}, 
	{0x2F684BDB, 6, 1}, {0x2C9FB4D9, 6, 1}, {0x29E4129F, 6, 1}, {0x27350B89, 6, 1}, 
	{0x24924925, 6, 1}, {0x21FB7813, 6, 1}, {0x1F7047DD, 6, 1}, {0x1CF06ADB, 6, 1}, 
	{0x8D3DCB09, 6, 0}, {0x18118119, 6, 1}, {0x22B63CBF, 4, 0}, {0x44D72045, 5, 0}, 
	{0x88888889, 6, 0}, {0x0ECF56BF, 6, 1}, {0x4325C53F, 5, 0}, {0x0A6810A7, 6, 1}, 
	{0x08421085, 6, 1}, {0x10624DD3, 3, 0}, {0x04104105, 6, 1}, {0x02040811, 6, 1}, 
	{0x02000000, 0, 0}, {0x0FE03F81, 3, 0}, {0xFC0FC0FD, 7, 0}, {0xFA232CF3, 7, 0}, 
	{0x3E0F83E1, 5, 0}, {0xF6603D99, 7, 0}, {0x07A44C6B, 2, 0}, {0xF2B9D649, 7, 0}, 
	{0xF0F0F0F1, 7, 0}, {0x077975B9, 2, 0}, {0x76B981DB, 6, 0}, {0x75DED953, 6, 0}, 
	{0xD41D41D5, 7, 1}, {0x3A196B1F, 5, 0}, {0xE6C2B449, 7, 0}, {0xE525982B, 7, 0}, 
	{0x38E38E39, 5, 0}, {0xE1FC780F, 7, 0}, {0xC0E07039, 7, 1}, {0xDEE95C4D, 7, 0}, 
	{0xBACF914D, 7, 1}, {0xDBEB61EF, 7, 0}, {0x1B4E81B5, 4, 0}, {0x36406C81, 5, 0}, 
	{0xAF286BCB, 7, 1}, {0xD62B80D7, 7, 0}, {0x3531DEC1, 5, 0}, {0xD3680D37, 7, 0}, 
	{0xA41A41A5, 7, 1}, {0x342DA7F3, 5, 0}, {0xCF6474A9, 7, 0}, {0x9C2D14EF, 7, 1}, 
	{0xCCCCCCCD, 7, 0}, {0xCB8727C1, 7, 0}, {0xCA4587E7, 7, 0}, {0xC907DA4F, 7, 0}, 
	{0xC7CE0C7D, 7, 0}, {0x634C0635, 6, 0}, {0x3159721F, 5, 0}, {0x621B97C3, 6, 0}, 
	{0x86186187, 7, 1}, {0x60F25DEB, 6, 0}, {0xC0C0C0C1, 7, 0}, {0x7F405FD1, 7, 1}, 
	{0x2FA0BE83, 5, 0}, {0x7AD2208F, 7, 1}, {0x2F149903, 5, 0}, {0x5D9F7391, 6, 0}, 
	{0xBA2E8BA3, 7, 0}, {0x724287F5, 7, 1}, {0xB81702E1, 7, 0}, {0x6E1F76B5, 7, 1}, 
	{0x6C16C16D, 7, 1}, {0xB509E68B, 7, 0}, {0x68168169, 7, 1}, {0xB30F6353, 7, 0}, 
	{0xB21642C9, 7, 0}, {0x623FA771, 7, 1}, {0xB02C0B03, 7, 0}, {0xAF3ADDC7, 7, 0}, 
	{0xAE4C415D, 7, 0}, {0x5AC056B1, 7, 1}, {0x58ED2309, 7, 1}, {0xAB8F69E3, 7, 0}, 
	{0xAAAAAAAB, 7, 0}, {0x15390949, 4, 0}, {0x51D07EAF, 7, 1}, {0x50150151, 7, 1}, 
	{0x5397829D, 6, 0}, {0x4CAB8873, 7, 1}, {0xA57EB503, 7, 0}, {0x5254E78F, 6, 0}, 
	{0x51EB851F, 6, 0}, {0x028C1979, 1, 0}, {0x446F8657, 7, 1}, {0xA16B312F, 7, 0}, 
	{0xA0A0A0A1, 7, 0}, {0x4FEC04FF, 6, 0}, {0x3E22CBCF, 7, 1}, {0x27932B49, 5, 0}, 
	{0x4EC4EC4F, 6, 0}, {0x9CC8E161, 7, 0}, {0x38138139, 7, 1}, {0x9B4C6F9F, 7, 0}, 
	{0x3521CFB3, 7, 1}, {0x99D722DB, 7, 0}, {0x323E34A3, 7, 1}, {0x4C346405, 6, 0}, 
	{0x2F684BDB, 7, 1}, {0x2E025C05, 7, 1}, {0x2C9FB4D9, 7, 1}, {0x2B404AD1, 7, 1}, 
	{0x29E4129F, 7, 1}, {0x288B0129, 7, 1}, {0x27350B89, 7, 1}, {0x25E22709, 7, 1}, 
	{0x24924925, 7, 1}, {0x91A2B3C5, 7, 0}, {0x21FB7813, 7, 1}, {0x20B470C7, 7, 1}, 
	{0x1F7047DD, 7, 1}, {0x478BBCED, 6, 0}, {0x1CF06ADB, 7, 1}, {0x1BB4A405, 7, 1}, 
	{0x8D3DCB09, 7, 0}, {0x19453809, 7, 1}, {0x18118119, 7, 1}, {0x16E06895, 7, 1}, 
	{0x22B63CBF, 5, 0}, {0x1485F0E1, 7, 1}, {0x44D72045, 6, 0}, {0x891AC73B, 7, 0}, 
	{0x88888889, 7, 0}, {0x10FEF011, 4, 0}, {0x0ECF56BF, 7, 1}, {0x86D90545, 7, 0}, 
	{0x4325C53F, 6, 0}, {0x0B7E6EC3, 7, 1}, {0x0A6810A7, 7, 1}, {0x0953F391, 7, 1}, 
	{0x08421085, 7, 1}, {0x073260A5, 7, 1}, {0x10624DD3, 4, 0}, {0x828CBFBF, 7, 0}, 
	{0x04104105, 7, 1}, {0x81848DA9, 7, 0}, {0x02040811, 7, 1}, {0x80808081, 7, 0}, 
	{0x01000000, 0, 0}, {0xFF00FF01, 8, 0}, {0x0FE03F81, 4, 0}, {0xFD08E551, 8, 0}, 
	{0xFC0FC0FD, 8, 0}, {0x7D8C42B3, 7, 0}, {0xFA232CF3, 8, 0}, {0x7C97D911, 7, 0}, 
	{0x3E0F83E1, 6, 0}, {0xF74E3FC3, 8, 0}, {0xF6603D99, 8, 0}, {0x7ABA01EB, 7, 0}, 
	{0x07A44C6B, 3, 0}, {0xF3A0D52D, 8, 0}, {0xF2B9D649, 8, 0}, {0xF1D48BCF, 8, 0}, 
	{0xF0F0F0F1, 8, 0}, {0xE01E01E1, 8, 1}, {0x077975B9, 3, 0}, {0xDCA01DCB, 8, 1}, 
	{0x76B981DB, 7, 0}, {0xEC979119, 8, 0}, {0x75DED953, 7, 0}, {0x3AB95901, 6, 0}, 
	{0xD41D41D5, 8, 1}, {0x0749CB29, 3, 0}, {0x3A196B1F, 6, 0}, {0xE79372E3, 8, 0}, 
	{0xE6C2B449, 8, 0}, {0xCBE6D961, 8, 1}, {0xE525982B, 8, 0}, {0x1C8B265B, 5, 0}, 
	{0x38E38E39, 6, 0}, {0xE2C4A689, 8, 0}, {0xE1FC780F, 8, 0}, {0x709AD4E5, 7, 0}, 
	{0xC0E07039, 8, 1}, {0xDFAC1F75, 8, 0}, {0xDEE95C4D, 8, 0}, {0xDE27EB2D, 8, 0}, 
	{0xBACF914D, 8, 1}, {0xDCA8F159, 8, 0}, {0xDBEB61EF, 8, 0}, {0x6D978B8F, 7, 0}, 
	{0x1B4E81B5, 5, 0}, {0xD9BA4257, 8, 0}, {0x36406C81, 6, 0}, {0xD84A598F, 8, 0}, 
	{0xAF286BCB, 8, 1}, {0xD6DF43FD, 8, 0}, {0xD62B80D7, 8, 0}, {0xD578E97D, 8, 0}, 
	{0x3531DEC1, 6, 0}, {0x6A0B9945, 7, 0}, {0xD3680D37, 8, 0}, {0x34AE820F, 6, 0}, 
	{0xA41A41A5, 8, 1}, {0xA2C2A87D, 8, 1}, {0x342DA7F3, 6, 0}, {0xA01A01A1, 8, 1}, 
	{0xCF6474A9, 8, 0}, {0x33AF3E2F, 6, 0}, {0x9C2D14EF, 8, 1}, {0xCD712753, 8, 0}, 
	{0xCCCCCCCD, 8, 0}, {0xCC29786D, 8, 0}, {0xCB8727C1, 8, 0}, {0x95CBB0BF, 8, 1}, 
	{0xCA4587E7, 8, 0}, {0xC9A633FD, 8, 0}, {0xC907DA4F, 8, 0}, {0x90D4F121, 8, 1}, 
	{0xC7CE0C7D, 8, 0}, {0x18E6527B, 5, 0}, {0x634C0635, 7, 0}, {0x8BFCE807, 8, 1}, 
	{0x3159721F, 6, 0}, {0x899C0F61, 8, 1}, {0x621B97C3, 7, 0}, {0x87427BCD, 8, 1}, 
	{0x86186187, 8, 1}, {0x309E0185, 6, 0}, {0x60F25DEB, 7, 0}, {0x82A4A019, 8, 1}, 
	{0xC0C0C0C1, 8, 0}, {0x80601807, 8, 1}, {0x7F405FD1, 8, 1}, {0xBF112A8B, 8, 0}, 
	{0x2FA0BE83, 6, 0}, {0x5EFACE49, 7, 0}, {0x7AD2208F, 8, 1}, {0x5E6EA9AF, 7, 0}, 
	{0x2F149903, 6, 0}, {0xBBC8408D, 8, 0}, {0x5D9F7391, 7, 0}, {0x756CAC21, 8, 1}, 
	{0xBA2E8BA3, 8, 0}, {0x734F0C55, 8, 1}, {0x724287F5, 8, 1}, {0xB89BC36D, 8, 0}, 
	{0xB81702E1, 8, 0}, {0x16F26017, 5, 0}, {0x6E1F76B5, 8, 1}, {0x6D1A6269, 8, 1}, 
	{0x6C16C16D, 8, 1}, {0x6B1490AB, 8, 1}, {0xB509E68B, 8, 0}, {0x691473A9, 8, 1}, 
	{0x68168169, 8, 1}, {0x6719F361, 8, 1}, {0xB30F6353, 8, 0}, {0x59493E15, 7, 0}, 
	{0xB21642C9, 8, 0}, {0xB19AB5C5, 8, 0}, {0x623FA771, 8, 1}, {0x5852CDA1, 7, 0}, 
	{0xB02C0B03, 8, 0}, {0x5F664343, 8, 1}, {0xAF3ADDC7, 8, 0}, {0x057619F1, 3, 0}, 
	{0xAE4C415D, 8, 0}, {0x5BABCC65, 8, 1}, {0x5AC056B1, 8, 1}, {0x59D61F13, 8, 1}, 
	{0x58ED2309, 8, 1}, {0xAC02B00B, 8, 0}, {0xAB8F69E3, 8, 0}, {0x2AC72F75, 6, 0}, 
	{0xAAAAAAAB, 8, 0}, {0x551C979B, 7, 0}, {0x15390949, 5, 0}, {0x52AFF56B, 8, 1}, 
	{0x51D07EAF, 8, 1}, {0xA8791709, 8, 0}, {0x50150151, 8, 1}, {0xA79C7B17, 8, 0}, 
	{0x5397829D, 7, 0}, {0xA6C21DF7, 8, 0}, {0x4CAB8873, 8, 1}, {0x4BD3EDDB, 8, 1}, 
	{0xA57EB503, 8, 0}, {0x2944FF5B, 6, 0}, {0x5254E78F, 7, 0}, {0x48805221, 8, 1}, 
	{0x51EB851F, 7, 0}, {0xA36E71A3, 8, 0}, {0x028C1979, 2, 0}, {0x453D9E2D, 8, 1}, 
	{0x446F8657, 8, 1}, {0x50E89CC3, 7, 0}, {0xA16B312F, 8, 0}, {0xA105A933, 8, 0}, 
	{0xA0A0A0A1, 8, 0}, {0xA03C1689, 8, 0}, {0x4FEC04FF, 7, 0}, {0x3EE8F42B, 8, 1}, 
	{0x3E22CBCF, 8, 1}, {0x3D5D991B, 8, 1}, {0x27932B49, 6, 0}, {0x3BD60D93, 8, 1}, 
	{0x4EC4EC4F, 7, 0}, {0x274A4871, 6, 0}, {0x9CC8E161, 8, 0}, {0x38D22D37, 8, 1}, 
	{0x38138139, 8, 1}, {0x3755BD1D, 8, 1}, {0x9B4C6F9F, 8, 0}, {0x9AEE72FD, 8, 0}, 
	{0x3521CFB3, 8, 1}, {0x34679ACF, 8, 1}, {0x99D722DB, 8, 0}, {0x32F5CED7, 8, 1}, 
	{0x323E34A3, 8, 1}, {0x3187758F, 8, 1}, {0x4C346405, 7, 0}, {0x301C82AD, 8, 1}, 
	{0x2F684BDB, 8, 1}, {0x0975A751, 4, 0}, {0x2E025C05, 8, 1}, {0x4B542805, 7, 0}, 
	{0x2C9FB4D9, 8, 1}, {0x95F7CC73, 8, 0}, {0x2B404AD1, 8, 1}, {0x12A91C93, 5, 0}, 
	{0x29E4129F, 8, 1}, {0x4A4DC96F, 7, 0}, {0x288B0129, 8, 1}, {0x27DFA38B, 8, 1}, 
	{0x27350B89, 8, 1}, {0x93459BE7, 8, 0}, {0x25E22709, 8, 1}, {0x929CEBF5, 8, 0}, 
	{0x24924925, 8, 1}, {0x91F5BCB9, 8, 0}, {0x91A2B3C5, 8, 0}, {0x22A0122B, 8, 1}, 
	{0x21FB7813, 8, 1}, {0x21579805, 8, 1}, {0x20B470C7, 8, 1}, {0x20120121, 8, 1}, 
	{0x1F7047DD, 8, 1}, {0x23D9E879, 6, 0}, {0x478BBCED, 7, 0}, {0x4763D59D, 7, 0}, 
	{0x1CF06ADB, 8, 1}, {0x8E2917E1, 8, 0}, {0x1BB4A405, 8, 1}, {0x02362F8D, 2, 0}, 
	{0x8D3DCB09, 8, 0}, {0x19E0119F, 8, 1}, {0x19453809, 8, 1}, {0x8C55841D, 8, 0}, 
	{0x18118119, 8, 1}, {0x8BBC50C9, 8, 0}, {0x16E06895, 8, 1}, {0x11648D51, 5, 0}, 
	{0x22B63CBF, 6, 0}, {0x04546E69, 3, 0}, {0x1485F0E1, 8, 1}, {0x44FC3A35, 7, 0}, 
	{0x44D72045, 7, 0}, {0x12C8B89F, 8, 1}, {0x891AC73B, 8, 0}, {0x11A3019B, 8, 1}, 
	{0x88888889, 8, 0}, {0x107FBBE1, 8, 1}, {0x10FEF011, 5, 0}, {0x43D7B7EB, 7, 0}, 
	{0x0ECF56BF, 8, 1}, {0x0E406559, 8, 1}, {0x86D90545, 8, 0}, {0x43491159, 7, 0}, 
	{0x4325C53F, 7, 0}, {0x0C0A7869, 8, 1}, {0x0B7E6EC3, 8, 1}, {0x0AF2F723, 8, 1}, 
	{0x0A6810A7, 8, 1}, {0x42776E9B, 7, 0}, {0x0953F391, 8, 1}, {0x21195767, 6, 0}, 
	{0x08421085, 8, 1}, {0x41EE7CA7, 7, 0}, {0x073260A5, 8, 1}, {0x20D56B39, 6, 0}, 
	{0x10624DD3, 5, 0}, {0x20B3DD41, 6, 0}, {0x828CBFBF, 8, 0}, {0x824A4E61, 8, 0}, 
	{0x04104105, 8, 1}, {0x038C6B79, 8, 1}, {0x81848DA9, 8, 0}, {0x2050C9F9, 6, 0}, 
	{0x02040811, 8, 1}, {0x80C121B3, 8, 0}, {0x80808081, 8, 0}, {0x00804021, 8, 1}, 
	{0x00800000, 0, 0}, {0xFF803FE1, 9, 0}, {0xFF00FF01, 9, 0}, {0x7F411E53, 8, 0}, 
	{0x0FE03F81, 5, 0}, {0xFD863087, 9, 0}, {0xFD08E551, 9, 0}, {0xFC8C15B5, 9, 0}, 
	{0xFC0FC0FD, 9, 0}, {0xFB93E673, 9, 0}, {0x7D8C42B3, 8, 0}, {0x07D4ECE9, 4, 0}, 
	{0xFA232CF3, 9, 0}, {0xF9A9342D, 9, 0}, {0x7C97D911, 8, 0}, {0xF16D4C45, 9, 1}, 
	{0x3E0F83E1, 7, 0}, {0xF7C5ED9D, 9, 0}, {0xF74E3FC3, 9, 0}, {0x7B6B82A7, 8, 0}, 
	{0xF6603D99, 9, 0}, {0xF5E9E7FD, 9, 0}, {0x7ABA01EB, 8, 0}, {0xF4FE9083, 9, 0}, 
	{0x07A44C6B, 4, 0}, {0x7A0A7CE7, 8, 0}, {0xF3A0D52D, 9, 0}, {0x079968F7, 4, 0}, 
	{0xF2B9D649, 9, 0}, {0x3C91BEB3, 7, 0}, {0xF1D48BCF, 9, 0}, {0xF16288B9, 9, 0}, 
	{0xF0F0F0F1, 9, 0}, {0xE0FF87C1, 9, 1}, {0xE01E01E1, 9, 1}, {0x3BE7A9E3, 7, 0}, 
	{0x077975B9, 4, 0}, {0xEEBF2F19, 9, 0}, {0xDCA01DCB, 9, 1}, {0x3B78557D, 7, 0}, 
	{0x76B981DB, 8, 0}, {0xDA0A2F39, 9, 1}, {0xEC979119, 9, 0}, {0xD854DF41, 9, 1}, 
	{0x75DED953, 8, 0}, {0x075A8ACD, 4, 0}, {0x3AB95901, 7, 0}, {0x753CE8A5, 8, 0}, 
	{0xD41D41D5, 9, 1}, {0xD347A4BD, 9, 1}, {0x0749CB29, 4, 0}, {0xE8CF58AB, 9, 0}, 
	{0x3A196B1F, 7, 0}, {0xE7FC600F, 9, 0}, {0xE79372E3, 9, 0}, {0x7395723B, 8, 0}, 
	{0xE6C2B449, 9, 0}, {0x3996B877, 7, 0}, {0xCBE6D961, 9, 1}, {0x72C62A25, 8, 0}, 
	{0xE525982B, 9, 0}, {0xE4BF37D9, 9, 0}, {0x1C8B265B, 6, 0}, {0xE3F388AF, 9, 0}, 
	{0x38E38E39, 7, 0}, {0xE32942FF, 9, 0}, {0xE2C4A689, 9, 0}, {0xE260630B, 9, 0}, 
	{0xE1FC780F, 9, 0}, {0x070CC729, 4, 0}, {0x709AD4E5, 8, 0}, {0x706962CD, 8, 0}, 
	{0xC0E07039, 9, 1}, {0xC01C01C1, 9, 1}, {0xDFAC1F75, 9, 0}, {0xDF4A9369, 9, 0}, 
	{0xDEE95C4D, 9, 0}, {0xDE8879B3, 9, 0}, {0xDE27EB2D, 9, 0}, {0xDDC7B04D, 9, 0}, 
	{0xBACF914D, 9, 1}, {0x6E8419E7, 8, 0}, {0xDCA8F159, 9, 0}, {0xDC4A00DD, 9, 0}, 
	{0xDBEB61EF, 9, 0}, {0x1B71A285, 6, 0}, {0x6D978B8F, 8, 0}, {0xDAD16A6B, 9, 0}, 
	{0x1B4E81B5, 6, 0}, {0xB42E00DB, 9, 1}, {0xD9BA4257, 9, 0}, {0x00D95DD3, 1, 0}, 
	{0x36406C81, 7, 0}, {0xD8A5DEFF, 9, 0}, {0xD84A598F, 9, 0}, {0x6BF790A9, 8, 0}, 
	{0xAF286BCB, 9, 1}, {0xD73996E9, 9, 0}, {0xD6DF43FD, 9, 0}, {0xD6853CC1, 9, 0}, 
	{0xD62B80D7, 9, 0}, {0xD5D20FDF, 9, 0}, {0xD578E97D, 9, 0}, {0xAA401AA5, 9, 1}, 
	{0x3531DEC1, 7, 0}, {0xD46F3235, 9, 0}, {0x6A0B9945, 8, 0}, {0xD3BF7BA9, 9, 0}, 
	{0xD3680D37, 9, 0}, {0xD310E6DB, 9, 0}, {0x34AE820F, 7, 0}, {0xD2637101, 9, 0}, 
	{0xA41A41A5, 9, 1}, {0xD1B71759, 9, 0}, {0xA2C2A87D, 9, 1}, {0x3442F5CB, 7, 0}, 
	{0x342DA7F3, 7, 0}, {0x6830D6E5, 8, 0}, {0xA01A01A1, 9, 1}, {0x33EE2623, 7, 0}, 
	{0xCF6474A9, 9, 0}, {0x33C42535, 7, 0}, {0x33AF3E2F, 7, 0}, {0xCE69A00D, 9, 0}, 
	{0x9C2D14EF, 9, 1}, {0x9B876F53, 9, 1}, {0xCD712753, 9, 0}, {0x3347B649, 7, 0}, 
	{0xCCCCCCCD, 9, 0}, {0x00663D81, 0, 0}, {0xCC29786D, 9, 0}, {0xCBD82FC7, 9, 0}, 
	{0xCB8727C1, 9, 0}, {0xCB36600D, 9, 0}, {0x95CBB0BF, 9, 1}, {0x32A5641B, 7, 0}, 
	{0xCA4587E7, 9, 0}, {0x64FADF43, 8, 0}, {0xC9A633FD, 9, 0}, {0x92ADD007, 9, 1}, 
	{0xC907DA4F, 9, 0}, {0x645C854B, 8, 0}, {0x90D4F121, 9, 1}, {0x903847EB, 9, 1}, 
	{0xC7CE0C7D, 9, 0}, {0x8F0063C1, 9, 1}, {0x18E6527B, 6, 0}, {0xC6E5321D, 9, 0}, 
	{0x634C0635, 8, 0}, {0x8C9644F1, 9, 1}, {0x8BFCE807, 9, 1}, {0x62D90063, 8, 0}, 
	{0x3159721F, 7, 0}, {0x8A3395C1, 9, 1}, {0x899C0F61, 9, 1}, {0x8904FD51, 9, 1}, 
	{0x621B97C3, 8, 0}, {0x61F60D03, 8, 0}, {0x87427BCD, 9, 1}, {0x61AB4D73, 8, 0}, 
	{0x86186187, 9, 1}, {0x6160FF9F, 8, 0}, {0x309E0185, 7, 0}, {0xC22E4507, 9, 0}, 
	{0x60F25DEB, 8, 0}, {0x60CDB521, 8, 0}, {0x82A4A019, 9, 1}, {0x6084B67B, 8, 0}, 
	{0xC0C0C0C1, 9, 0}, {0xC0784B2F, 9, 0}, {0x80601807, 9, 1}, {0x00BFE803, 1, 0}, 
	{0x7F405FD1, 9, 1}, {0x017EB125, 2, 0}, {0xBF112A8B, 9, 0}, {0x5F64FBE7, 8, 0}, 
	{0x2FA0BE83, 7, 0}, {0x2F8F0C43, 7, 0}, {0x5EFACE49, 8, 0}, {0x2F6BCF19, 7, 0}, 
	{0x7AD2208F, 9, 1}, {0xBD231803, 9, 0}, {0x5E6EA9AF, 8, 0}, {0x792F843D, 9, 1}, 
	{0x2F149903, 7, 0}, {0x781A71DD, 9, 1}, {0xBBC8408D, 9, 0}, {0xBB837AB1, 9, 0}, 
	{0x5D9F7391, 8, 0}, {0x75F50B53, 9, 1}, {0x756CAC21, 9, 1}, {0x74E4B041, 9, 1}, 
	{0xBA2E8BA3, 9, 0}, {0xB9EAF063, 9, 0}, {0x734F0C55, 9, 1}, {0x2E591331, 7, 0}, 
	{0x724287F5, 9, 1}, {0x5C6F35CD, 8, 0}, {0xB89BC36D, 9, 0}, {0xB8594B41, 9, 0}, 
	{0xB81702E1, 9, 0}, {0x6FA9D433, 9, 1}, {0x16F26017, 6, 0}, {0xB7514689, 9, 0}, 
	{0x6E1F76B5, 9, 1}, {0x6D9CBDF3, 9, 1}, {0x6D1A6269, 9, 1}, {0xB64C31D9, 9, 0}, 
	{0x6C16C16D, 9, 1}, {0xB5CABD9B, 9, 0}, {0x6B1490AB, 9, 1}, {0x6A94016B, 9, 1}, 
	{0xB509E68B, 9, 0}, {0xB4C9F9A5, 9, 0}, {0x691473A9, 9, 1}, {0x68954DD3, 9, 1}, 
	{0x68168169, 9, 1}, {0x59E60383, 8, 0}, {0x6719F361, 9, 1}, {0x2CD38621, 7, 0}, 
	{0xB30F6353, 9, 0}, {0xB2D0D9EF, 9, 0}, {0x59493E15, 8, 0}, {0xB25449D7, 9, 0}, 
	{0xB21642C9, 9, 0}, {0x63B0CDA3, 9, 1}, {0xB19AB5C5, 9, 0}, {0x58AE97BB, 8, 0}, 
	{0x623FA771, 9, 1}, {0x61C544C1, 9, 1}, {0x5852CDA1, 8, 0}, {0xB068BE31, 9, 0}, 
	{0xB02C0B03, 9, 0}, {0x2BFBE063, 7, 0}, {0x5F664343, 9, 1}, {0xAF76EB19, 9, 0}, 
	{0xAF3ADDC7, 9, 0}, {0x577F7CC1, 8, 0}, {0x057619F1, 4, 0}, {0x5D0F56ED, 9, 1}, 
	{0xAE4C415D, 9, 0}, {0xAE10FFA9, 9, 0}, {0x5BABCC65, 9, 1}, {0x0AD9AF4D, 5, 0}, 
	{0x5AC056B1, 9, 1}, {0x2B496269, 7, 0}, {0x59D61F13, 9, 1}, {0x596179C3, 9, 1}, 
	{0x58ED2309, 9, 1}, {0x561E46A5, 8, 0}, {0xAC02B00B, 9, 0}, {0x5791F341, 9, 1}, 
	{0xAB8F69E3, 9, 0}, {0x2AD5802B, 7, 0}, {0x2AC72F75, 7, 0}, {0x5571D09B, 8, 0}, 
	{0xAAAAAAAB, 9, 0}, {0xAA71DA0D, 9, 0}, {0x551C979B, 8, 0}, {0xAA00AA01, 9, 0}, 
	{0x15390949, 6, 0}, {0x54C807F3, 8, 0}, {0x52AFF56B, 9, 1}, {0x52401525, 9, 1}, 
	{0x51D07EAF, 9, 1}, {0x516131C1, 9, 1}, {0xA8791709, 9, 0}, {0xA841B9AD, 9, 0}, 
	{0x50150151, 9, 1}, {0x14FA6D7B, 6, 0}, {0xA79C7B17, 9, 0}, {0x29D96B91, 7, 0}, 
	{0x5397829D, 8, 0}, {0x4DF0FFAD, 9, 1}, {0xA6C21DF7, 9, 0}, {0xA68BDF79, 9, 0}, 
	{0x4CAB8873, 9, 1}, {0x4C3F982D, 9, 1}, {0x4BD3EDDB, 9, 1}, {0xA5B4449D, 9, 0}, 
	{0xA57EB503, 9, 0}, {0x52A4A3FF, 8, 0}, {0x2944FF5B, 7, 0}, {0x49BDAA59, 9, 1}, 
	{0x5254E78F, 8, 0}, {0x48E9D63F, 9, 1}, {0x48805221, 9, 1}, {0x0A40B88D, 5, 0}, 
	{0x51EB851F, 8, 0}, {0x47455A73, 9, 1}, {0xA36E71A3, 9, 0}, {0x4674AEB5, 9, 1}, 
	{0x028C1979, 3, 0}, {0x28B4A18D, 7, 0}, {0x453D9E2D, 9, 1}, {0xA26B38C9, 9, 0}, 
	{0x446F8657, 9, 1}, {0x4408DC3F, 9, 1}, {0x50E89CC3, 8, 0}, {0x02867895, 3, 0}, 
	{0xA16B312F, 9, 0}, {0xA1385D35, 9, 0}, {0xA105A933, 9, 0}, {0x2834C543, 7, 0}, 
	{0xA0A0A0A1, 9, 0}, {0x40DC97A9, 9, 1}, {0xA03C1689, 9, 0}, {0x40140141, 9, 1}, 
	{0x4FEC04FF, 8, 0}, {0x27E98CA1, 7, 0}, {0x3EE8F42B, 9, 1}, {0x3E85C12B, 9, 1}, 
	{0x3E22CBCF, 9, 1}, {0x3DC013DD, 9, 1}, {0x3D5D991B, 9, 1}, {0x9E7DADA9, 9, 0}, 
	{0x27932B49, 7, 0}, {0x9E1BCAE3, 9, 0}, {0x3BD60D93, 9, 1}, {0x3B74C177, 9, 1}, 
	{0x4EC4EC4F, 8, 0}, {0x3AB2DCA9, 9, 1}, {0x274A4871, 7, 0}, {0x39F1E5A3, 9, 1}, 
	{0x9CC8E161, 9, 0}, {0x13931DAB, 6, 0}, {0x38D22D37, 9, 1}, {0x3872BA21, 9, 1}, 
	{0x38138139, 9, 1}, {0x37B48249, 9, 1}, {0x3755BD1D, 9, 1}, {0x026DEE63, 3, 0}, 
	{0x9B4C6F9F, 9, 0}, {0x363AC623, 9, 1}, {0x9AEE72FD, 9, 0}, {0x357F3E91, 9, 1}, 
	{0x3521CFB3, 9, 1}, {0x9A624C97, 9, 0}, {0x34679ACF, 9, 1}, {0x9A056A31, 9, 0}, 
	{0x99D722DB, 9, 0}, {0x266A3DD3, 7, 0}, {0x32F5CED7, 9, 1}, {0x3299E641, 9, 1}, 
	{0x323E34A3, 9, 1}, {0x98F15CE7, 9, 0}, {0x3187758F, 9, 1}, {0x312C67B7, 9, 1}, 
	{0x4C346405, 8, 0}, {0x983B773B, 9, 0}, {0x301C82AD, 9, 1}, {0x2FC24C89, 9, 1}, 
	{0x2F684BDB, 9, 1}, {0x97874039, 9, 0}, {0x0975A751, 5, 0}, {0x972DC45B, 9, 0}, 
	{0x2E025C05, 9, 1}, {0x96D4B1EF, 9, 0}, {0x4B542805, 8, 0}, {0x967C083B, 9, 0}, 
	{0x2C9FB4D9, 9, 1}, {0x2C478D0D, 9, 1}, {0x95F7CC73, 9, 0}, {0x95CBEC1B, 9, 0}, 
	{0x2B404AD1, 9, 1}, {0x255D1E11, 7, 0}, {0x12A91C93, 6, 0}, {0x4A8EB527, 8, 0}, 
	{0x29E4129F, 9, 1}, {0x94C6C187, 9, 0}, {0x4A4DC96F, 8, 0}, {0x94707D3F, 9, 0}, 
	{0x288B0129, 9, 1}, {0x28353991, 9, 1}, {0x27DFA38B, 9, 1}, {0x49E28FBB, 8, 0}, 
	{0x27350B89, 9, 1}, {0x24DC0127, 7, 0}, {0x93459BE7, 9, 0}, {0x931B4B91, 9, 0}, 
	{0x25E22709, 9, 1}, {0x258DE759, 9, 1}, {0x929CEBF5, 9, 0}, {0x24E5F891, 9, 1}, 
	{0x24924925, 9, 1}, {0x921F64BF, 9, 0}, {0x91F5BCB9, 9, 0}, {0x239858D9, 9, 1}, 
	{0x91A2B3C5, 9, 0}, {0x917952AF, 9, 0}, {0x22A0122B, 9, 1}, {0x9126D6E5, 9, 0}, 
	{0x21FB7813, 9, 1}, {0x90D4B86F, 9, 0}, {0x21579805, 9, 1}, {0x09082F6B, 5, 0}, 
	{0x20B470C7, 9, 1}, {0x4818C885, 8, 0}, {0x20120121, 9, 1}, {0x8FE086E3, 9, 0}, 
	{0x1F7047DD, 9, 1}, {0x047C7EBD, 4, 0}, {0x23D9E879, 7, 0}, {0x1E7F0551, 9, 1}, 
	{0x478BBCED, 8, 0}, {0x4777C3B3, 8, 0}, {0x4763D59D, 8, 0}, {0x1D3FCA85, 9, 1}, 
	{0x1CF06ADB, 9, 1}, {0x1CA13751, 9, 1}, {0x8E2917E1, 9, 0}, {0x8E01AA05, 9, 0}, 
	{0x1BB4A405, 9, 1}, {0x1B661F8D, 9, 1}, {0x02362F8D, 3, 0}, {0x23593317, 7, 0}, 
	{0x8D3DCB09, 9, 0}, {0x1A2DBE6B, 9, 1}, {0x19E0119F, 9, 1}, {0x8CC947C5, 9, 0}, 
	{0x19453809, 9, 1}, {0x8C7C057D, 9, 0}, {0x8C55841D, 9, 0}, {0x185E2FA5, 9, 1}, 
	{0x18118119, 9, 1}, {0x17C4FC73, 9, 1}, {0x8BBC50C9, 9, 0}, {0x172C7053, 9, 1}, 
	{0x16E06895, 9, 1}, {0x45A5228D, 8, 0}, {0x11648D51, 6, 0}, {0x15FD4907, 9, 1}, 
	{0x22B63CBF, 7, 0}, {0x1566ABC1, 9, 1}, {0x04546E69, 4, 0}, {0x8A6858AB, 9, 0}, 
	{0x1485F0E1, 9, 1}, {0x143B58C1, 9, 1}, {0x44FC3A35, 8, 0}, {0x89D3507D, 9, 0}, 
	{0x44D72045, 8, 0}, {0x01131289, 2, 0}, {0x12C8B89F, 9, 1}, {0x127F0FD1, 9, 1}, 
	{0x891AC73B, 9, 0}, {0x11EC346F, 9, 1}, {0x11A3019B, 9, 1}, {0x44567D77, 8, 0}, 
	{0x88888889, 9, 0}, {0x8864298F, 9, 0}, {0x107FBBE1, 9, 1}, {0x440DD2CF, 8, 0}, 
	{0x10FEF011, 6, 0}, {0x043E9B75, 4, 0}, {0x43D7B7EB, 8, 0}, {0x878B841B, 9, 0}, 
	{0x0ECF56BF, 9, 1}, {0x8743E595, 9, 0}, {0x0E406559, 9, 1}, {0x0DF9252D, 9, 1}, 
	{0x86D90545, 9, 0}, {0x10D6B155, 6, 0}, {0x43491159, 8, 0}, {0x0CDD9AA7, 9, 1}, 
	{0x4325C53F, 8, 0}, {0x0C50B447, 9, 1}, {0x0C0A7869, 9, 1}, {0x0BC46147, 9, 1}, 
	{0x0B7E6EC3, 9, 1}, {0x0B38A0C1, 9, 1}, {0x0AF2F723, 9, 1}, {0x0AAD71CF, 9, 1}, 
	{0x0A6810A7, 9, 1}, {0x0A22D38F, 9, 1}, {0x42776E9B, 8, 0}, {0x084CC629, 5, 0}, 
	{0x0953F391, 9, 1}, {0x8487A2D1, 9, 0}, {0x21195767, 7, 0}, {0x08865437, 9, 1}, 
	{0x08421085, 9, 1}, {0x07FDF005, 9, 1}, {0x41EE7CA7, 8, 0}, {0x10776183, 6, 0}, 
	{0x073260A5, 9, 1}, {0x06EECBE1, 9, 1}, {0x20D56B39, 7, 0}, {0x06680A41, 9, 1}, 
	{0x10624DD3, 6, 0}, {0x05E1D27B, 9, 1}, {0x20B3DD41, 7, 0}, {0x415708EF, 8, 0}, 
	{0x828CBFBF, 9, 0}, {0x04D6FD33, 9, 1}, {0x824A4E61, 9, 0}, {0x104525E1, 6, 0}, 
	{0x04104105, 9, 1}, {0x03CE4585, 9, 1}, {0x038C6B79, 9, 1}, {0x81A55963, 9, 0}, 
	{0x81848DA9, 9, 0}, {0x8163D283, 9, 0}, {0x2050C9F9, 7, 0}, {0x81228DBF, 9, 0}, 
	{0x02040811, 9, 1}, {0x80E18AB3, 9, 0}, {0x80C121B3, 9, 0}, {0x014191F7, 9, 1}, 
	{0x80808081, 9, 0}, {0x00C0906D, 9, 1}, {0x00804021, 9, 1}, {0x00401005, 9, 1}, 
	{0x00400000, 0, 0}};

  FPFloat<uint32_t, uint64_t, uint32_t, T4, false, false> tmp;
  union {
    uint64_t v;
    uint32_t p[2];
  } m;

  if (divider > 1024)
    {
       tmp = fpf / FPFloat<uint32_t, uint64_t, uint32_t, T4, false, false>(divider);
    }
  else
    {
      if (magic_uint32[divider][2] == 0)
        {
          m.v = (uint64_t) fpf.getValue() * (uint64_t) magic_uint32[divider][0];
          tmp.assign(m.p[1] >> magic_uint32[divider][1]);
        }
      else
        {
          m.v = (uint64_t) fpf.getValue() * (uint64_t) magic_uint32[divider][0];
          m.p[1] = (((fpf.getValue() - m.p[1]) >> 1) + m.p[1]) >> magic_uint32[divider][1];
          tmp.assign(m.p[1]);
        }
    }

  return tmp;
}
*/

/** @}*/

} /* namespace types */
} /* namespace dd */
} /* namespace mke */

#endif /* _FPREAL_H_ */
