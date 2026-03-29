/*
 * TinyGPS++ - STM32 HAL Port
 *
 * Originally: TinyGPS++ by Mikal Hart (C) 2008-2024
 * STM32 HAL port: Replaces Arduino.h / millis() with STM32 HAL equivalents.
 * Target: STM32F411 (STM32F4xx HAL)
 *
 * LGPL v2.1 - see original license header.
 *
 * -----------------------------------------------------------------------
 * Key changes from the Arduino version:
 *   - #include "Arduino.h"  →  #include "stm32f4xx_hal.h"
 *   - millis()              →  HAL_GetTick()
 *   - byte                  →  uint8_t
 *   - radians/degrees/sq    →  inline constexpr helpers (no Arduino macros)
 *   - TWO_PI                →  GPS_TWO_PI constant
 *   - ULONG_MAX             →  comes from <climits>
 * -----------------------------------------------------------------------
 */

#pragma once

#include <inttypes.h>
#include <climits>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "stm32f4xx_hal.h"   /* HAL_GetTick() lives here */

/* ── version ─────────────────────────────────────────────────── */
#define _GPS_VERSION            "1.1.0-stm32"
#define _GPS_MAX_FIELD_SIZE     15

/* ── unit-conversion constants ────────────────────────────────── */
#define _GPS_MPH_PER_KNOT       1.15077945
#define _GPS_MPS_PER_KNOT       0.51444444
#define _GPS_KMPH_PER_KNOT      1.852
#define _GPS_MILES_PER_METER    0.00062137112
#define _GPS_KM_PER_METER       0.001
#define _GPS_FEET_PER_METER     3.2808399
#define _GPS_EARTH_MEAN_RADIUS  6371009UL   /* metres */

/* ── math helpers (replace Arduino macros) ────────────────────── */
static constexpr double GPS_PI     = 3.14159265358979323846;
static constexpr double GPS_TWO_PI = 6.28318530717958647692;

inline double gps_radians(double deg) { return deg * (GPS_PI / 180.0); }
inline double gps_degrees(double rad) { return rad * (180.0 / GPS_PI); }
inline double gps_sq(double x)        { return x * x; }

/* ── timing helper ────────────────────────────────────────────── */
inline uint32_t gps_millis() { return HAL_GetTick(); }


/* ════════════════════════════════════════════════════════════════
 *  Data types
 * ════════════════════════════════════════════════════════════════ */

struct RawDegrees
{
    uint16_t deg;
    uint32_t billionths;
    bool     negative;

    RawDegrees() : deg(0), billionths(0), negative(false) {}
};

/* ── Location ─────────────────────────────────────────────────── */
struct TinyGPSLocation
{
    friend class TinyGPSPlus;
public:
    enum Quality {
        Invalid    = '0',
        GPS        = '1',
        DGPS       = '2',
        PPS        = '3',
        RTK        = '4',
        FloatRTK   = '5',
        Estimated  = '6',
        Manual     = '7',
        Simulated  = '8'
    };
    enum Mode { N = 'N', A = 'A', D = 'D', E = 'E' };

    bool        isValid()  const { return valid; }
    bool        isUpdated()const { return updated; }
    uint32_t    age()      const { return valid ? gps_millis() - lastCommitTime
                                                : static_cast<uint32_t>(ULONG_MAX); }

    const RawDegrees &rawLat() { updated = false; return rawLatData; }
    const RawDegrees &rawLng() { updated = false; return rawLngData; }

    double  lat();
    double  lng();
    Quality fixQuality()  { updated = false; return _fixQuality; }
    Mode    fixMode()     { updated = false; return _fixMode; }

    TinyGPSLocation()
        : valid(false), updated(false),
          _fixQuality(Invalid), _fixMode(N), lastCommitTime(0) {}

private:
    bool valid, updated;
    RawDegrees rawLatData, rawLngData, rawNewLatData, rawNewLngData;
    Quality    _fixQuality, newFixQuality;
    Mode       _fixMode,    newFixMode;
    uint32_t   lastCommitTime;

    void commit();
    void setLatitude (const char *term);
    void setLongitude(const char *term);
};

/* ── Date ─────────────────────────────────────────────────────── */
struct TinyGPSDate
{
    friend class TinyGPSPlus;
public:
    bool     isValid()  const { return valid; }
    bool     isUpdated()const { return updated; }
    uint32_t age()      const { return valid ? gps_millis() - lastCommitTime
                                             : static_cast<uint32_t>(ULONG_MAX); }

    uint32_t value()  { updated = false; return date; }
    uint16_t year();
    uint8_t  month();
    uint8_t  day();

    TinyGPSDate() : valid(false), updated(false), date(0), newDate(0), lastCommitTime(0) {}

private:
    bool     valid, updated;
    uint32_t date, newDate;
    uint32_t lastCommitTime;
    void commit();
    void setDate(const char *term);
};

/* ── Time ─────────────────────────────────────────────────────── */
struct TinyGPSTime
{
    friend class TinyGPSPlus;
public:
    bool     isValid()  const { return valid; }
    bool     isUpdated()const { return updated; }
    uint32_t age()      const { return valid ? gps_millis() - lastCommitTime
                                             : static_cast<uint32_t>(ULONG_MAX); }

    uint32_t value()       { updated = false; return time; }
    uint8_t  hour();
    uint8_t  minute();
    uint8_t  second();
    uint8_t  centisecond();

    TinyGPSTime() : valid(false), updated(false), time(0), newTime(0), lastCommitTime(0) {}

private:
    bool     valid, updated;
    uint32_t time, newTime;
    uint32_t lastCommitTime;
    void commit();
    void setTime(const char *term);
};

/* ── Decimal (fixed-point, 2 decimal places) ─────────────────── */
struct TinyGPSDecimal
{
    friend class TinyGPSPlus;
public:
    bool     isValid()  const { return valid; }
    bool     isUpdated()const { return updated; }
    uint32_t age()      const { return valid ? gps_millis() - lastCommitTime
                                             : static_cast<uint32_t>(ULONG_MAX); }
    int32_t  value()    { updated = false; return val; }

    TinyGPSDecimal() : valid(false), updated(false), lastCommitTime(0), val(0), newval(0) {}

private:
    bool     valid, updated;
    uint32_t lastCommitTime;
    int32_t  val, newval;
    void commit();
    void set(const char *term);
};

/* ── Integer ──────────────────────────────────────────────────── */
struct TinyGPSInteger
{
    friend class TinyGPSPlus;
public:
    bool     isValid()  const { return valid; }
    bool     isUpdated()const { return updated; }
    uint32_t age()      const { return valid ? gps_millis() - lastCommitTime
                                             : static_cast<uint32_t>(ULONG_MAX); }
    uint32_t value()    { updated = false; return val; }

    TinyGPSInteger() : valid(false), updated(false), lastCommitTime(0), val(0), newval(0) {}

private:
    bool     valid, updated;
    uint32_t lastCommitTime;
    uint32_t val, newval;
    void commit();
    void set(const char *term);
};

/* ── Derived measurement types ────────────────────────────────── */
struct TinyGPSSpeed : public TinyGPSDecimal
{
    double knots() { return value() / 100.0; }
    double mph()   { return _GPS_MPH_PER_KNOT  * value() / 100.0; }
    double mps()   { return _GPS_MPS_PER_KNOT  * value() / 100.0; }
    double kmph()  { return _GPS_KMPH_PER_KNOT * value() / 100.0; }
};

struct TinyGPSCourse : public TinyGPSDecimal
{
    double deg() { return value() / 100.0; }
};

struct TinyGPSAltitude : public TinyGPSDecimal
{
    double meters()     { return value() / 100.0; }
    double miles()      { return _GPS_MILES_PER_METER * value() / 100.0; }
    double kilometers() { return _GPS_KM_PER_METER    * value() / 100.0; }
    double feet()       { return _GPS_FEET_PER_METER  * value() / 100.0; }
};

struct TinyGPSHDOP : public TinyGPSDecimal
{
    double hdop() { return value() / 100.0; }
};


/* ════════════════════════════════════════════════════════════════
 *  Forward declaration
 * ════════════════════════════════════════════════════════════════ */
class TinyGPSPlus;

/* ── Custom sentence field ────────────────────────────────────── */
class TinyGPSCustom
{
public:
    TinyGPSCustom() : lastCommitTime(0), valid(false), updated(false),
                      sentenceName(nullptr), termNumber(0), next(nullptr)
    {
        stagingBuffer[0] = buffer[0] = '\0';
    }

    TinyGPSCustom(TinyGPSPlus &gps, const char *sentenceName, int termNumber);
    void begin(TinyGPSPlus &gps, const char *sentenceName, int termNumber);

    bool        isUpdated() const { return updated; }
    bool        isValid()   const { return valid; }
    uint32_t    age()       const { return valid ? gps_millis() - lastCommitTime
                                                 : static_cast<uint32_t>(ULONG_MAX); }
    const char *value()     { updated = false; return buffer; }

private:
    void commit();
    void set(const char *term);

    char     stagingBuffer[_GPS_MAX_FIELD_SIZE + 1];
    char     buffer       [_GPS_MAX_FIELD_SIZE + 1];
    uint32_t lastCommitTime;
    bool     valid, updated;
    const char    *sentenceName;
    int            termNumber;
    friend class TinyGPSPlus;
    TinyGPSCustom *next;
};


/* ════════════════════════════════════════════════════════════════
 *  Main parser class
 * ════════════════════════════════════════════════════════════════ */
class TinyGPSPlus
{
public:
    TinyGPSPlus();

    /**
     * @brief Feed one character from the GPS UART into the parser.
     * @return true when a complete, checksum-validated sentence is received.
     *
     * Typical usage (polling):
     *   uint8_t ch;
     *   if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
     *       gps.encode(static_cast<char>(ch));
     */
    bool encode(char c);

    /** Stream-insertion operator (Arduino-style convenience). */
    TinyGPSPlus &operator<<(char c) { encode(c); return *this; }

    /* ── parsed fields ──────────────────────────────────────── */
    TinyGPSLocation location;
    TinyGPSDate     date;
    TinyGPSTime     time;
    TinyGPSSpeed    speed;
    TinyGPSCourse   course;
    TinyGPSAltitude altitude;
    TinyGPSInteger  satellites;
    TinyGPSHDOP     hdop;

    /* ── library meta ───────────────────────────────────────── */
    static const char *libraryVersion() { return _GPS_VERSION; }

    /* ── geodesic helpers ───────────────────────────────────── */
    static double      distanceBetween(double lat1, double lon1,
                                       double lat2, double lon2);
    static double      courseTo(double lat1, double lon1,
                                double lat2, double lon2);
    static const char *cardinal(double course);

    /* ── low-level helpers (needed by custom fields) ────────── */
    static int32_t parseDecimal(const char *term);
    static void    parseDegrees(const char *term, RawDegrees &deg);

    /* ── statistics ─────────────────────────────────────────── */
    uint32_t charsProcessed()   const { return encodedCharCount; }
    uint32_t sentencesWithFix() const { return sentencesWithFixCount; }
    uint32_t failedChecksum()   const { return failedChecksumCount; }
    uint32_t passedChecksum()   const { return passedChecksumCount; }

private:
    enum { GPS_SENTENCE_GGA, GPS_SENTENCE_RMC, GPS_SENTENCE_OTHER };

    /* parsing state */
    uint8_t parity;
    bool    isChecksumTerm;
    char    term[_GPS_MAX_FIELD_SIZE];
    uint8_t curSentenceType;
    uint8_t curTermNumber;
    uint8_t curTermOffset;
    bool    sentenceHasFix;

    /* custom element support */
    friend class TinyGPSCustom;
    TinyGPSCustom *customElts;
    TinyGPSCustom *customCandidates;
    void insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int index);

    /* statistics */
    uint32_t encodedCharCount;
    uint32_t sentencesWithFixCount;
    uint32_t failedChecksumCount;
    uint32_t passedChecksumCount;

    /* internals */
    int  fromHex(char a);
    bool endOfTermHandler();
};
