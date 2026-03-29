/*
 * TinyGPS++ - STM32 HAL Port
 *
 * Originally: TinyGPS++ by Mikal Hart (C) 2008-2024
 * STM32 HAL port for STM32F411.
 * LGPL v2.1.
 */

#include "TinyGPSPlus.h"

#define _RMCterm "RMC"
#define _GGAterm "GGA"

/* ════════════════════════════════════════════════════════════════
 *  TinyGPSPlus
 * ════════════════════════════════════════════════════════════════ */

TinyGPSPlus::TinyGPSPlus()
    : parity(0)
    , isChecksumTerm(false)
    , curSentenceType(GPS_SENTENCE_OTHER)
    , curTermNumber(0)
    , curTermOffset(0)
    , sentenceHasFix(false)
    , customElts(nullptr)
    , customCandidates(nullptr)
    , encodedCharCount(0)
    , sentencesWithFixCount(0)
    , failedChecksumCount(0)
    , passedChecksumCount(0)
{
    term[0] = '\0';
}

/* ── Public: encode one character ───────────────────────────── */

bool TinyGPSPlus::encode(char c)
{
    ++encodedCharCount;

    switch (c)
    {
    case ',':   /* term terminator – also XOR'd into parity */
        parity ^= static_cast<uint8_t>(c);
        /* FALL THROUGH */
    case '\r':
    case '\n':
    case '*':
    {
        bool isValidSentence = false;
        if (curTermOffset < sizeof(term))
        {
            term[curTermOffset] = '\0';
            isValidSentence = endOfTermHandler();
        }
        ++curTermNumber;
        curTermOffset  = 0;
        isChecksumTerm = (c == '*');
        return isValidSentence;
    }

    case '$':   /* sentence begin */
        curTermNumber  = 0;
        curTermOffset  = 0;
        parity         = 0;
        curSentenceType = GPS_SENTENCE_OTHER;
        isChecksumTerm  = false;
        sentenceHasFix  = false;
        return false;

    default:    /* ordinary character */
        if (curTermOffset < sizeof(term) - 1)
            term[curTermOffset++] = c;
        if (!isChecksumTerm)
            parity ^= static_cast<uint8_t>(c);
        return false;
    }
}

/* ── Internal utilities ─────────────────────────────────────── */

int TinyGPSPlus::fromHex(char a)
{
    if (a >= 'A' && a <= 'F') return a - 'A' + 10;
    if (a >= 'a' && a <= 'f') return a - 'a' + 10;
    return a - '0';
}

/* static */
int32_t TinyGPSPlus::parseDecimal(const char *term)
{
    bool negative = (*term == '-');
    if (negative) ++term;

    int32_t ret = 100 * static_cast<int32_t>(atol(term));
    while (isdigit(static_cast<unsigned char>(*term))) ++term;

    if (*term == '.' && isdigit(static_cast<unsigned char>(term[1])))
    {
        ret += 10 * (term[1] - '0');
        if (isdigit(static_cast<unsigned char>(term[2])))
            ret += (term[2] - '0');
    }
    return negative ? -ret : ret;
}

/* static
 * Parse NMEA degree format:  DDMM.MMMM  →  RawDegrees */
void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
{
    uint32_t leftOfDecimal = static_cast<uint32_t>(atol(term));
    uint16_t minutes       = static_cast<uint16_t>(leftOfDecimal % 100);
    uint32_t multiplier    = 10000000UL;
    uint32_t tenMillionthsOfMinutes = minutes * multiplier;

    deg.deg = static_cast<uint16_t>(leftOfDecimal / 100);

    while (isdigit(static_cast<unsigned char>(*term))) ++term;

    if (*term == '.')
        while (isdigit(static_cast<unsigned char>(*++term)))
        {
            multiplier /= 10;
            tenMillionthsOfMinutes += (*term - '0') * multiplier;
        }

    deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
    deg.negative   = false;
}

/* Packs sentence type + term number into one integer for the switch */
#define COMBINE(sentence_type, term_number) \
    (((unsigned)(sentence_type) << 5) | (term_number))

bool TinyGPSPlus::endOfTermHandler()
{
    /* ── checksum term ──────────────────────────────────────── */
    if (isChecksumTerm)
    {
        uint8_t checksum = static_cast<uint8_t>(
            16 * fromHex(term[0]) + fromHex(term[1]));

        if (checksum == parity)
        {
            ++passedChecksumCount;
            if (sentenceHasFix) ++sentencesWithFixCount;

            switch (curSentenceType)
            {
            case GPS_SENTENCE_RMC:
                date.commit();
                time.commit();
                if (sentenceHasFix)
                {
                    location.commit();
                    speed.commit();
                    course.commit();
                }
                break;

            case GPS_SENTENCE_GGA:
                time.commit();
                if (sentenceHasFix)
                {
                    location.commit();
                    altitude.commit();
                }
                satellites.commit();
                hdop.commit();
                break;

            default:
                break;
            }

            /* Commit matching custom listeners */
            for (TinyGPSCustom *p = customCandidates;
                 p != nullptr && strcmp(p->sentenceName,
                                        customCandidates->sentenceName) == 0;
                 p = p->next)
            {
                p->commit();
            }
            return true;
        }
        else
        {
            ++failedChecksumCount;
        }
        return false;
    }

    /* ── first term: identify sentence type ─────────────────── */
    if (curTermNumber == 0)
    {
        if (term[0] == 'G' &&
            strchr("PNABL", term[1]) != nullptr &&
            strcmp(term + 2, _RMCterm) == 0)
        {
            curSentenceType = GPS_SENTENCE_RMC;
        }
        else if (term[0] == 'G' &&
                 strchr("PNABL", term[1]) != nullptr &&
                 strcmp(term + 2, _GGAterm) == 0)
        {
            curSentenceType = GPS_SENTENCE_GGA;
        }
        else
        {
            curSentenceType = GPS_SENTENCE_OTHER;
        }

        /* Locate matching custom candidates (sorted linked list) */
        for (customCandidates = customElts;
             customCandidates != nullptr &&
             strcmp(customCandidates->sentenceName, term) < 0;
             customCandidates = customCandidates->next) {}

        if (customCandidates != nullptr &&
            strcmp(customCandidates->sentenceName, term) > 0)
        {
            customCandidates = nullptr;
        }
        return false;
    }

    /* ── data terms ─────────────────────────────────────────── */
    if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    {
        switch (COMBINE(curSentenceType, curTermNumber))
        {
        /* Time – shared by RMC and GGA */
        case COMBINE(GPS_SENTENCE_RMC, 1):
        case COMBINE(GPS_SENTENCE_GGA, 1):
            time.setTime(term);
            break;

        /* RMC: validity flag */
        case COMBINE(GPS_SENTENCE_RMC, 2):
            sentenceHasFix = (term[0] == 'A');
            break;

        /* Latitude */
        case COMBINE(GPS_SENTENCE_RMC, 3):
        case COMBINE(GPS_SENTENCE_GGA, 2):
            location.setLatitude(term);
            break;

        /* N/S */
        case COMBINE(GPS_SENTENCE_RMC, 4):
        case COMBINE(GPS_SENTENCE_GGA, 3):
            location.rawNewLatData.negative = (term[0] == 'S');
            break;

        /* Longitude */
        case COMBINE(GPS_SENTENCE_RMC, 5):
        case COMBINE(GPS_SENTENCE_GGA, 4):
            location.setLongitude(term);
            break;

        /* E/W */
        case COMBINE(GPS_SENTENCE_RMC, 6):
        case COMBINE(GPS_SENTENCE_GGA, 5):
            location.rawNewLngData.negative = (term[0] == 'W');
            break;

        /* Speed (RMC) */
        case COMBINE(GPS_SENTENCE_RMC, 7):
            speed.set(term);
            break;

        /* Course (RMC) */
        case COMBINE(GPS_SENTENCE_RMC, 8):
            course.set(term);
            break;

        /* Date (RMC) */
        case COMBINE(GPS_SENTENCE_RMC, 9):
            date.setDate(term);
            break;

        /* GGA: fix quality */
        case COMBINE(GPS_SENTENCE_GGA, 6):
            sentenceHasFix = (term[0] > '0');
            location.newFixQuality = static_cast<TinyGPSLocation::Quality>(term[0]);
            break;

        /* GGA: satellites in use */
        case COMBINE(GPS_SENTENCE_GGA, 7):
            satellites.set(term);
            break;

        /* GGA: HDOP */
        case COMBINE(GPS_SENTENCE_GGA, 8):
            hdop.set(term);
            break;

        /* GGA: altitude */
        case COMBINE(GPS_SENTENCE_GGA, 9):
            altitude.set(term);
            break;

        /* RMC: mode indicator (field 12) */
        case COMBINE(GPS_SENTENCE_RMC, 12):
            location.newFixMode = static_cast<TinyGPSLocation::Mode>(term[0]);
            break;

        default:
            break;
        }
    }

    /* Feed matching custom candidates */
    for (TinyGPSCustom *p = customCandidates;
         p != nullptr &&
         strcmp(p->sentenceName, customCandidates->sentenceName) == 0 &&
         p->termNumber <= static_cast<int>(curTermNumber);
         p = p->next)
    {
        if (p->termNumber == static_cast<int>(curTermNumber))
            p->set(term);
    }

    return false;
}

/* ── Geodesic computations ──────────────────────────────────── */

/* static */
double TinyGPSPlus::distanceBetween(double lat1, double lon1,
                                     double lat2, double lon2)
{
    double delta  = gps_radians(lon1 - lon2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);

    lat1 = gps_radians(lat1);
    lat2 = gps_radians(lat2);

    double slat1 = sin(lat1), clat1 = cos(lat1);
    double slat2 = sin(lat2), clat2 = cos(lat2);

    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = gps_sq(delta) + gps_sq(clat2 * sdlong);
    delta = sqrt(delta);

    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * static_cast<double>(_GPS_EARTH_MEAN_RADIUS);
}

/* static */
double TinyGPSPlus::courseTo(double lat1, double lon1,
                              double lat2, double lon2)
{
    double dlon = gps_radians(lon2 - lon1);
    lat1 = gps_radians(lat1);
    lat2 = gps_radians(lat2);

    double a1 = sin(dlon) * cos(lat2);
    double a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);

    if (a2 < 0.0) a2 += GPS_TWO_PI;
    return gps_degrees(a2);
}

/* static */
const char *TinyGPSPlus::cardinal(double course)
{
    static const char *const directions[] = {
        "N","NNE","NE","ENE","E","ESE","SE","SSE",
        "S","SSW","SW","WSW","W","WNW","NW","NNW"
    };
    int dir = static_cast<int>((course + 11.25f) / 22.5f);
    return directions[dir % 16];
}

/* ── insertCustom ───────────────────────────────────────────── */

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt,
                                const char    *sentenceName,
                                int            termNumber)
{
    TinyGPSCustom **ppelt;
    for (ppelt = &customElts; *ppelt != nullptr; ppelt = &(*ppelt)->next)
    {
        int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
        if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
            break;
    }
    pElt->next = *ppelt;
    *ppelt = pElt;
}


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSLocation
 * ════════════════════════════════════════════════════════════════ */

void TinyGPSLocation::commit()
{
    rawLatData       = rawNewLatData;
    rawLngData       = rawNewLngData;
    _fixQuality      = newFixQuality;
    _fixMode         = newFixMode;
    lastCommitTime   = gps_millis();
    valid = updated  = true;
}

void TinyGPSLocation::setLatitude (const char *term) { TinyGPSPlus::parseDegrees(term, rawNewLatData); }
void TinyGPSLocation::setLongitude(const char *term) { TinyGPSPlus::parseDegrees(term, rawNewLngData); }

double TinyGPSLocation::lat()
{
    updated = false;
    double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
    return rawLatData.negative ? -ret : ret;
}

double TinyGPSLocation::lng()
{
    updated = false;
    double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
    return rawLngData.negative ? -ret : ret;
}


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSDate
 * ════════════════════════════════════════════════════════════════ */

void TinyGPSDate::commit()
{
    date = newDate;
    lastCommitTime = gps_millis();
    valid = updated = true;
}

void     TinyGPSDate::setDate(const char *term) { newDate = atol(term); }
uint16_t TinyGPSDate::year()  { updated = false; return (date % 100) + 2000; }
uint8_t  TinyGPSDate::month() { updated = false; return (date / 100) % 100; }
uint8_t  TinyGPSDate::day()   { updated = false; return date / 10000; }


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSTime
 * ════════════════════════════════════════════════════════════════ */

void TinyGPSTime::commit()
{
    time = newTime;
    lastCommitTime = gps_millis();
    valid = updated = true;
}

void    TinyGPSTime::setTime(const char *term)
{
    newTime = static_cast<uint32_t>(TinyGPSPlus::parseDecimal(term));
}

uint8_t TinyGPSTime::hour()        { updated = false; return time / 1000000; }
uint8_t TinyGPSTime::minute()      { updated = false; return (time / 10000) % 100; }
uint8_t TinyGPSTime::second()      { updated = false; return (time / 100) % 100; }
uint8_t TinyGPSTime::centisecond() { updated = false; return time % 100; }


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSDecimal
 * ════════════════════════════════════════════════════════════════ */

void TinyGPSDecimal::commit()
{
    val = newval;
    lastCommitTime = gps_millis();
    valid = updated = true;
}

void TinyGPSDecimal::set(const char *term)
{
    newval = TinyGPSPlus::parseDecimal(term);
}


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSInteger
 * ════════════════════════════════════════════════════════════════ */

void TinyGPSInteger::commit()
{
    val = newval;
    lastCommitTime = gps_millis();
    valid = updated = true;
}

void TinyGPSInteger::set(const char *term)
{
    newval = static_cast<uint32_t>(atol(term));
}


/* ════════════════════════════════════════════════════════════════
 *  TinyGPSCustom
 * ════════════════════════════════════════════════════════════════ */

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps,
                              const char  *_sentenceName,
                              int          _termNumber)
{
    begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps,
                           const char  *_sentenceName,
                           int          _termNumber)
{
    lastCommitTime  = 0;
    valid = updated = false;
    sentenceName    = _sentenceName;
    termNumber      = _termNumber;
    memset(stagingBuffer, '\0', sizeof(stagingBuffer));
    memset(buffer,        '\0', sizeof(buffer));
    gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
    strcpy(buffer, stagingBuffer);
    lastCommitTime  = gps_millis();
    valid = updated = true;
}

void TinyGPSCustom::set(const char *term)
{
    strncpy(stagingBuffer, term, sizeof(stagingBuffer) - 1);
    stagingBuffer[sizeof(stagingBuffer) - 1] = '\0';
}
