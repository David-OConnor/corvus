//! See `elrs` module. From `https://github.com/betaflight/betaflight/blob/master/src/main/rx/expresslrs_common.c`

// todo: Consider how you set up your ELRS code. Its own module?

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */


static mut crc14tab: [u16; ELRS_CRC_LEN] = [0; ELRS_CRC_LEN];

static mut FHSSptr: u8 = 0;
static mut  FHSSsequence: [u8; ELRS_NR_SEQUENCE_ENTRIES] = [0; ELRS_NR_SEQUENCE_ENTRIES];
static FHSSfreqs: [u32; 69] = [0; 69];
static numFreqs: u8 = 0; // The number of FHSS frequencies in the table
static seqCount: u8 = 0;
static mut syncChannel: u8 = 0;

fn ms_to_us(ms: u32) {
    ms * 1000
}

// Regarding failsafe timeout values:
// @CapnBry - Higher rates shorter timeout. Usually it runs 1-1.5 seconds with complete sync 500Hz.
//            250Hz is 2-5s. 150Hz 2.5s. 50Hz stays in sync all 5 seconds of my test.
// The failsafe timeout values come from the ELRS project's ExpressLRS_AirRateConfig definitions.
elrsModSettings_t airRateConfig()[ELRS_RATE_MAX] = {

    [
        [0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12],
        [1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14],
        [2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12],
        [3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 2, 12]
    ],
];

elrsRfPerfParams_t rfPerfConfig()[ELRS_RATE_MAX] = {

    [
        [0, RATE_500HZ, -105, 1665, 2500, 2500, 3, 5000],
        [1, RATE_250HZ, -108, 3300, 3000, 2500, 6, 5000],
        [2, RATE_150HZ, -112, 5871, 3500, 2500, 10, 5000],
        [3, RATE_50HZ, -117, 18443, 4000, 2500, 0, 5000]
    ],
};

const FHSSfreqsISM2400: [u32; 69] = [
    FREQ_HZ_TO_REG_VAL_24(2400400000),
    FREQ_HZ_TO_REG_VAL_24(2401400000),
    FREQ_HZ_TO_REG_VAL_24(2402400000),
    FREQ_HZ_TO_REG_VAL_24(2403400000),
    FREQ_HZ_TO_REG_VAL_24(2404400000),

    FREQ_HZ_TO_REG_VAL_24(2405400000),
    FREQ_HZ_TO_REG_VAL_24(2406400000),
    FREQ_HZ_TO_REG_VAL_24(2407400000),
    FREQ_HZ_TO_REG_VAL_24(2408400000),
    FREQ_HZ_TO_REG_VAL_24(2409400000),

    FREQ_HZ_TO_REG_VAL_24(2410400000),
    FREQ_HZ_TO_REG_VAL_24(2411400000),
    FREQ_HZ_TO_REG_VAL_24(2412400000),
    FREQ_HZ_TO_REG_VAL_24(2413400000),
    FREQ_HZ_TO_REG_VAL_24(2414400000),

    FREQ_HZ_TO_REG_VAL_24(2415400000),
    FREQ_HZ_TO_REG_VAL_24(2416400000),
    FREQ_HZ_TO_REG_VAL_24(2417400000),
    FREQ_HZ_TO_REG_VAL_24(2418400000),
    FREQ_HZ_TO_REG_VAL_24(2419400000),

    FREQ_HZ_TO_REG_VAL_24(2420400000),
    FREQ_HZ_TO_REG_VAL_24(2421400000),
    FREQ_HZ_TO_REG_VAL_24(2422400000),
    FREQ_HZ_TO_REG_VAL_24(2423400000),
    FREQ_HZ_TO_REG_VAL_24(2424400000),

    FREQ_HZ_TO_REG_VAL_24(2425400000),
    FREQ_HZ_TO_REG_VAL_24(2426400000),
    FREQ_HZ_TO_REG_VAL_24(2427400000),
    FREQ_HZ_TO_REG_VAL_24(2428400000),
    FREQ_HZ_TO_REG_VAL_24(2429400000),

    FREQ_HZ_TO_REG_VAL_24(2430400000),
    FREQ_HZ_TO_REG_VAL_24(2431400000),
    FREQ_HZ_TO_REG_VAL_24(2432400000),
    FREQ_HZ_TO_REG_VAL_24(2433400000),
    FREQ_HZ_TO_REG_VAL_24(2434400000),

    FREQ_HZ_TO_REG_VAL_24(2435400000),
    FREQ_HZ_TO_REG_VAL_24(2436400000),
    FREQ_HZ_TO_REG_VAL_24(2437400000),
    FREQ_HZ_TO_REG_VAL_24(2438400000),
    FREQ_HZ_TO_REG_VAL_24(2439400000),

    FREQ_HZ_TO_REG_VAL_24(2440400000),
    FREQ_HZ_TO_REG_VAL_24(2441400000),
    FREQ_HZ_TO_REG_VAL_24(2442400000),
    FREQ_HZ_TO_REG_VAL_24(2443400000),
    FREQ_HZ_TO_REG_VAL_24(2444400000),

    FREQ_HZ_TO_REG_VAL_24(2445400000),
    FREQ_HZ_TO_REG_VAL_24(2446400000),
    FREQ_HZ_TO_REG_VAL_24(2447400000),
    FREQ_HZ_TO_REG_VAL_24(2448400000),
    FREQ_HZ_TO_REG_VAL_24(2449400000),

    FREQ_HZ_TO_REG_VAL_24(2450400000),
    FREQ_HZ_TO_REG_VAL_24(2451400000),
    FREQ_HZ_TO_REG_VAL_24(2452400000),
    FREQ_HZ_TO_REG_VAL_24(2453400000),
    FREQ_HZ_TO_REG_VAL_24(2454400000),

    FREQ_HZ_TO_REG_VAL_24(2455400000),
    FREQ_HZ_TO_REG_VAL_24(2456400000),
    FREQ_HZ_TO_REG_VAL_24(2457400000),
    FREQ_HZ_TO_REG_VAL_24(2458400000),
    FREQ_HZ_TO_REG_VAL_24(2459400000),

    FREQ_HZ_TO_REG_VAL_24(2460400000),
    FREQ_HZ_TO_REG_VAL_24(2461400000),
    FREQ_HZ_TO_REG_VAL_24(2462400000),
    FREQ_HZ_TO_REG_VAL_24(2463400000),
    FREQ_HZ_TO_REG_VAL_24(2464400000),

    FREQ_HZ_TO_REG_VAL_24(2465400000),
    FREQ_HZ_TO_REG_VAL_24(2466400000),
    FREQ_HZ_TO_REG_VAL_24(2467400000),
    FREQ_HZ_TO_REG_VAL_24(2468400000),
    FREQ_HZ_TO_REG_VAL_24(2469400000),

    FREQ_HZ_TO_REG_VAL_24(2470400000),
    FREQ_HZ_TO_REG_VAL_24(2471400000),
    FREQ_HZ_TO_REG_VAL_24(2472400000),
    FREQ_HZ_TO_REG_VAL_24(2473400000),
    FREQ_HZ_TO_REG_VAL_24(2474400000),

    FREQ_HZ_TO_REG_VAL_24(2475400000),
    FREQ_HZ_TO_REG_VAL_24(2476400000),
    FREQ_HZ_TO_REG_VAL_24(2477400000),
    FREQ_HZ_TO_REG_VAL_24(2478400000),
    FREQ_HZ_TO_REG_VAL_24(2479400000)
];

fn generateCrc14Table() {
{
    let mut crc: u16 = 0;
    for i in 0..ELRS_CRC_LEN {
        crc = i << (14 - 8);
        for j in 0..8 {
            crc = (crc << 1) ^ ((crc & 0x2000) ? ELRS_CRC14_POLY : 0);
        }
        crc14tab[i] = crc;
    }
}

fn calcCrc14(data: &[u8], len: u8, crc: u16) -> u16
{
    while (len--) {
        crc = (crc << 8) ^ crc14tab[((crc >> 6) ^ (uint16_t) *data++) & 0x00FF];
    }
    return crc & 0x3FFF;
}

fn initializeFHSSFrequencies(dom: FreqDomain) {
    match dom {

        ISM2400 => {
            FHSSfreqs = FHSSfreqsISM2400;
            numFreqs = sizeof(FHSSfreqsISM2400) / sizeof(uint32_t);
        }
        _ => {
            FHSSfreqs = NULL;
            numFreqs = 0;
        }
    }
}

uint32_t getInitialFreq(const int32_t freqCorrection)
{
    return FHSSfreqs[syncChannel] - freqCorrection;
}

fn getFHSSNumEntries() -> u8
{
    numFreqs
}

fn FHSSgetCurrIndex() -> u8
{
    FHSSptr
}

fn FHSSsetCurrIndex(value: u8)
{
    FHSSptr = value % seqCount;
}

fn FHSSgetNextFreqfreqCorrection: i32) -> u32
{
    FHSSptr = (FHSSptr + 1) % seqCount;
    return FHSSfreqs[FHSSsequence[FHSSptr]] - freqCorrection;
}

static mut seed: u32 = 0;

// returns 0 <= x < max where max < 256
fn rngN(max: u8) -> u8
{
    let m: u32 = 2147483648;
    let a: u32 = 214013;
    let c: u32 = 2531011;
    unsafe {
        seed = (a * seed + c) % m;
        return (seed >> 16) % max;
    }
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.

*/
void FHSSrandomiseFHSSsequence(const uint8_t UID[], const elrsFreqDomain_e dom)
{
    seed = ((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5];

    initializeFHSSFrequencies(dom);

    seqCount = (256 / MAX(numFreqs, 1)) * numFreqs;

    syncChannel = numFreqs / 2;

    // initialize the sequence array
    for (uint8_t i = 0; i < seqCount; i++) {
        if (i % numFreqs == 0) {
            FHSSsequence[i] = syncChannel;
        } else if (i % numFreqs == syncChannel) {
            FHSSsequence[i] = 0;
        } else {
            FHSSsequence[i] = i % numFreqs;
        }
    }

    for i in 0..seqCount {
        // if it's not the sync channel
        if (i % numFreqs != 0) {
            uint8_t offset = (i / numFreqs) * numFreqs; // offset to start of current block
            uint8_t rand = rngN(numFreqs - 1) + 1; // random number between 1 and numFreqs

            // switch this entry and another random entry in the same block
            uint8_t temp = FHSSsequence[i];
            FHSSsequence[i] = FHSSsequence[offset + rand];
            FHSSsequence[offset + rand] = temp;
        }
    }
}


uint16_t txPowerIndexToValue(const uint8_t index)
{
    switch (index) {
    case 0: return 0;
    case 1: return 10;
    case 2: return 25;
    case 3: return 100;
    case 4: return 500;
    case 5: return 1000;
    case 6: return 2000;
    case 7: return 250;
    case 8: return 50;
    default: return 0;
    }
}

const  ELRS_LQ_DEPTH: u8 = 4; //100 % 32

struct LinkQuality {
    array: [u32; ELRS_LQ_DEPTH];
    value: u8;
    index: u8;
    mask: u32;
}

static linkQuality_t lq;

void lqIncrease(void)
{
    if (lqPeriodIsSet()) {
        return;
    }
    lq.array[lq.index] |= lq.mask;
    lq.value += 1;
}

void lqNewPeriod(void)
{
    lq.mask <<= 1;
    if (lq.mask == 0) {
        lq.mask = (1 << 0);
        lq.index += 1;
    }

    // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
    if ((lq.index == 3) && (lq.mask & (1 << ELRS_LQ_DEPTH))) {
        lq.index = 0;
        lq.mask = (1 << 0);
    }

    if ((lq.array[lq.index] & lq.mask) != 0) {
        lq.array[lq.index] &= ~lq.mask;
        lq.value -= 1;
    }
}

uint8_t lqGet(void)
{
    return lq.value;
}

bool lqPeriodIsSet(void)
{
    return lq.array[lq.index] & lq.mask;
}

fn lqReset() ->{
    memset(&lq, 0, sizeof(lq));
    lq.mask = (1 << 0);
}

fn onvertSwitch1b(uint16_t val: u16) -> u16
{
    val ? 2000 : 1000
}

// 3b to decode 7 pos switches
fn convertSwitch3b(val: u16) -> u16
{
    switch (val) {
    case 0:
        return 1000;
    case 1:
        return 1275;
    case 2:
        return 1425;
    case 3:
        return 1575;
    case 4:
        return 1725;
    case 5:
        return 2000;
    default:
        return 1500;
    }
}

fn convertSwitchNb(val: u16, max: u16) -> u16
{
    if val > max ? { 1500 } else { val * 1000 / max + 1000 }
}

fn convertAnalog(val: u16) -> u16
{
    return CRSF_RC_CHANNEL_SCALE_LEGACY * val + 881;
}

fn hybridWideNonceToSwitchIndex(nonce: u8) -> u8
{
    // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
    // 0, 1, 2, 3, 4, 5, 6, 7,
    // 1, 2, 3, 4, 5, 6, 7, 0
    // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
    // this makes sure each of the 8 values is sent at least once every 16 packets
    // regardless of the TLM ratio
    // Index 7 also can never fall on a telemetry slot
    ((nonce & 0x07) + ((nonce >> 3) & 0x01)) % 8
}

