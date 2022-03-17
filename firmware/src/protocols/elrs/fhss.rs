#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/FHSS/FHSS.h


const FreqCorrectionMax: i32 =  100000 / FREQ_STEP;
const FreqCorrectionMin: i32 = -100000 / FREQ_STEP;

// #define FREQ_HZ_TO_REG_VAL(freq) ((uint32_t)((double)freq/(double)FREQ_STEP))

pub const FHSSptr: u8 = 0;
pub const FreqCorrection: i8 = 0;
pub const FHSSsequence: [u8; 69] = [0; 69];
pub const FHSSfreqs: [u32; 69] = [0; 69];
pub const sync_channel: u8 = 0;
pub const FHSS_SEQUENCE_CNT: u8 = 0;

// get the initial frequency, which is also the sync channel
#[inline(always)]
fn GetInitialFreq() -> u32
{
    return FHSSfreqs[sync_channel] - FreqCorrection;
}

// Get the current sequence pointer
#[inline(always)]
fn FHSSgetCurrIndex() -> u8
{
    return FHSSptr;
}

// Set the sequence pointer, used by RX on SYNC
#[inline(always)]
fn FHSSsetCurrIndex(value: u8)
{
    FHSSptr = value % FHSS_SEQUENCE_CNT;
}

// Advance the pointer to the next hop and return the frequency of that channel
#[inline(always)]
fn FHSSgetNextFreq() -> u32
{
    FHSSptr = (FHSSptr + 1) % FHSS_SEQUENCE_CNT;
    let freq: u32 = FHSSfreqs[FHSSsequence[FHSSptr]] - FreqCorrection;
    return freq;
}

// get the number of entries in the FHSS sequence
#[inline(always)]
fn FHSSgetSequenceCount() -> u8
{
    return FHSS_SEQUENCE_CNT;
}

// Our table of FHSS frequencies. Define a regulatory domain to select the correct set for your location and radio

const FHSSfreqs_AU_433: [u32; 3] = [
    FREQ_HZ_TO_REG_VAL(433420000),
    FREQ_HZ_TO_REG_VAL(433920000),
    FREQ_HZ_TO_REG_VAL(434420000)];

const FHSSfreqs_AU_915: [u32; 20] = [
    FREQ_HZ_TO_REG_VAL(915500000),
    FREQ_HZ_TO_REG_VAL(916100000),
    FREQ_HZ_TO_REG_VAL(916700000),
    FREQ_HZ_TO_REG_VAL(917300000),

    FREQ_HZ_TO_REG_VAL(917900000),
    FREQ_HZ_TO_REG_VAL(918500000),
    FREQ_HZ_TO_REG_VAL(919100000),
    FREQ_HZ_TO_REG_VAL(919700000),

    FREQ_HZ_TO_REG_VAL(920300000),
    FREQ_HZ_TO_REG_VAL(920900000),
    FREQ_HZ_TO_REG_VAL(921500000),
    FREQ_HZ_TO_REG_VAL(922100000),

    FREQ_HZ_TO_REG_VAL(922700000),
    FREQ_HZ_TO_REG_VAL(923300000),
    FREQ_HZ_TO_REG_VAL(923900000),
    FREQ_HZ_TO_REG_VAL(924500000),

    FREQ_HZ_TO_REG_VAL(925100000),
    FREQ_HZ_TO_REG_VAL(925700000),
    FREQ_HZ_TO_REG_VAL(926300000),
    FREQ_HZ_TO_REG_VAL(926900000)
];

/* Frequency bands taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: these frequencies fall in the license free H-band, but in combination with 500kHz
 * LoRa modem bandwidth used by ExpressLRS (EU allows up to 125kHz modulation BW only) they
 * will never pass RED certification and they are ILLEGAL to use.
 *
 * Therefore we simply maximize the usage of available spectrum so laboratory testing of the software won't disturb existing
 * 868MHz ISM band traffic too much.
 */
const FHSSfreqs_EU_868: [u32; 13] = [
    FREQ_HZ_TO_REG_VAL(863275000), // band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(863800000),
    FREQ_HZ_TO_REG_VAL(864325000),
    FREQ_HZ_TO_REG_VAL(864850000),
    FREQ_HZ_TO_REG_VAL(865375000), // Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(865900000),
    FREQ_HZ_TO_REG_VAL(866425000),
    FREQ_HZ_TO_REG_VAL(866950000),
    FREQ_HZ_TO_REG_VAL(867475000),
    FREQ_HZ_TO_REG_VAL(868000000),
    FREQ_HZ_TO_REG_VAL(868525000), // Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(869050000),
    FREQ_HZ_TO_REG_VAL(869575000),
];


/**
 * India currently delicensed the 865-867 MHz band with a maximum of 1W Transmitter power,
 * 4Watts Effective Radiated Power and 200Khz carrier bandwidth as per
 * https://dot.gov.in/sites/default/files/Delicensing%20in%20865-867%20MHz%20band%20%5BGSR%20564%20%28E%29%5D_0.pdf .
 * There is currently no mention of Direct-sequence spread spectrum,
 * So these frequencies are a subset of Regulatory_Domain_EU_868 frequencies.
 */
const  FHSSfreqs_IN_86: [u32; 4] = [
    FREQ_HZ_TO_REG_VAL(865375000),
    FREQ_HZ_TO_REG_VAL(865900000),
    FREQ_HZ_TO_REG_VAL(866425000),
    FREQ_HZ_TO_REG_VAL(866950000)
];


/* Frequency band G, taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: As is the case with the 868Mhz band, these frequencies only comply to the license free portion
 * of the spectrum, nothing else. As such, these are likely illegal to use.
 */
const FHSSfreqs_EU_43: [u32; 3] = [
    FREQ_HZ_TO_REG_VAL(433100000),
    FREQ_HZ_TO_REG_VAL(433925000),
    FREQ_HZ_TO_REG_VAL(434450000)
];

/* Very definitely not fully checked. An initial pass at increasing the hops
*/
const FHSSfreqs_FCC_915: [u32; 40] = [
    FREQ_HZ_TO_REG_VAL(903500000),
    FREQ_HZ_TO_REG_VAL(904100000),
    FREQ_HZ_TO_REG_VAL(904700000),
    FREQ_HZ_TO_REG_VAL(905300000),

    FREQ_HZ_TO_REG_VAL(905900000),
    FREQ_HZ_TO_REG_VAL(906500000),
    FREQ_HZ_TO_REG_VAL(907100000),
    FREQ_HZ_TO_REG_VAL(907700000),

    FREQ_HZ_TO_REG_VAL(908300000),
    FREQ_HZ_TO_REG_VAL(908900000),
    FREQ_HZ_TO_REG_VAL(909500000),
    FREQ_HZ_TO_REG_VAL(910100000),

    FREQ_HZ_TO_REG_VAL(910700000),
    FREQ_HZ_TO_REG_VAL(911300000),
    FREQ_HZ_TO_REG_VAL(911900000),
    FREQ_HZ_TO_REG_VAL(912500000),

    FREQ_HZ_TO_REG_VAL(913100000),
    FREQ_HZ_TO_REG_VAL(913700000),
    FREQ_HZ_TO_REG_VAL(914300000),
    FREQ_HZ_TO_REG_VAL(914900000),

    FREQ_HZ_TO_REG_VAL(915500000), // as per AU..
    FREQ_HZ_TO_REG_VAL(916100000),
    FREQ_HZ_TO_REG_VAL(916700000),
    FREQ_HZ_TO_REG_VAL(917300000),

    FREQ_HZ_TO_REG_VAL(917900000),
    FREQ_HZ_TO_REG_VAL(918500000),
    FREQ_HZ_TO_REG_VAL(919100000),
    FREQ_HZ_TO_REG_VAL(919700000),

    FREQ_HZ_TO_REG_VAL(920300000),
    FREQ_HZ_TO_REG_VAL(920900000),
    FREQ_HZ_TO_REG_VAL(921500000),
    FREQ_HZ_TO_REG_VAL(922100000),

    FREQ_HZ_TO_REG_VAL(922700000),
    FREQ_HZ_TO_REG_VAL(923300000),
    FREQ_HZ_TO_REG_VAL(923900000),
    FREQ_HZ_TO_REG_VAL(924500000),

    FREQ_HZ_TO_REG_VAL(925100000),
    FREQ_HZ_TO_REG_VAL(925700000),
    FREQ_HZ_TO_REG_VAL(926300000),
    FREQ_HZ_TO_REG_VAL(926900000)
];


const FHSSfreqs_ISM_2400: [u32; 80] = [
    FREQ_HZ_TO_REG_VAL(2400400000),
    FREQ_HZ_TO_REG_VAL(2401400000),
    FREQ_HZ_TO_REG_VAL(2402400000),
    FREQ_HZ_TO_REG_VAL(2403400000),
    FREQ_HZ_TO_REG_VAL(2404400000),

    FREQ_HZ_TO_REG_VAL(2405400000),
    FREQ_HZ_TO_REG_VAL(2406400000),
    FREQ_HZ_TO_REG_VAL(2407400000),
    FREQ_HZ_TO_REG_VAL(2408400000),
    FREQ_HZ_TO_REG_VAL(2409400000),

    FREQ_HZ_TO_REG_VAL(2410400000),
    FREQ_HZ_TO_REG_VAL(2411400000),
    FREQ_HZ_TO_REG_VAL(2412400000),
    FREQ_HZ_TO_REG_VAL(2413400000),
    FREQ_HZ_TO_REG_VAL(2414400000),

    FREQ_HZ_TO_REG_VAL(2415400000),
    FREQ_HZ_TO_REG_VAL(2416400000),
    FREQ_HZ_TO_REG_VAL(2417400000),
    FREQ_HZ_TO_REG_VAL(2418400000),
    FREQ_HZ_TO_REG_VAL(2419400000),

    FREQ_HZ_TO_REG_VAL(2420400000),
    FREQ_HZ_TO_REG_VAL(2421400000),
    FREQ_HZ_TO_REG_VAL(2422400000),
    FREQ_HZ_TO_REG_VAL(2423400000),
    FREQ_HZ_TO_REG_VAL(2424400000),

    FREQ_HZ_TO_REG_VAL(2425400000),
    FREQ_HZ_TO_REG_VAL(2426400000),
    FREQ_HZ_TO_REG_VAL(2427400000),
    FREQ_HZ_TO_REG_VAL(2428400000),
    FREQ_HZ_TO_REG_VAL(2429400000),

    FREQ_HZ_TO_REG_VAL(2430400000),
    FREQ_HZ_TO_REG_VAL(2431400000),
    FREQ_HZ_TO_REG_VAL(2432400000),
    FREQ_HZ_TO_REG_VAL(2433400000),
    FREQ_HZ_TO_REG_VAL(2434400000),

    FREQ_HZ_TO_REG_VAL(2435400000),
    FREQ_HZ_TO_REG_VAL(2436400000),
    FREQ_HZ_TO_REG_VAL(2437400000),
    FREQ_HZ_TO_REG_VAL(2438400000),
    FREQ_HZ_TO_REG_VAL(2439400000),

    FREQ_HZ_TO_REG_VAL(2440400000),
    FREQ_HZ_TO_REG_VAL(2441400000),
    FREQ_HZ_TO_REG_VAL(2442400000),
    FREQ_HZ_TO_REG_VAL(2443400000),
    FREQ_HZ_TO_REG_VAL(2444400000),

    FREQ_HZ_TO_REG_VAL(2445400000),
    FREQ_HZ_TO_REG_VAL(2446400000),
    FREQ_HZ_TO_REG_VAL(2447400000),
    FREQ_HZ_TO_REG_VAL(2448400000),
    FREQ_HZ_TO_REG_VAL(2449400000),

    FREQ_HZ_TO_REG_VAL(2450400000),
    FREQ_HZ_TO_REG_VAL(2451400000),
    FREQ_HZ_TO_REG_VAL(2452400000),
    FREQ_HZ_TO_REG_VAL(2453400000),
    FREQ_HZ_TO_REG_VAL(2454400000),

    FREQ_HZ_TO_REG_VAL(2455400000),
    FREQ_HZ_TO_REG_VAL(2456400000),
    FREQ_HZ_TO_REG_VAL(2457400000),
    FREQ_HZ_TO_REG_VAL(2458400000),
    FREQ_HZ_TO_REG_VAL(2459400000),

    FREQ_HZ_TO_REG_VAL(2460400000),
    FREQ_HZ_TO_REG_VAL(2461400000),
    FREQ_HZ_TO_REG_VAL(2462400000),
    FREQ_HZ_TO_REG_VAL(2463400000),
    FREQ_HZ_TO_REG_VAL(2464400000),

    FREQ_HZ_TO_REG_VAL(2465400000),
    FREQ_HZ_TO_REG_VAL(2466400000),
    FREQ_HZ_TO_REG_VAL(2467400000),
    FREQ_HZ_TO_REG_VAL(2468400000),
    FREQ_HZ_TO_REG_VAL(2469400000),

    FREQ_HZ_TO_REG_VAL(2470400000),
    FREQ_HZ_TO_REG_VAL(2471400000),
    FREQ_HZ_TO_REG_VAL(2472400000),
    FREQ_HZ_TO_REG_VAL(2473400000),
    FREQ_HZ_TO_REG_VAL(2474400000),

    FREQ_HZ_TO_REG_VAL(2475400000),
    FREQ_HZ_TO_REG_VAL(2476400000),
    FREQ_HZ_TO_REG_VAL(2477400000),
    FREQ_HZ_TO_REG_VAL(2478400000),
    FREQ_HZ_TO_REG_VAL(2479400000)
];

// Number of FHSS frequencies in the table
const FHSS_FREQ_CNT: u32 = (sizeof(FHSSfreqs) / sizeof(u32));
// Number of hops in the FHSSsequence list before circling back around, even multiple of the number of frequencies
const FHSS_SEQUENCE_CNT: u8 = (256 / FHSS_FREQ_CNT) as u8 * FHSS_FREQ_CNT as u8;
// Actual sequence of hops as indexes into the frequency list
static mut FHSSsequence: [u8; FHSS_SEQUENCE_CNT] = [0; FHSS_SEQUENCE_CNT];
// Which entry in the sequence we currently are on
static mut FHSSptr: u8 = 0;
// Channel for sync packets and initial connection establishment
static mut sync_channel: u8 = 0;
// Offset from the predefined frequency determined by AFC on Team900 (register units)
static mut FreqCorrection: i32 = 0;

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
fn FHSSrandomiseFHSSsequence(seed: u32)
{
// #ifdef Regulatory_Domain_AU_915
//     INFOLN("Setting 915MHz AU Mode");
// #elif defined Regulatory_Domain_FCC_915
//     INFOLN("Setting 915MHz FCC Mode");
// #elif defined Regulatory_Domain_EU_868
//     INFOLN("Setting 868MHz EU Mode");
// #elif defined Regulatory_Domain_IN_866
//     INFOLN("Setting 866MHz IN Mode");
// #elif defined Regulatory_Domain_AU_433
//     INFOLN("Setting 433MHz AU Mode");
// #elif defined Regulatory_Domain_EU_433
//     INFOLN("Setting 433MHz EU Mode");
// #elif defined Regulatory_Domain_ISM_2400
//     INFOLN("Setting 2400MHz Mode");
// #else
// #error No regulatory domain defined, please define one in common.h
// #endif

    println!("Number of FHSS frequencies = %u", FHSS_FREQ_CNT);

    sync_channel = FHSS_FREQ_CNT / 2;
    println!("Sync channel = %u", sync_channel);

    // reset the pointer (otherwise the tests fail)
    FHSSptr = 0;
    rngSeed(unsafe { seed });

    // initialize the sequence array
for i in 0..FHSS_SEQUENCE_CNT
    {
        if i % FHSS_FREQ_CNT == 0 {
            FHSSsequence[i] = sync_channel;
        } else if i % FHSS_FREQ_CNT == sync_channel {
            FHSSsequence[i] = 0;
        } else {
            FHSSsequence[i] = i % FHSS_FREQ_CNT;
        }
    }

for i in 0..FHSS_SEQUENCE_CNT
    {
        // if it's not the sync channel
        if i % FHSS_FREQ_CNT != 0
        {
            let offset: u8 = (i / FHSS_FREQ_CNT) * FHSS_FREQ_CNT; // offset to start of current block
            let rand: u8 = rngN(FHSS_FREQ_CNT-1)+1; // random number between 1 and FHSS_FREQ_CNT

            // switch this entry and another random entry in the same block
            let temp: u8 = FHSSsequence[i];
            FHSSsequence[i] = FHSSsequence[offset+rand];
            FHSSsequence[offset+rand] = temp;
        }
    }

    // output FHSS sequence
        for i in 0..FHSS_SEQUENCE_CNT{
        println!("{} ",FHSSsequence[i]);
        if i % 10 == 9 {
            // DBGCR;
        }
    }
    DBGCR;
}

fn FHSSgetChannelCount() -> u32
{
    return FHSS_FREQ_CNT;
}

// the max value returned by rng
const RNG_MAX: u16 = 0x7FFF;

static mut seed: u32 = 0;

// returns values between 0 and 0x7FFF
// NB rngN depends on this output range, so if we change the
// behaviour rngN will need updating
fn rng() -> u16
{
    let m: u32 = 2147483648;
    let a: u32 = 214013;
    let c: u32 = 2531011;

    unsafe {
        seed = (a * seed + c) % m;
        return (seed >> 16) as u16;
    }
}

fn rngSeed(newSeed: u32)
{
    unsafe { seed = newSeed };
}

// returns 0 <= x < max where max < 256
fn rngN(max: u8) -> u8
{
    return (rng() % max) as u8;
}

// 0..255 returned
fn rng8Bit() -> u8
{
    return (rng() & 0xff) as u8;
}

// 0..31 returned
fn rng5Bit() -> u8
{
    return (rng() & 0x1F) as u8;
}