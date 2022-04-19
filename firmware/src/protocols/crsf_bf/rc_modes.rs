#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adaptation from Betaflight's general RF code, for use with CRSF>
//! https://github.com/betaflight/betaflight/blob/master/src/main/fc/rc_modes.h
//! https://github.com/betaflight/betaflight/blob/master/src/main/fc/rc_modes.c

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

const BOXID_NONE: u8 = 255; // todo: type

use core::mem;

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum BoxId {
    // ARM flag
    BOXARM = 0,
    // FLIGHT_MODE
    BOXANGLE,
    BOXHORIZON,
    BOXMAG,
    BOXHEADFREE,
    BOXPASSTHRU,
    BOXFAILSAFE,
    BOXGPSRESCUE,
    BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,

// When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

    // RCMODE flags
    BOXANTIGRAVITY,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXBEEPERON,
    BOXLEDLOW,
    BOXCALIB,
    BOXOSD,
    BOXTELEMETRY,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXAIRMODE,
    BOX3D,
    BOXFPVANGLEMIX,
    BOXBLACKBOXERASE,
    BOXCAMERA1,
    BOXCAMERA2,
    BOXCAMERA3,
    BOXFLIPOVERAFTERCRASH,
    BOXPREARM,
    BOXBEEPGPSCOUNT,
    BOXVTXPITMODE,
    BOXPARALYZE,
    BOXUSER1,
    BOXUSER2,
    BOXUSER3,
    BOXUSER4,
    BOXPIDAUDIO,
    BOXACROTRAINER,
    BOXVTXCONTROLDISABLE,
    BOXLAUNCHCONTROL,
    BOXMSPOVERRIDE,
    BOXSTICKCOMMANDDISABLE,
    BOXBEEPERMUTE,
    CHECKBOX_ITEM_COUNT
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum ModeLogic {
    MODELOGIC_OR = 0,
    MODELOGIC_AND
}

// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
typedef struct boxBitmask_s { uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32]; } boxBitmask_t;

// todo: Types on these defines.
const MAX_MODE_ACTIVATION_CONDITION_COUNT: u16 = 20;

const CHANNEL_RANGE_MIN: u16 = 900;
const CHANNEL_RANGE_MAX: u16 = 2100;

// const MODE_STEP_TO_CHANNEL_VALUE(step) (CHANNEL_RANGE_MIN + 25 * step);
// const CHANNEL_VALUE_TO_STEP(channelValue) ((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25);

const MIN_MODE_RANGE_STEP: u16 = 0;
const MAX_MODE_RANGE_STEP: u16 = ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25);

// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub struct ChannelRange {
    pub startStep: u8,
    pub endStep: u8,
}

pub struct ModeActivationCondition {
    pub modeId: BoxId,
    pub auxChannelIndex: u8,
    pub range: ChannelRange,
    pub modeLogic: ModeLogic,
    pub linkedTo: BoxId,
}

// PG_DECLARE_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions);

// #if defined(USE_CUSTOM_BOX_NAMES)
//
// const MAX_BOX_USER_NAME_LENGTH 16
//
// typedef struct modeActivationConfig_s {
//     char box_user_1_name[MAX_BOX_USER_NAME_LENGTH];
//     char box_user_2_name[MAX_BOX_USER_NAME_LENGTH];
//     char box_user_3_name[MAX_BOX_USER_NAME_LENGTH];
//     char box_user_4_name[MAX_BOX_USER_NAME_LENGTH];
// } modeActivationConfig_t;
//
// PG_DECLARE(modeActivationConfig_t, modeActivationConfig);
// #endif

struct ModeActivationProfile {
    pub modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
}

const IS_RANGE_USABLE(range) ((range)->startStep < (range)->endStep)


const STICKY_MODE_BOOT_DELAY_US: u32 = 5_000_000; // todo: Define type

static mut rcModeActivationMask: BoxBitmask = unsafe { mem::zeroed() };  // one bit per mode defined in boxId_e
static mut stickyModesEverDisabled: BoxBitmask = unsafe { mem::zeroed() };

static mut airmodeEnabled: bool = false;

static mut activeMacCount: usize = 0;
static mut mutactiveMacArray: [u8; MAX_MODE_ACTIVATION_CONDITION_COUNT] = [0; MAX_MODE_ACTIVATION_CONDITION_COUNT];
static mut activeLinkedMacCount: usize = 0;
static mut activeLinkedMacArray: [u8; MAX_MODE_ACTIVATION_CONDITION_COUNT] = [0; MAX_MODE_ACTIVATION_CONDITION_COUNT];

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 2);

// #if defined(USE_CUSTOM_BOX_NAMES)
// PG_REGISTER_WITH_RESET_TEMPLATE(modeActivationConfig_t, modeActivationConfig, PG_MODE_ACTIVATION_CONFIG, 0);
//
// PG_RESET_TEMPLATE(modeActivationConfig_t, modeActivationConfig,
//     .box_user_1_name = { 0 },
//     .box_user_2_name = { 0 },
//     .box_user_3_name = { 0 },
//     .box_user_4_name = { 0 },
// );
// #endif

fn IS_RC_MODE_ACTIVE(boxId: BoxId) -> bool {
    bitArrayGet(&rcModeActivationMask, boxId)
}

fn rcModeUpdate(boxBitmask_t newState: &BoxBitmask) {
    unsafe { rcModeActivationMask = newState };
}

fn airmodeIsEnabled() -> bool {
    unsafe { airmodeEnabled }
}

fn isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range) -> bool {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    const uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

/*
 *  updateMasksForMac:
 *
 *  The following are the possible logic states at each MAC update:
 *      AND     NEW
 *      ---     ---
 *       F       F      - no previous AND macs evaluated, no previous active OR macs
 *       F       T      - at least 1 previous active OR mac (***this state is latched True***)
 *       T       F      - all previous AND macs active, no previous active OR macs
 *       T       T      - at least 1 previous inactive AND mac, no previous active OR macs
 */
void updateMasksForMac(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask, bool bActive)
{
    if (bitArrayGet(andMask, mac->modeId) || !bitArrayGet(newMask, mac->modeId)) {
        bool bAnd = mac->modeLogic == MODELOGIC_AND;

        if (!bAnd) {    // OR mac
            if (bActive) {
                bitArrayClr(andMask, mac->modeId);
                bitArraySet(newMask, mac->modeId);
            }
        } else {        // AND mac
            bitArraySet(andMask, mac->modeId);
            if (!bActive) {
                bitArraySet(newMask, mac->modeId);
            }
        }
    }
}

void updateMasksForStickyModes(const modeActivationCondition_t *mac, boxBitmask_t *andMask, boxBitmask_t *newMask)
{
    if (IS_RC_MODE_ACTIVE(mac->modeId)) {
        bitArrayClr(andMask, mac->modeId);
        bitArraySet(newMask, mac->modeId);
    } else {
        bool bActive = isRangeActive(mac->auxChannelIndex, &mac->range);

        if (bitArrayGet(&stickyModesEverDisabled, mac->modeId)) {
            updateMasksForMac(mac, andMask, newMask, bActive);
        } else {
            if (micros() >= STICKY_MODE_BOOT_DELAY_US && !bActive) {
                bitArraySet(&stickyModesEverDisabled, mac->modeId);
            }
        }
    }
}

void updateActivatedModes(void)
{
    boxBitmask_t newMask, andMask, stickyModes;
    memset(&andMask, 0, sizeof(andMask));
    memset(&newMask, 0, sizeof(newMask));
    memset(&stickyModes, 0, sizeof(stickyModes));
    bitArraySet(&stickyModes, BOXPARALYZE);

    // determine which conditions set/clear the mode
    for (int i = 0; i < activeMacCount; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(activeMacArray[i]);

        if (bitArrayGet(&stickyModes, mac->modeId)) {
            updateMasksForStickyModes(mac, &andMask, &newMask);
        } else if (mac->modeId < CHECKBOX_ITEM_COUNT) {
            bool bActive = isRangeActive(mac->auxChannelIndex, &mac->range);
            updateMasksForMac(mac, &andMask, &newMask, bActive);
        }
    }

    // Update linked modes
    for (int i = 0; i < activeLinkedMacCount; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(activeLinkedMacArray[i]);
        bool bActive = bitArrayGet(&andMask, mac->linkedTo) != bitArrayGet(&newMask, mac->linkedTo);

        updateMasksForMac(mac, &andMask, &newMask, bActive);
    }

    bitArrayXor(&newMask, sizeof(newMask), &newMask, &andMask);

    rcModeUpdate(&newMask);

    airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOXAIRMODE);
}

bool isModeActivationConditionPresent(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && (IS_RANGE_USABLE(&mac->range) || mac->linkedTo)) {
            return true;
        }
    }

    return false;
}

bool isModeActivationConditionLinked(boxId_e modeId)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->modeId == modeId && mac->linkedTo != 0) {
            return true;
        }
    }

    return false;
}

void removeModeActivationCondition(const boxId_e modeId)
{
    unsigned offset = 0;
    for (unsigned i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        modeActivationCondition_t *mac = modeActivationConditionsMutable(i);

        if (mac->modeId == modeId && !offset) {
            offset++;
        }

        if (offset) {
            while (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT && modeActivationConditions(i + offset)->modeId == modeId) {
                offset++;
            }

            if (i + offset < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
                memcpy(mac, modeActivationConditions(i + offset), sizeof(modeActivationCondition_t));
            } else {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        }
    }
}

bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac)
{
    if (memcmp(mac, emptyMac, sizeof(*emptyMac))) {
        return true;
    } else {
        return false;
    }
}

// Build the list of used modeActivationConditions indices
// We can then use this to speed up processing by only evaluating used conditions
void analyzeModeActivationConditions(void)
{
    modeActivationCondition_t emptyMac;
    memset(&emptyMac, 0, sizeof(emptyMac));

    activeMacCount = 0;
    activeLinkedMacCount = 0;

    for (uint8_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->linkedTo) {
            activeLinkedMacArray[activeLinkedMacCount++] = i;
        } else if (isModeActivationConditionConfigured(mac, &emptyMac)) {
            activeMacArray[activeMacCount++] = i;
        }
    }
#ifdef USE_PINIOBOX
    pinioBoxTaskControl();
#endif
}
