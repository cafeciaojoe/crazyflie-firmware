/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * lighthouse_core.c - central part of the lighthouse positioning system
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "log.h"
#include "param.h"
#include "statsCnt.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "crtp_localization_service.h"

#include "pulse_processor.h"
#include "pulse_processor_v1.h"
#include "pulse_processor_v2.h"

#include "lighthouse_deck_flasher.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"

#include "storage.h"

#include "test_support.h"
#include "static_mem.h"

static const uint32_t MAX_WAIT_TIME_FOR_HEALTH_MS = 4000;

static pulseProcessorResult_t angles;
static lighthouseUartFrame_t frame;
static lighthouseBsIdentificationData_t bsIdentificationData;

// Stats

typedef enum uwbEvent_e {
  statusNotReceiving = 0,
  statusMissingData = 1,
  statusToEstimator = 2,
} lhSystemStatus_t;

static bool uartSynchronized = false;

#define ONE_SECOND 1000
#define HALF_SECOND 500
#define FIFTH_SECOND 200
static STATS_CNT_RATE_DEFINE(serialFrameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(frameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(cycleRate, ONE_SECOND);

static STATS_CNT_RATE_DEFINE(bs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(bs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsRates[PULSE_PROCESSOR_N_BASE_STATIONS] = {&bs0Rate, &bs1Rate};

// Contains a bit map that indicates which base staions that are actively used, that is recevied
// and has valid geo and calib dats
static uint16_t baseStationActiveMapWs;
static uint16_t baseStationActiveMap;

// An overall system status indicating if data is sent to the estimator
static lhSystemStatus_t systemStatus;
static lhSystemStatus_t systemStatusWs;

static const uint32_t SYSTEM_STATUS_UPDATE_INTERVAL = FIFTH_SECOND;
static uint32_t nextUpdateTimeOfSystemStatus = 0;

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];
pulseProcessor_t lighthouseCoreState = {
////        finding mt values
//       .bsGeometry = {
//     {.valid = true, .origin = {-0.535540, -0.818941, 1.286868, }, .mat = {{0.565372, -0.620311, 0.543662, }, {0.595597, 0.762999, 0.251192, }, {-0.570631, 0.181787, 0.800833, }, }},
//   },

   //Mounted LH2s
   .bsGeometry = {
//     {.valid = true, .origin = {1.372729,0.828007,2.535797,}, .mat = {{-0.397244,0.717836,-0.571759,}, {-0.579784,-0.679249,-0.449969,}, {-0.711370,0.152749,0.686018,}, }},
//     {.valid = true, .origin = {0.044971,-0.344800,2.586955,}, .mat = {{0.385659,-0.685018,0.618076,}, {0.528814,0.713060,0.460326,}, {-0.756057,0.149319,0.637245,}, }},

//     8 measurements, CF on its side. global ref on the floor.
       {.valid = true, .origin = {1.478476,0.623313,2.515511,}, .mat = {{-0.484060,0.612882,-0.624549,}, {-0.524108,-0.774621,-0.353938,}, {-0.700711,0.156004,0.696180,}, }},
       {.valid = true, .origin = {-0.558502,-0.788018,2.526516,}, .mat = {{0.574795,-0.600447,0.555945,}, {0.560090,0.783996,0.267672,}, {-0.596581,0.157523,0.786942,}, }},
       {.valid = true, .origin = {0.015382,-0.366631,2.703934,}, .mat = {{0.748674,-0.561457,0.352496,}, {0.478762,0.825704,0.298330,}, {-0.458557,-0.054590,0.886987,}, }},

//     8 measurements facing up, Cf on the floor
//     {.valid = true, .origin = {1.496474,0.676385,1.321087,}, .mat = {{-0.463849,0.643641,-0.608745,}, {-0.526838,-0.752839,-0.394558,}, {-0.712240,0.137695,0.688298,}, }},
//     {.valid = true, .origin = {-0.480878,-0.785803,1.324651,}, .mat = {{0.542786,-0.631659,0.553526,}, {0.589587,0.755942,0.284499,}, {-0.598140,0.171929,0.782732,}, }},
//     {.valid = true, .origin = {-0.000422,-0.938488,1.203382,}, .mat = {{0.626998,-0.660871,0.412460,}, {0.611438,0.745560,0.265112,}, {-0.482719,0.085969,0.871546,}, }},
   },

   .bsCalibration = {
      //Mounted LH2s
    { // Base station 0 mode 1
    .sweep = {
      {.tilt = -0.051078, .phase = 0.000000, .curve = 0.247043, .gibphase = 1.665625, .gibmag = -0.011288, .ogeephase = 1.538609, .ogeemag = 0.471610, },
      {.tilt = 0.043630, .phase = -0.004223, .curve = 0.507986, .gibphase = 2.046868, .gibmag = -0.009459, .ogeephase = 2.389616, .ogeemag = 0.375476, },
             },
    .valid = true,
    },
     { // Base station 1 mode 2
       .valid = true,
    .sweep = {
      {.tilt = -0.049504, .phase = 0.000000, .curve = 0.069301, .gibphase = 1.765391, .gibmag = -0.024516, .ogeephase = 1.742652, .ogeemag = 1.290244, },
      {.tilt = 0.042626, .phase = -0.004367, .curve = 0.540287, .gibphase = 2.484687, .gibmag = -0.019553, .ogeephase = 2.698655, .ogeemag = 1.096106, },
       },
     },
     { // Base station 2 mode 3
       .valid = true,
    .sweep = {
      {.tilt = -0.049865, .phase = 0.000000, .curve = -0.273323, .gibphase = 1.327240, .gibmag = -0.005672, .ogeephase = 1.370818, .ogeemag = 0.126641, },
      {.tilt = 0.045034, .phase = 0.006017, .curve = 0.509140, .gibphase = 1.742129, .gibmag = -0.006332, .ogeephase = 2.285476, .ogeemag = 0.210732, },
       },
    },
   }
};


//  // finding mat values
//    { // Base station 0 mode 1
//    .sweep = {
//      {.tilt = -0.049865, .phase = 0.000000, .curve = -0.273323, .gibphase = 1.327240, .gibmag = -0.005672, .ogeephase = 1.370818, .ogeemag = 0.126641, },
//      {.tilt = 0.045034, .phase = 0.006017, .curve = 0.509140, .gibphase = 1.742129, .gibmag = -0.006332, .ogeephase = 2.285476, .ogeemag = 0.210732, },
//             },
//    .valid = true,
//    },
//  }
//};

#if LIGHTHOUSE_FORCE_TYPE == 1
pulseProcessorProcessPulse_t pulseProcessorProcessPulse = pulseProcessorV1ProcessPulse;
#elif LIGHTHOUSE_FORCE_TYPE == 2
pulseProcessorProcessPulse_t pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;
#else
pulseProcessorProcessPulse_t pulseProcessorProcessPulse = (void*)0;
#endif

#define UART_FRAME_LENGTH 12


// Persistent storage
#define STORAGE_VERSION_KEY "lh/ver"
#define CURRENT_STORAGE_VERSION "1"
#define STORAGE_KEY_GEO "lh/sys/0/geo/"
#define STORAGE_KEY_CALIB "lh/sys/0/cal/"
#define KEY_LEN 20

static void verifySetStorageVersion();
static baseStationGeometry_t geoBuffer;
TESTABLE_STATIC void initializeGeoDataFromStorage();
static lighthouseCalibration_t calibBuffer;
TESTABLE_STATIC void initializeCalibDataFromStorage();

// LED timer
static xTimerHandle timer;
static StaticTimer_t timerBuffer;
static uint8_t ledInternalStatus = 2;

static void ledTimer(xTimerHandle timer)
{
  switch (systemStatus)
  {
    case 0:
      if(ledInternalStatus != systemStatus)
      {
        lighthouseCoreSetLeds(lh_led_on, lh_led_off, lh_led_off);
        ledInternalStatus = systemStatus;
      }
      break;
    case 1: 
      if(ledInternalStatus != systemStatus)
      {
        lighthouseCoreSetLeds(lh_led_off, lh_led_on, lh_led_off);
        ledInternalStatus = systemStatus;
      }
      break;
    case 2:
      if(ledInternalStatus != systemStatus)
      {
        lighthouseCoreSetLeds(lh_led_off, lh_led_off, lh_led_on);
        ledInternalStatus = systemStatus;
      }
      break;
    default:
      ASSERT(false);
  } 
}

void lighthouseCoreInit() {
  lighthousePositionEstInit();
  timer = xTimerCreateStatic("ledTimer", M2T(FIFTH_SECOND), pdTRUE,
    NULL, ledTimer, &timerBuffer);
}

TESTABLE_STATIC bool getUartFrameRaw(lighthouseUartFrame_t *frame) {
  static char data[UART_FRAME_LENGTH];
  int syncCounter = 0;

  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    uart1Getchar(&data[i]);
    if ((unsigned char)data[i] == 0xff) {
      syncCounter += 1;
    }
  }

  memset(frame, 0, sizeof(*frame));

  frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

  frame->data.sensor = data[0] & 0x03;
  frame->data.channelFound = (data[0] & 0x80) == 0;
  frame->data.channel = (data[0] >> 3) & 0x0f;
  frame->data.slowbit = (data[0] >> 2) & 0x01;
  memcpy(&frame->data.width, &data[1], 2);
  memcpy(&frame->data.offset, &data[3], 3);
  memcpy(&frame->data.beamData, &data[6], 3);
  memcpy(&frame->data.timestamp, &data[9], 3);

  // Offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
  frame->data.offset *= 4;

  bool isPaddingZero = (((data[5] | data[8]) & 0xfe) == 0);
  bool isFrameValid = (isPaddingZero || frame->isSyncFrame);

  STATS_CNT_RATE_EVENT(&serialFrameRate);

  return isFrameValid;
}

TESTABLE_STATIC void waitForUartSynchFrame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    uart1Getchar(&c);
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
}

void lighthouseCoreSetLeds(lighthouseCoreLedState_t red, lighthouseCoreLedState_t orange, lighthouseCoreLedState_t green)
{
  uint8_t commandBuffer[2];

  commandBuffer[0] = 0x01;
  commandBuffer[1] = (green<<4) | (orange<<2) | red;

  uart1SendData(2, commandBuffer);
}


// Method used to estimate position
// 0 = Position calculated outside the estimator using intersection point of beams.
//     Yaw error calculated outside the estimator. Position and yaw error is pushed to the
//     estimator as pre-calculated.
// 1 = Sweep angles pushed into the estimator. Yaw error calculated outside the estimator
//     and pushed to the estimator as a pre-calculated value.
static uint8_t estimationMethod = 1;


static void usePulseResultCrossingBeams(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation) {
  pulseProcessorClearOutdated(appState, angles, basestation);

  if (basestation == 1) {
    STATS_CNT_RATE_EVENT(&cycleRate);

    lighthousePositionEstimatePoseCrossingBeams(appState, angles, 1);

    pulseProcessorProcessed(angles, 0);
    pulseProcessorProcessed(angles, 1);
  }
}


static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation) {
  STATS_CNT_RATE_EVENT(&cycleRate);

  pulseProcessorClearOutdated(appState, angles, basestation);

  lighthousePositionEstimatePoseSweeps(appState, angles, basestation);

  pulseProcessorProcessed(angles, basestation);
}

static void convertV2AnglesToV1Angles(pulseProcessorResult_t* angles) {
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    for (int bs = 0; bs < PULSE_PROCESSOR_N_BASE_STATIONS; bs++) {
      pulseProcessorBaseStationMeasuremnt_t* from = &angles->sensorMeasurementsLh2[sensor].baseStatonMeasurements[bs];
      pulseProcessorBaseStationMeasuremnt_t* to = &angles->sensorMeasurementsLh1[sensor].baseStatonMeasurements[bs];

      if (2 == from->validCount) {
        pulseProcessorV2ConvertToV1Angles(from->correctedAngles[0], from->correctedAngles[1], to->correctedAngles);
        to->validCount = from->validCount;
      } else {
        to->validCount = 0;
      }
    }
  }
}

static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int sweepId) {
  if (sweepId == sweepIdSecond) {
    const bool hasCalibrationData = pulseProcessorApplyCalibration(appState, angles, basestation);
    const bool hasGeoData = appState->bsGeometry[basestation].valid;
    if (hasCalibrationData && hasGeoData) {
      if (lighthouseBsTypeV2 == angles->measurementType) {
        // Emulate V1 base stations for now, convert to V1 angles
        convertV2AnglesToV1Angles(angles);
      }

      // Send measurement to the ground
      locSrvSendLighthouseAngle(basestation, angles);

      baseStationActiveMapWs = baseStationActiveMapWs | (1 << basestation);

      switch(estimationMethod) {
        case 0:
          usePulseResultCrossingBeams(appState, angles, basestation);
          break;
        case 1:
          usePulseResultSweeps(appState, angles, basestation);
          break;
        default:
          break;
      }
    }

    if (baseStationActiveMapWs != 0) {
      systemStatusWs = statusToEstimator;
    } else {
      systemStatusWs = statusMissingData;
    }
  }
}

/**
 * @brief Identify the type of base stations used in the system.
 * There is not member of the UART frame data we can use to directly identify the type, but we can use statistical methods.
 * The beamWord will vary for V2 base stations, while it will have the value 0x1ffff fairly offen for V1 base stations.
 *
 * @param frame
 * @param state
 * @return TESTABLE_STATIC identifyBaseStationType
 */
TESTABLE_STATIC lighthouseBaseStationType_t identifyBaseStationType(const lighthouseUartFrame_t* frame, lighthouseBsIdentificationData_t* state) {
    const uint32_t v1Indicator = 0x1ffff;
    const int requiredIndicatorsForV1 = 6;
    const int requiredSamplesForV2 = 20;
    state->sampleCount++;
    if (frame->data.beamData == v1Indicator) {
        state->hitCount++;
    }

    if (state->hitCount >= requiredIndicatorsForV1) {
        return lighthouseBsTypeV1;
    }

    if (state->sampleCount >= requiredSamplesForV2) {
        return lighthouseBsTypeV2;
    }

    return lighthouseBsTypeUnknown;
}

static pulseProcessorProcessPulse_t identifySystem(const lighthouseUartFrame_t* frame, lighthouseBsIdentificationData_t* bsIdentificationData) {
  pulseProcessorProcessPulse_t result = (void*)0;

  switch (identifyBaseStationType(frame, bsIdentificationData)) {
    case lighthouseBsTypeV1:
      DEBUG_PRINT("Locking to V1 system\n");
      result = pulseProcessorV1ProcessPulse;
      break;
    case lighthouseBsTypeV2:
      DEBUG_PRINT("Locking to V2 system\n");
      result = pulseProcessorV2ProcessPulse;
      break;
    default:
      // Nothing here
      break;
  }

  return result;
}

static void processFrame(pulseProcessor_t *appState, pulseProcessorResult_t* angles, const lighthouseUartFrame_t* frame) {
    int basestation;
    int sweepId;

    pulseWidth[frame->data.sensor] = frame->data.width;

    if (pulseProcessorProcessPulse(appState, &frame->data, angles, &basestation, &sweepId)) {
        STATS_CNT_RATE_EVENT(bsRates[basestation]);
        usePulseResult(appState, angles, basestation, sweepId);
    }
}

static void deckHealthCheck(pulseProcessor_t *appState, const lighthouseUartFrame_t* frame, const uint32_t now_ms) {
  if (!appState->healthDetermined) {
    if (0 == appState->healthFirstSensorTs) {
      appState->healthFirstSensorTs = now_ms;
    }

    if (0x0f == appState->healthSensorBitField) {
      appState->healthDetermined = true;
      // DEBUG_PRINT("All sensors good\n");
    } else {
      appState->healthSensorBitField |= (0x01 << frame->data.sensor);

      if ((now_ms - appState->healthFirstSensorTs) > MAX_WAIT_TIME_FOR_HEALTH_MS) {
        appState->healthDetermined = true;
        DEBUG_PRINT("Warning: not getting data from all sensors\n");
        for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
          if (appState->healthSensorBitField & (0x1 << i)) {
            DEBUG_PRINT("  %d - OK\n", i);
          } else {
            DEBUG_PRINT("  %d - error\n", i);
          }
        }
      }
    }
  }
}

static void updateSystemStatus(const uint32_t now_ms) {
  if (now_ms > nextUpdateTimeOfSystemStatus) {
    baseStationActiveMap = baseStationActiveMapWs;
    baseStationActiveMapWs = 0;

    systemStatus = systemStatusWs;
    systemStatusWs = statusNotReceiving;

    nextUpdateTimeOfSystemStatus = now_ms + SYSTEM_STATUS_UPDATE_INTERVAL;
  }
}

void lighthouseCoreTask(void *param) {
  bool isUartFrameValid = false;

  uart1Init(230400);
  systemWaitStart();

  verifySetStorageVersion();
  initializeGeoDataFromStorage();
  initializeCalibDataFromStorage();

  lighthouseDeckFlasherCheckVersionAndBoot();

  vTaskDelay(M2T(100));

  xTimerStart(timer, M2T(0));

  memset(&bsIdentificationData, 0, sizeof(bsIdentificationData));

  while(1) {
    memset(pulseWidth, 0, sizeof(pulseWidth[0]) * PULSE_PROCESSOR_N_SENSORS);
    waitForUartSynchFrame();
    uartSynchronized = true;

    bool previousWasSyncFrame = false;

    while((isUartFrameValid = getUartFrameRaw(&frame))) {
      const uint32_t now_ms = T2M(xTaskGetTickCount());

      // If a sync frame is getting through, we are only receiving sync frames. So nothing else. Reset state
      if(frame.isSyncFrame && previousWasSyncFrame) {
          pulseProcessorAllClear(&angles);
      }
      // Now we are receiving items
      else if(!frame.isSyncFrame) {
        STATS_CNT_RATE_EVENT(&frameRate);

        deckHealthCheck(&lighthouseCoreState, &frame, now_ms);
        if (pulseProcessorProcessPulse) {
          processFrame(&lighthouseCoreState, &angles, &frame);
        } else {
          pulseProcessorProcessPulse = identifySystem(&frame, &bsIdentificationData);
        }
      }

      previousWasSyncFrame = frame.isSyncFrame;

      updateSystemStatus(now_ms);
    }

    uartSynchronized = false;
  }
}

void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration) {
  if (baseStation < PULSE_PROCESSOR_N_BASE_STATIONS) {
    lighthouseCoreState.bsCalibration[baseStation] = *calibration;
  }
}

static void generateStorageKey(char* buf, const char* base, const uint8_t baseStation) {
  // TOOD make an implementation that supports baseStations with 2 digits
  ASSERT(baseStation <= 9);

  const int baseLen = strlen(base);
  memcpy(buf, base, baseLen);
  buf[baseLen] = '0' + baseStation;
  buf[baseLen + 1] = '\0';
}

bool lighthouseCorePersistData(const uint8_t baseStation, const bool geoData, const bool calibData) {
  bool result = true;
  char key[KEY_LEN];

  if (baseStation < PULSE_PROCESSOR_N_BASE_STATIONS) {
    if (geoData) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      result = result && storageStore(key, &lighthouseCoreState.bsGeometry[baseStation], sizeof(lighthouseCoreState.bsGeometry[baseStation]));
    }
    if (calibData) {
      generateStorageKey(key, STORAGE_KEY_CALIB, baseStation);
      result = result && storageStore(key, &lighthouseCoreState.bsCalibration[baseStation], sizeof(lighthouseCoreState.bsCalibration[baseStation]));
    }
  }

  return result;
}

static void verifySetStorageVersion() {
  const int bufLen = 5;
  char buffer[bufLen];

  const size_t fetched = storageFetch(STORAGE_VERSION_KEY, buffer, bufLen);
  if (fetched == 0) {
    storageStore(STORAGE_VERSION_KEY, CURRENT_STORAGE_VERSION, strlen(CURRENT_STORAGE_VERSION) + 1);
  } else {
    if (strcmp(buffer, CURRENT_STORAGE_VERSION) != 0) {
      // The storage format version is wrong! What to do?
      // No need to handle until we bump the storage version, assert for now.
      ASSERT_FAILED();
    }
  }
}

TESTABLE_STATIC void initializeGeoDataFromStorage() {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < PULSE_PROCESSOR_N_BASE_STATIONS; baseStation++) {
    if (!lighthouseCoreState.bsGeometry[baseStation].valid) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      const size_t geoSize = sizeof(geoBuffer);
      const size_t fetched = storageFetch(key, (void*)&geoBuffer, geoSize);
      if (fetched == geoSize) {
        lighthousePositionSetGeometryData(baseStation, &geoBuffer);
      }
    }
  }
}

TESTABLE_STATIC void initializeCalibDataFromStorage() {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < PULSE_PROCESSOR_N_BASE_STATIONS; baseStation++) {
    if (!lighthouseCoreState.bsCalibration[baseStation].valid) {
      generateStorageKey(key, STORAGE_KEY_CALIB, baseStation);
      const size_t calibSize = sizeof(calibBuffer);
      const size_t fetched = storageFetch(key, (void*)&calibBuffer, calibSize);
      if (fetched == calibSize) {
        lighthouseCoreSetCalibrationData(baseStation, &calibBuffer);
      }
    }
  }
}

LOG_GROUP_START(lighthouse)
LOG_ADD_BY_FUNCTION(LOG_UINT8, validAngles, &pulseProcessorAnglesQuality)

LOG_ADD(LOG_FLOAT, rawAngle0x, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[0].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle0y, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[0].angles[1])
LOG_ADD(LOG_FLOAT, rawAngle1x, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[1].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle1y, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[1].angles[1])
LOG_ADD(LOG_FLOAT, angle0x, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y, &angles.sensorMeasurementsLh1[0].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_1, &angles.sensorMeasurementsLh1[1].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_1, &angles.sensorMeasurementsLh1[1].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_1, &angles.sensorMeasurementsLh1[1].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_1, &angles.sensorMeasurementsLh1[1].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_2, &angles.sensorMeasurementsLh1[2].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_2, &angles.sensorMeasurementsLh1[2].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_2, &angles.sensorMeasurementsLh1[2].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_2, &angles.sensorMeasurementsLh1[2].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_3, &angles.sensorMeasurementsLh1[3].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_3, &angles.sensorMeasurementsLh1[3].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_3, &angles.sensorMeasurementsLh1[3].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_3, &angles.sensorMeasurementsLh1[3].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, rawAngle0xlh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[0].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle0ylh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[0].angles[1])
LOG_ADD(LOG_FLOAT, rawAngle1xlh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[1].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle1ylh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[1].angles[1])

LOG_ADD(LOG_FLOAT, angle0x_0lh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_0lh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_0lh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_0lh2, &angles.sensorMeasurementsLh2[0].baseStatonMeasurements[1].correctedAngles[1])

STATS_CNT_RATE_LOG_ADD(serRt, &serialFrameRate)
STATS_CNT_RATE_LOG_ADD(frmRt, &frameRate)
STATS_CNT_RATE_LOG_ADD(cycleRt, &cycleRate)

STATS_CNT_RATE_LOG_ADD(bs0Rt, &bs0Rate)
STATS_CNT_RATE_LOG_ADD(bs1Rt, &bs1Rate)

LOG_ADD(LOG_UINT16, width0, &pulseWidth[0])
LOG_ADD(LOG_UINT16, width1, &pulseWidth[1])
LOG_ADD(LOG_UINT16, width2, &pulseWidth[2])
LOG_ADD(LOG_UINT16, width3, &pulseWidth[3])

LOG_ADD(LOG_UINT8, comSync, &uartSynchronized)

LOG_ADD(LOG_UINT16, bsActive, &baseStationActiveMap)
LOG_ADD(LOG_UINT8, status, &systemStatus)
LOG_GROUP_STOP(lighthouse)

PARAM_GROUP_START(lighthouse)
PARAM_ADD(PARAM_UINT8, method, &estimationMethod)
PARAM_GROUP_STOP(lighthouse)
