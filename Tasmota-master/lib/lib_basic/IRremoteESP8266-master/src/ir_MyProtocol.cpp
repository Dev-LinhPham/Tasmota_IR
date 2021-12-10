// Copyright 2020-2021 David Conran (crankyoldgit)
/// @file
/// @brief Support for Mirage protocol
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/1289
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/1573


#include "ir_MyProtocol.h"
#include <algorithm>
#include <cstring>
#ifndef ARDUINO
#include <string>
#endif
#include "IRrecv.h"
#include "IRsend.h"
#include "IRtext.h"
#include "IRutils.h"

using irutils::addBoolToString;
using irutils::addFanToString;
using irutils::addIntToString;
using irutils::addLabeledString;
using irutils::addModeToString;
using irutils::addModelToString;
//using irutils::addSwingHToString;
//using irutils::addSwingVToString;
using irutils::addTempToString;
//using irutils::addToggleToString;
using irutils::minsToString;
using irutils::bcdToUint8;
using irutils::uint8ToBcd;
using irutils::sumNibbles;

// Constants
const uint16_t kMyProtocolHdrMark = 8358;//8360;            ///< uSeconds 8358
const uint16_t kMyProtocolBitMark = 554;             ///< uSeconds
const uint16_t kMyProtocolHdrSpace = 4206;//4248;           ///< uSeconds 4206
const uint16_t kMyProtocolOneSpace = 1628;//1592;           ///< uSeconds 1628
const uint16_t kMyProtocolZeroSpace = 532;//545;           ///< uSeconds 532
const uint32_t kMyProtocolGap = kDefaultMessageGap;  ///< uSeconds (just a guess)
const uint16_t kMyProtocolFreq = 38000;              ///< Hz. (Just a guess)

const uint8_t kMyProtocolAcPowerOn  = 0b00;  // 0
const uint8_t kMyProtocolAcPowerOff = 0b11;  // 3


#if SEND_MYPROTOCOL
/// Send a Mirage formatted message.
/// Status: STABLE / Reported as working.
/// @param[in] data An array of bytes containing the IR command.
/// @param[in] nbytes Nr. of bytes of data in the array. (>=kMirageStateLength)
/// @param[in] repeat Nr. of times the message is to be repeated.
void IRsend::sendMyProtocol(const uint8_t data[], const uint16_t nbytes,
                        const uint16_t repeat) {
  sendGeneric(kMyProtocolHdrMark, kMyProtocolHdrSpace,  // Mục này giống các mã khác 
              kMyProtocolBitMark, kMyProtocolOneSpace,
              kMyProtocolBitMark, kMyProtocolZeroSpace,
              kMyProtocolBitMark, kMyProtocolGap,
              data, nbytes, kMyProtocolFreq, false,  // LSB
              repeat, kDutyDefault);
}
#endif  // SEND_MYPROTOCOL

#if DECODE_MYPROTOCOL
/// Decode the supplied Mirage message.
/// Status: STABLE / Reported as working.
/// @param[in,out] results Ptr to the data to decode & where to store the decode
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return A boolean. True if it can decode it, false if it can't.
/// Giải mã tín hiệu 
bool IRrecv::decodeMyProtocol(decode_results *results, uint16_t offset,
                          const uint16_t nbits, const bool strict) {
                            
  if (strict && nbits != kMyProtocolBits) return false;  // Compliance // Nếu SL bit không giống sẽ trả về false

  if (!matchGeneric(results->rawbuf + offset, results->state,
                    results->rawlen - offset, nbits,
                    kMyProtocolHdrMark, kMyProtocolHdrSpace,
                    kMyProtocolBitMark, kMyProtocolOneSpace,
                    kMyProtocolBitMark, kMyProtocolZeroSpace,
                    kMyProtocolBitMark, kMyProtocolGap, true,
                    kUseDefTol, kMarkExcess, false)) return false;
  // Compliance
  if (strict && !IRMyProtocolAc::validChecksum(results->state)) return false;

  // Success
  results->decode_type = decode_type_t::MYPROTOCOL;
  results->bits = nbits;
  // No need to record the state as we stored it as we decoded it.
  // As we use result->state, we don't record value, address, or command as it
  // is a union data type.
  return true;
}

// Code to emulate Mirage A/C IR remote control unit.

/// Class constructor
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
/// Cấu trúc lớp 
IRMyProtocolAc::IRMyProtocolAc(const uint16_t pin, const bool inverted,
                               const bool use_modulation)
    : _irsend(pin, inverted, use_modulation) { stateReset(); }

/// Reset the state of the remote to a known good state/sequence.
/// Đặt lại điều hòa 
void IRMyProtocolAc::stateReset(void) {
  // The state of the IR remote in IR code form.
  static const uint8_t kReset[kMyProtocolStateLength] = {
      0x56, 0x6C, 0x00, 0x00, 0x20, 0x1A, 0x00, 0x00,
      0x0C, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x42};
  setRaw(kReset);
  _model = myprotocol_ac_remote_model_t::KKG9AC1; // khi reset trạng thái => model chuyển sang dạng KKG9AC1 
}

/// Set up hardware to be able to send a message.
void IRMyProtocolAc::begin(void) { _irsend.begin(); }// Khởi động ỉr 

#if SEND_MITSUBISHI_AC
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
///Gửi mã Raw trong ví dụ IRsendDemo 
void IRMyProtocolAc::send(const uint16_t repeat) {
  _irsend.sendMyProtocol(getRaw(), kMyProtocolStateLength, repeat);
  // Reset any toggles after a send.
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      //setCleanToggle(false);
      //setLight(false);  // For this model (only), Light is a toggle.
      break;
    default:
      break;
  }
}
#endif  // SEND_MITSUBISHI_AC

/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
/// Lấy mã Raw thu được từ điều khiển 
uint8_t *IRMyProtocolAc::getRaw(void) {
  checksum(); // kiểm tra tổng 
  return _.raw;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] data A valid code for this protocol.
/// Set mã Raw 
void IRMyProtocolAc::setRaw(const uint8_t *data) {
  std::memcpy(_.raw, data, kMyProtocolStateLength);
  _model = getModel(true);
}

/// Guess the Mirage remote model from the supplied state code.
/// @param[in] state A valid state code for this protocol.
/// @return The model code.
/// @note This result isn't perfect. Both protocols can look the same but have
///       wildly different settings.
/// Kiểm tra các chức năng để trả model điều hòa đang sử dụng 
myprotocol_ac_remote_model_t IRMyProtocolAc::getModel(const uint8_t *state) {
  MyProtocol120Protocol p;
  std::memcpy(p.raw, state, kMyProtocolStateLength);
  // Check for KKG29AC1 specific settings.
  if (p.Sleep_Kkg21bc1 || p.OffTimerEnable || p.OnTimerEnable)
    return myprotocol_ac_remote_model_t::KKG21BC1;
  // Check for things specific to KKG9AC1
  if ((p.Minutes || p.Seconds) ||  // Is part of the clock set?
      // Are the timer times set, but not enabled? (enable check filtered above)
      (p.OffTimerHours || p.OffTimerMins) ||
      (p.OnTimerHours || p.OnTimerMins))
    return myprotocol_ac_remote_model_t::KKG9AC1;
  // As the above test has a 1 in 3600+ (for 1 second an hour) chance of a false
  // negative in theory, we are going assume that anything left should be a
  // KKG29AC1 model.
  return myprotocol_ac_remote_model_t::KKG21BC1;  // Default.
}

/// Get the model code of the interal message state.
/// @param[in] useRaw If set, we try to get the model info from just the state.
/// @return The model code.
myprotocol_ac_remote_model_t IRMyProtocolAc::getModel(const bool useRaw) const {
  return useRaw ? getModel(_.raw) : _model;
}

/// Set the model code of the interal message state.
/// @param[in] model The desired model to use for the settings
/// Set model điều hòa muốn điều khiển 
void IRMyProtocolAc::setModel(const myprotocol_ac_remote_model_t model) {
  if (model != _model) {  // Only change things if we need to.
    // Save the old settings.
    stdAc::state_t state = toCommon();
    const uint16_t ontimer = getOnTimer();
    const uint16_t offtimer = getOffTimer();
    //const bool ifeel = getIFeel();
    //const uint8_t sensor = getSensorTemp();
    // Change the model.
    state.model = model;
    // Restore/Convert the settings.
    fromCommon(state);
    setOnTimer(ontimer);
    setOffTimer(offtimer);
    //setIFeel(ifeel);
    //setSensorTemp(sensor);
  }
}

/// Calculate and set the checksum values for the internal state.
void IRMyProtocolAc::checksum(void) { _.Sum = calculateChecksum(_.raw); }

/// Verify the checksum is valid for a given state.
/// @param[in] data The array to verify the checksum of.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IRMyProtocolAc::validChecksum(const uint8_t *data) {
  return calculateChecksum(data) == data[kMyProtocolStateLength - 1];
}

/// Calculate the checksum for a given state.
/// @param[in] data The value to calc the checksum of.
/// @return The calculated checksum value.
uint8_t IRMyProtocolAc::calculateChecksum(const uint8_t *data) {
  return sumNibbles(data, kMyProtocolStateLength - 1);
}

/// Set the requested power state of the A/C to on.
void IRMyProtocolAc::on(void) { setPower(true); }

/// Set the requested power state of the A/C to off.
void IRMyProtocolAc::off(void) { setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
/// Set bật tắt điều hòa 
void IRMyProtocolAc::setPower(bool on) {
//kiểm tra model 
  //switch (_model) {
    //case mirage_ac_remote_model_t::KKG29AC1:
      _.Power = on ? kMyProtocolAcPowerOn : kMyProtocolAcPowerOff;
    //  break;
   // default:
      // In order to change the power setting, it seems must be less than
      // kMirageAcPowerOff. kMirageAcPowerOff is larger than half of the
      // possible value stored in the allocated bit space.
      // Thus if the value is larger than kMirageAcPowerOff the power is off.
      // Less than, then power is on.
      // We can't just aribitarily add or subtract the value (which analysis
      // indicates is how the power status changes. Very weird, I know!) as that
      // is not an idempotent action, we must check if the addition or
      // substraction is needed first. e.g. via getPower()
      // i.e. If we added or subtracted twice, we would cause a wrap of the
      // integer and not get the desired result.
      // Đoạn này ko hiểu quy luật do khác model 
     // if (on)
      //  _.SwingAndPower -= getPower() ? 0 : kMirageAcPowerOff;
      //else
      //  _.SwingAndPower += getPower() ? kMirageAcPowerOff : 0;
  //}
}

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IRMyProtocolAc::getPower(void) const {
  //switch (_model) {
    //case mirage_ac_remote_model_t::KKG29AC1:
      return _.Power == kMyProtocolAcPowerOn;
    //default:
     // return _.SwingAndPower < kMirageAcPowerOff;
  //}
}

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IRMyProtocolAc::getMode(void) const { return _.Mode; }

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
void IRMyProtocolAc::setMode(const uint8_t mode) {
  switch (mode) {
    case kMyProtocolAcCool:
    case kMyProtocolAcDry:
    case kMyProtocolAcHeat:
    case kMyProtocolAcFan:
    //case kMirageAcRecycle:
      _.Mode = mode;
      // Reset turbo if we have to.
    // setTurbo(getTurbo());
     break;
    default:  // Default to cool mode for anything else.
      setMode(kMyProtocolAcCool);
  }
}

/// Set the temperature.
/// @param[in] degrees The temperature in degrees celsius.
void IRMyProtocolAc::setTemp(const uint8_t degrees) {
  // Make sure we have desired temp in the correct range.
  uint8_t celsius = std::max(degrees, kMyProtocolAcMinTemp); // so sánh degree nhập vào so với Min Temp đã khai báo 
  _.Temp = std::min(celsius, kMyProtocolAcMaxTemp) + kMyProtocolAcTempOffset;// 16 + 92(0x5C) = 108 (0x6C) 
}

/// Get the current temperature setting.
/// @return The current setting for temp. in degrees celsius.
uint8_t IRMyProtocolAc::getTemp(void) const { return _.Temp - kMyProtocolAcTempOffset; }

/// Set the speed of the fan.
/// @param[in] speed The desired setting.
/// Nếu Speed <= FanLow thì Fan = Speed , còn nêú lớn hơn cho reset về FanAuto
void IRMyProtocolAc::setFan(const uint8_t speed) {
  _.Fan = (speed <= kMyProtocolAcFanLow) ? speed : kMyProtocolAcFanAuto;
}

/// Get the current fan speed setting.
/// @return The current fan speed/mode.
uint8_t IRMyProtocolAc::getFan(void) const { return _.Fan; }

/// Change the Turbo setting.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMyProtocolAc::setTurbo(bool on) {
//   const bool value = (on && (getMode() == kMirageAcCool)); //nếu điều hòa on và mode: Cool thì value: true 
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.Turbo_Kkg29ac1 = value;
//       break;
//     default:
//       _.Turbo_Kkg9ac1 = value;
//   }
// }

/// Get the value of the current Turbo setting.
/// @return true, the setting is on. false, the setting is off.
// bool IRMirageAc::getTurbo(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.Turbo_Kkg29ac1;
//     default:                                 return _.Turbo_Kkg9ac1;
//   }
// }

/// Change the Sleep setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRMyProtocolAc::setSleep(bool on) {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      _.Sleep_Kkg21bc1 = on;
      break;
    default:
      _.Sleep_Kkg9ac1 = on;
  }
}

/// Get the value of the current Sleep setting.
/// @return true, the setting is on. false, the setting is off.
bool IRMyProtocolAc::getSleep(void) const {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1: return _.Sleep_Kkg21bc1;
    default:                                 return _.Sleep_Kkg9ac1;
  }
}

/// Change the Light/Display setting.
/// @param[in] on true, the setting is on. false, the setting is off.
/// @note Light is a toggle on the KKG29AC1 model.
// void IRMirageAc::setLight(bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.LightToggle_Kkg29ac1 = on;
//       break;
//     default:
//       _.Light_Kkg9ac1 = on;
//   }
// }

/// Get the value of the current Light/Display setting.
/// @return true, the setting is on. false, the setting is off.
/// @note Light is a toggle on the KKG29AC1 model.
// bool IRMirageAc::getLight(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.LightToggle_Kkg29ac1;
//     default:                                 return _.Light_Kkg9ac1;
//   }
// }

  /// Get the clock time of the A/C unit.
  /// @return Nr. of seconds past midnight.
uint32_t IRMyProtocolAc::getClock(void) const {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      return 0;
    default:
      return ((bcdToUint8(_.Hours) * 60) + bcdToUint8(_.Minutes)) * 60 +
          bcdToUint8(_.Seconds);
  }
}

/// Set the clock time on the A/C unit.
/// @param[in] nr_of_seconds Nr. of seconds past midnight.
void IRMyProtocolAc::setClock(const uint32_t nr_of_seconds) {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      _.Minutes = _.Seconds = 0;  // No clock setting. Clear it just in case.
      break;
    default:
      uint32_t remaining = std::min(
          nr_of_seconds, (uint32_t)(24 * 60 * 60 - 1));  // Limit to 23:59:59.
      _.Seconds = uint8ToBcd(remaining % 60);
      remaining /= 60;
      _.Minutes = uint8ToBcd(remaining % 60);
      remaining /= 60;
      _.Hours = uint8ToBcd(remaining);
  }
}

/// Set the Vertical Swing setting/position of the A/C.
/// @param[in] position The desired swing setting.
// void IRMirageAc::setSwingV(const uint8_t position) {
//   switch (position) {
//     case kMirageAcSwingVOff:
//     case kMirageAcSwingVLowest:
//     case kMirageAcSwingVLow:
//     case kMirageAcSwingVMiddle:
//     case kMirageAcSwingVHigh:
//     case kMirageAcSwingVHighest:
//     case kMirageAcSwingVAuto:
//       switch (_model) {
//         case mirage_ac_remote_model_t::KKG29AC1:
//           _.SwingV = (position != kMirageAcSwingVOff);
//           break;
//         default:
//           const bool power = getPower();
//           _.SwingAndPower = position;
//           // Power needs to be reapplied after overwriting SwingAndPower
//           setPower(power);
//       }
//       break;
//     default:  // Default to Auto for anything else.
//       setSwingV(kMirageAcSwingVAuto);
//   }
// }

/// Get the Vertical Swing setting/position of the A/C.
/// @return The desired Vertical Swing setting/position.
// uint8_t IRMirageAc::getSwingV(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       return _.SwingV ? kMirageAcSwingVAuto : kMirageAcSwingVOff;
//     default:
//       return _.SwingAndPower - (getPower() ? 0 : kMirageAcPowerOff);
//   }
// }

/// Set the Horizontal Swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMirageAc::setSwingH(const bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.SwingH = on;
//       break;
//     default:
//       break;
//   }
// }

/// Get the Horizontal Swing setting of the A/C.
/// @return on true, the setting is on. false, the setting is off.
// bool IRMirageAc::getSwingH(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.SwingH;
//     default:                                 return false;
//   }
// }

/// Set the Quiet setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMirageAc::setQuiet(const bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.Quiet = on;
//       break;
//     default:
//       break;
//   }
// }

/// Get the Quiet setting of the A/C.
/// @return on true, the setting is on. false, the setting is off.
// bool IRMirageAc::getQuiet(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.Quiet;
//     default:                                 return false;
//   }
// }

/// Set the CleanToggle setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMirageAc::setCleanToggle(const bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.CleanToggle = on;
//       break;
//     default:
//       break;
//   }
// }

/// Get the Clean Toggle setting of the A/C.
/// @return on true, the setting is on. false, the setting is off.
// bool IRMirageAc::getCleanToggle(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.CleanToggle;
//     default:                                 return false;
//   }
// }

/// Set the Filter setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMirageAc::setFilter(const bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.Filter = on;
//       break;
//     default:
//       break;
//   }
// }

/// Get the Filter setting of the A/C.
/// @return on true, the setting is on. false, the setting is off.
// bool IRMirageAc::getFilter(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.Filter;
//     default:                                 return false;
//   }
// }

/// Set the IFeel setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
// void IRMirageAc::setIFeel(const bool on) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.IFeel = on;
//       if (on) {
//         // If no previous sensor temp, default to currently desired temp.
//         if (!_.SensorTemp) _.SensorTemp = getTemp();
//       } else {
//         _.SensorTemp = 0;  // When turning it off, clear the Sensor Temp.
//       }
//       break;
//     default:
//       break;
//   }
// }

/// Get the IFeel setting of the A/C.
/// @return on true, the setting is on. false, the setting is off.
// bool IRMirageAc::getIFeel(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1: return _.IFeel;
//     default:                                 return false;
//   }
// }

/// Set the Sensor Temp setting of the A/C's remote.
/// @param[in] degrees The desired sensor temp. in degrees celsius.
// void IRMirageAc::setSensorTemp(const uint8_t degrees) {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       _.SensorTemp = std::min(kMirageAcSensorTempMax, degrees) +
//           kMirageAcSensorTempOffset;
//       break;
//     default:
//       break;
//   }
// }

/// Get the Sensor Temp setting of the A/C's remote.
/// @return The current setting for the sensor temp. in degrees celsius.
// uint16_t IRMirageAc::getSensorTemp(void) const {
//   switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       return _.SensorTemp - kMirageAcSensorTempOffset;
//     default:
//       return false;
//   }
// }

/// Get the number of minutes the On Timer is currently set for.
/// @return Nr. of Minutes the timer is set for. 0, is the timer is not in use.
uint16_t IRMyProtocolAc::getOnTimer(void) const {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      return _.OnTimerEnable ? _.OnTimerHours * 60 + _.OnTimerMins : 0;// nếu OnTimerEnable bật thì = hours *60 + min
    default:
      return 0;
  }
}

/// Set the number of minutes for the On Timer.
/// @param[in] nr_of_mins How long to set the timer for. 0 disables the timer.
void IRMyProtocolAc::setOnTimer(const uint16_t nr_of_mins) {
  uint16_t mins = std::min(nr_of_mins, (uint16_t)(24 * 60)); // số phút max là 65535
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      _.OnTimerEnable = (mins > 0);
      _.OnTimerHours = mins / 60;
      _.OnTimerMins = mins % 60;
      break;
    default:
      break;
  }
}

/// Get the number of minutes the Off Timer is currently set for.
/// @return Nr. of Minutes the timer is set for. 0, is the timer is not in use.
uint16_t IRMyProtocolAc::getOffTimer(void) const {
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      return _.OffTimerEnable ? _.OffTimerHours * 60 + _.OffTimerMins : 0;
    default:
      return 0;
  }
}

/// Set the number of minutes for the Off Timer.
/// @param[in] nr_of_mins How long to set the timer for. 0 disables the timer.
void IRMyProtocolAc::setOffTimer(const uint16_t nr_of_mins) {
  uint16_t mins = std::min(nr_of_mins, (uint16_t)(24 * 60));
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      _.OffTimerEnable = (mins > 0);
      _.OffTimerHours = mins / 60;
      _.OffTimerMins = mins % 60;
      break;
    default:
      break;
  }
}

/// Convert a native mode into its stdAc equivalent.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::opmode_t IRMyProtocolAc::toCommonMode(const uint8_t mode) {
  switch (mode) {
    case kMyProtocolAcCool: return stdAc::opmode_t::kCool;
    case kMyProtocolAcDry:  return stdAc::opmode_t::kDry;
    case kMyProtocolAcFan:  return stdAc::opmode_t::kFan;
    default:                return stdAc::opmode_t::kHeat;
  }
}

/// Convert a native fan speed into its stdAc equivalent.
/// @param[in] speed The native setting to be converted.
/// @param[in] model The model type to use to influence the conversion.
/// @return The stdAc equivalent of the native setting.
stdAc::fanspeed_t IRMyProtocolAc::toCommonFanSpeed(const uint8_t speed){
   // const mirage_ac_remote_model_t model) {
  //switch (model) {
     // case mirage_ac_remote_model_t::KKG29AC1:
      switch (speed) {
        case kMyProtocolAcFanHigh:  return stdAc::fanspeed_t::kHigh;
        case kMyProtocolAcFanMed:   return stdAc::fanspeed_t::kMedium;
        case kMyProtocolAcFanLow:   return stdAc::fanspeed_t::kLow;
        default:                    return stdAc::fanspeed_t::kAuto;
      }
     // break;
    // default:
    //   switch (speed) {
    //     case kMirageAcFanHigh:  return stdAc::fanspeed_t::kHigh;
    //     case kMirageAcFanMed:   return stdAc::fanspeed_t::kMedium;
    //     case kMirageAcFanLow:   return stdAc::fanspeed_t::kLow;
    //     default:                return stdAc::fanspeed_t::kAuto;
    //   }
  //}
}

/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivalent of the enum.
/// Cần hàm này 
uint8_t IRMyProtocolAc::convertMode(const stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool: return kMyProtocolAcCool;
    case stdAc::opmode_t::kDry:  return kMyProtocolAcDry;
    case stdAc::opmode_t::kFan:  return kMyProtocolAcFan;
    default:                     return kMyProtocolAcHeat;
  }
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @param[in] model The model type to use to influence the conversion.
/// @return The native equivalent of the enum.
/// Cần hàm này 
uint8_t IRMyProtocolAc::convertFan(const stdAc::fanspeed_t speed){
                               //const mirage_ac_remote_model_t model) {
  //uint8_t low;
  //uint8_t med;
  //switch (model) {
    //case mirage_ac_remote_model_t::KKG29AC1:
      //low = kMirageAcKKG29AC1FanLow;
     // med = kMirageAcKKG29AC1FanMed;
      //break;
    //default:
     // low = kMirageAcFanLow;
      //med = kMirageAcFanMed;
  //}
  switch (speed) {
    case stdAc::fanspeed_t::kMin:
    case stdAc::fanspeed_t::kLow:    return kMyProtocolAcFanLow;
    case stdAc::fanspeed_t::kMedium: return kMyProtocolAcFanMed;
    case stdAc::fanspeed_t::kHigh:   return kMyProtocolAcFanHigh;
    case stdAc::fanspeed_t::kMax:    
    default:                         return kMyProtocolAcFanAuto;
  }
}

/// Convert a stdAc::swingv_t enum into it's native setting.
/// @param[in] position The enum to be converted.
/// @return The native equivalent of the enum.
// uint8_t IRMirageAc::convertSwingV(const stdAc::swingv_t position) {
//   switch (position) {
//     case stdAc::swingv_t::kHighest:     return kMirageAcSwingVHighest;
//     case stdAc::swingv_t::kHigh:        return kMirageAcSwingVHigh;
//     case stdAc::swingv_t::kMiddle:      return kMirageAcSwingVMiddle;
//     case stdAc::swingv_t::kLow:         return kMirageAcSwingVLow;
//     case stdAc::swingv_t::kLowest:      return kMirageAcSwingVLowest;
//     case stdAc::swingv_t::kOff:         return kMirageAcSwingVOff;
//     default:                            return kMirageAcSwingVAuto;
//   }
// }

/// Convert a native vertical swing postion to it's common equivalent.
/// @param[in] pos A native position to convert.
/// @return The common vertical swing position.
// stdAc::swingv_t IRMirageAc::toCommonSwingV(const uint8_t pos) {
//   switch (pos) {
//     case kMirageAcSwingVHighest: return stdAc::swingv_t::kHighest;
//     case kMirageAcSwingVHigh:    return stdAc::swingv_t::kHigh;
//     case kMirageAcSwingVMiddle:  return stdAc::swingv_t::kMiddle;
//     case kMirageAcSwingVLow:     return stdAc::swingv_t::kLow;
//     case kMirageAcSwingVLowest:  return stdAc::swingv_t::kLowest;
//     case kMirageAcSwingVAuto:    return stdAc::swingv_t::kAuto;
//     default:                     return stdAc::swingv_t::kOff;
//   }
// }

/// Convert the current internal state into its stdAc::state_t equivalent.
/// @return The stdAc equivalent of the native settings.
stdAc::state_t IRMyProtocolAc::toCommon(void) const {
  stdAc::state_t result;
  result.protocol = decode_type_t::MYPROTOCOL;
  result.model = _model;
  result.power = getPower();
  result.mode = toCommonMode(_.Mode);
  result.celsius = true;
  result.degrees = getTemp();
  result.fanspeed = toCommonFanSpeed(getFan());
  //result.swingv = toCommonSwingV(getSwingV());
  //result.swingh = getSwingH() ? stdAc::swingh_t::kAuto : stdAc::swingh_t::kOff;
  //result.turbo = getTurbo();
  //result.light = getLight();
  //result.clean = getCleanToggle();
  //result.filter = getFilter();
  result.sleep = getSleep() ? 0 : -1;
  //result.quiet = getQuiet();
  result.clock = getClock() / 60;
  // Not supported.
  result.econo = false;
  result.beep = false;
  return result;
}

/// Convert & set a stdAc::state_t to its equivalent internal settings.
/// @param[in] state The desired state in stdAc::state_t form.
void IRMyProtocolAc::fromCommon(const stdAc::state_t state) {
  stateReset();
  _model = (myprotocol_ac_remote_model_t)state.model;  // Set directly to avoid loop
  setPower(state.power);
  setTemp(state.celsius ? state.degrees : fahrenheitToCelsius(state.degrees));
  setMode(convertMode(state.mode));
  setFan(convertFan(state.fanspeed));
  //setTurbo(state.turbo);
  setSleep(state.sleep >= 0);
  //setLight(state.light);
  //setSwingV(convertSwingV(state.swingv));
  //setSwingH(state.swingh != stdAc::swingh_t::kOff);
  //setQuiet(state.quiet);
  //setCleanToggle(state.clean);
  //setFilter(state.filter);
  // setClock() expects seconds, not minutes.
  setClock((state.clock > 0) ? state.clock * 60 : 0);
  // Non-common settings.
  setOnTimer(0);
  setOffTimer(0);
  //setIFeel(false);
}

/// Convert the internal state into a human readable string.
/// @return A string containing the settings in human-readable form.
String IRMyProtocolAc::toString(void) const {
  String result = "";
  result.reserve(240);  // Reserve some heap for the string to reduce fragging.
  result += addModelToString(decode_type_t::MYPROTOCOL, _model, false);
  result += addBoolToString(getPower(), kPowerStr);
  result += addModeToString(_.Mode, 0xFF, kMyProtocolAcCool, kMyProtocolAcHeat,
                            kMyProtocolAcDry, kMyProtocolAcFan);         
  result += addTempToString(getTemp());
  //uint8_t fanlow;
  //uint8_t fanmed;
  //switch (_model) {
//     case mirage_ac_remote_model_t::KKG29AC1:
//       fanlow = kMirageAcKKG29AC1FanLow;
//       fanmed = kMirageAcKKG29AC1FanMed;
//       break;
//     default:  // e.g. Model KKG9AC1
//       fanlow = kMirageAcFanLow;
//       fanmed = kMirageAcFanMed;
//   }
  result += addFanToString(_.Fan, kMyProtocolAcFanHigh, kMyProtocolAcFanLow, kMyProtocolAcFanAuto,
                           kMyProtocolAcFanAuto, kMyProtocolAcFanMed);
//  result += addBoolToString(getTurbo(), kTurboStr);
  result += addBoolToString(getSleep(), kSleepStr);
  switch (_model) {
    case myprotocol_ac_remote_model_t::KKG21BC1:
      //result += addBoolToString(_.Quiet, kQuietStr);
      //result += addToggleToString(getLight(), kLightStr);
      //result += addBoolToString(_.SwingV, kSwingVStr);
      //result += addBoolToString(_.SwingH, kSwingHStr);
      //result += addBoolToString(_.Filter, kFilterStr);
      //result += addToggleToString(_.CleanToggle, kCleanStr);
      result += addLabeledString(getOnTimer() ? minsToString(getOnTimer())
                                              : kOffStr,
                                 kOnTimerStr);
      result += addLabeledString(getOffTimer() ? minsToString(getOffTimer())
                                               : kOffStr,
                                 kOffTimerStr);
      //result += addBoolToString(_.IFeel, kIFeelStr);
      //if (_.IFeel) {
      //  result += addIntToString(getSensorTemp(), kSensorTempStr);
       // result += 'C';
     // }
      break;
    default:  // e.g. Model KKG9AC1
      //result += addBoolToString(getLight(), kLightStr);
      //result += addSwingVToString(getSwingV(),
                                //   kMirageAcSwingVAuto,
                                //   kMirageAcSwingVHighest,
                                //   kMirageAcSwingVHigh,
                                //   0xFF,  // Unused.
                                //   kMirageAcSwingVMiddle,
                                //   0xFF,  // Unused.
                                //   kMirageAcSwingVLow,
                                //   kMirageAcSwingVLowest,
                                //   kMirageAcSwingVOff,
                                //   0xFF, 0xFF, 0xFF);  // Unused.
      result += addLabeledString(minsToString(getClock() / 60), kClockStr);
  }
  return result;
}
#endif  // DECODE_MIRAGE
