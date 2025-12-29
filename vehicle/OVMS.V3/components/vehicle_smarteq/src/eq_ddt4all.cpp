/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          1th October 2018
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
 ;    (C) 2018       Martin Graml
 ;    (C) 2019       Thomas Heuer
 ;
 ; Permission is hereby granted, free of charge, to any person obtaining a copy
 ; of this software and associated documentation files (the "Software"), to deal
 ; in the Software without restriction, including without limitation the rights
 ; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 ; copies of the Software, and to permit persons to whom the Software is
 ; furnished to do so, subject to the following conditions:
 ;
 ; The above copyright notice and this permission notice shall be included in
 ; all copies or substantial portions of the Software.
 ;
 ; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 ; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 ; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 ; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 ; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 ; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 ; THE SOFTWARE.
 ;
 ; Most of the CAN Messages are based on https://github.com/MyLab-odyssey/ED_BMSdiag
 ; https://github.com/MyLab-odyssey/ED4scan
 */

static const char *TAG = "v-smarteq";

#include "vehicle_smarteq.h"

void OvmsVehicleSmartEQ::xsq_ddt4all(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartEQ* smarteq = GetInstance(writer);
  if (!smarteq)
    return;

  if (argc != 1) {
    writer->puts("Error: xsq ddt4all requires one numerical argument");
    return;
  }
  
  smarteq->CommandDDT4all(atoi(argv[0]), writer);
}
OvmsVehicle::vehicle_command_t OvmsVehicleSmartEQ::CommandDDT4all(int number, OvmsWriter* writer) {
  if (number < 0 || number > 1000) {
    writer->printf("DDT4all command out of range");
    return Fail;
  }

  OvmsVehicle::vehicle_command_t res = Fail;
  ESP_LOGI(TAG, "DDT4all number=%d", number);

  if(number == 999) {
    ESP_LOGI(TAG, "DDT4ALL session activated for 5 minutes");
    writer->printf("DDT4ALL session activated for 5 minutes");
    m_ddt4all = true;
    m_ddt4all_ticker = 0;
    return Success;
  }

  if((!m_ddt4all || !m_enable_write) && number > 9) {
    ESP_LOGE(TAG, "DDT4all failed / no Canbus write access or DDT4all not enabled");
    writer->printf("DDT4all failed / no Canbus write access or DDT4all not enabled");
    return Fail;
  }

  if(m_ddt4all_exec > 0) {
    writer->printf("DDT4all command cooldown active, please wait %d seconds", m_ddt4all_exec);
    return Fail;
  }

  switch (number)
  {
    case 0:
    {
      // indicator 5x on
      CommandCanVector(0x745, 0x765, {"30082002","30082002"}, false, true);
      res = Success;
      break;
    }
    case 1:
    {
      // open trunk
      CommandCanVector(0x745, 0x765, {"300500","300500","300500","300500","300500"}, false, true);
      res = Success;
      break;
    }
    case 2:
    {
      // AUTO_WIPE false
      CommandCanVector(0x74d, 0x76d, {"2E033C00"}, false, true);
      res = Success;
      break;
    }
    case 3:
    {
      // AUTO_WIPE true
      CommandCanVector(0x74d, 0x76d, {"2E033C80"}, false, true);
      res = Success;
      break;
    }
    case 4:
    {
      #ifdef CONFIG_OVMS_COMP_WIFI
        if (MyPeripherals && MyPeripherals->m_esp32wifi) {
            WifiRestart();
            res = Success;
        } else {
            res = Fail;
        }
      #else
        ESP_LOGE(TAG, "WiFi support not enabled");
        res = NotImplemented;
      #endif
      break;
    }
    case 5:
    {
      #ifdef CONFIG_OVMS_COMP_CELLULAR
        if (MyPeripherals && MyPeripherals->m_cellular_modem) {
            ModemRestart();
            res = Success;
        } else {
            res = Fail;
        }
      #else
        ESP_LOGE(TAG, "Cellular support not enabled");
        res = NotImplemented;
      #endif
      break;
    }
    // CommandCanVector(txid, rxid, hexbytes = {"30010000","30082002"}, reset CAN = true/false, CommandWakeup = true/ CommandWakeup2 = false)
    case 6:
    {
      // BIPBIP_Lock false
      CommandCanVector(0x745, 0x765, {"3B1400"}, false, true);
      res = Success;
      break;
    }
    case 7:
    {
      // BIPBIP_Lock true
      CommandCanVector(0x745, 0x765, {"3B1480"}, false, true);
      res = Success;
      break;
    }
    case 8:
    {
      // REAR_WIPER_LINK false
      CommandCanVector(0x745, 0x765, {"3B5800"}, false, true);
      res = Success;
      break;
    }
    case 9:
    {
      // REAR_WIPER_LINK true
      CommandCanVector(0x745, 0x765, {"3B5880"}, false, true);
      res = Success;
      break;
    }
    case 10:
    {
      // RKE_Backdoor_open false
      CommandCanVector(0x745, 0x765, {"3B7800"}, false, true);
      res = Success;
      break;
    }
    case 11:
    {
      // RKE_Backdoor_open true
      CommandCanVector(0x745, 0x765, {"3B7880"}, false, true);
      res = Success;
      break;
    }
    case 12:
    {
      // Precond_by_key 00
      CommandCanVector(0x745, 0x765, {"3B7700"}, false, true);
      res = Success;
      break;
    }
    case 13:
    {
      // Precond_by_key 03
      CommandCanVector(0x745, 0x765, {"3B7703"}, false, true);
      res = Success;
      break;
    }
    case 14:
    {
      // ECOMODE_PRE_Restart false
      CommandCanVector(0x745, 0x765, {"3B7600"}, false, true);
      res = Success;
      break;
    }
    case 15:
    {
      // ECOMODE_PRE_Restart true
      CommandCanVector(0x745, 0x765, {"3B7680"}, false, true);
      res = Success;
      break;
    }
    case 16:
    {
      // Charging screen false
      CommandCanVector(0x743, 0x763, {"2E013D00"}, true, false);
      res = Success;
      break;
    }
    case 17:
    {
      // Charging screen true
      CommandCanVector(0x743, 0x763, {"2E013D01"}, true, false);
      res = Success;
      break;
    }
    case 18:
    {
      // ONLY_DRL_OFF_CF false
      CommandCanVector(0x74d, 0x76d, {"2E045F00"}, false, true);
      res = Success;
      break;
    }
    case 19:
    {
      // ONLY_DRL_OFF_CF true
      CommandCanVector(0x74d, 0x76d, {"2E045F80"}, false, true);
      res = Success;
      break;
    }
    case 20:
    {
      // Welcome_Goodbye_CF false
      CommandCanVector(0x74d, 0x76d, {"2E033400"}, false, true);
      res = Success;
      break;
    }
    case 21:
    {
      // Welcome_Goodbye_CF true
      CommandCanVector(0x74d, 0x76d, {"2E033480"}, false, true);
      res = Success;
      break;
    }    
    case 22:
    {      
      // Default variant, tailgate open true, Precond_by_key 00
      CommandCanVector(0x745, 0x765, {"3B7880","3B7700"}, false, true);
      res = Success;
      break;
    }
    case 23:
    {      
      // Default variant, tailgate open false, Precond_by_key 03
      CommandCanVector(0x745, 0x765, {"3B7800","3B7703"}, false, true);
      res = Success;
      break;
    }
    case 26:
    {
      // AT_BeepInRPresent_CF false
      CommandCanVector(0x743, 0x763, {"2E014900"}, true, false);
      res = Success;
      break;
    }
    case 27:
    {
      // AT_BeepInRPresent_CF true
      CommandCanVector(0x743, 0x763, {"2E014980"}, true, false);
      res = Success;
      break;
    }
    
    case 28:
    {
      // EVStartupSoundInhibition_CF false
      CommandCanVector(0x743, 0x763, {"2E013501"}, true, false);
      res = Success;
      break;
    }
    case 29:
    {
      // EVStartupSoundInhibition_CF true
      CommandCanVector(0x743, 0x763, {"2E013500"}, true, false);
      res = Success;
      break;
    }
    case 32:
    {
      // key reminder false
      CommandCanVector(0x745, 0x765, {"3B5E00"}, false, true);
      res = Success;
      break;
    }
    case 33:
    {
      // key reminder true
      CommandCanVector(0x745, 0x765, {"3B5E80"}, false, true);
      res = Success;
      break;
    }
    case 34:
    {
      // long tempo display false
      CommandCanVector(0x745, 0x765, {"3B5700"}, false, true);
      res = Success;
      break;
    }
    case 35:
    {
      // long tempo display true
      CommandCanVector(0x745, 0x765, {"3B5780"}, false, true);
      res = Success;
      break;
    }
    case 36:
    {
      // AmbientLightPresent_CF false
      CommandCanVector(0x743, 0x763, {"2E018900"}, true, false);
      res = Success;
      break;
    }
    case 37:
    {
      // AmbientLightPresent_CF true
      CommandCanVector(0x743, 0x763, {"2E018901"}, true, false);
      res = Success;
      break;
    }
    case 40:
    {
      // ClockDisplayed_CF not displayed
      CommandCanVector(0x743, 0x763, {"2E012100"}, true, false);
      res = Success;
      break;
    }
    case 41:
    {
      // ClockDisplayed_CF displayed managed
      CommandCanVector(0x743, 0x763, {"2E012101"}, true, false);
      res = Success;
      break;
    }
    case 42:
    {
      // ClockDisplayed_CF displayed not managed
      CommandCanVector(0x743, 0x763, {"2E012102"}, true, false);
      res = Success;
      break;
    }
    case 43:
    {
      // ClockDisplayed_CF not used (EQ Smart Connect)
      CommandCanVector(0x743, 0x763, {"2E012103"}, true, false);
      res = Success;
      break;
    }
    case 44:
    {
      // Auto Light false
      CommandCanVector(0x74d, 0x76d, {"2E002300"}, false, true);
      res = Success;
      break;
    }
    case 45:
    {
      // Auto Light true
      CommandCanVector(0x74d, 0x76d, {"2E002380"}, false, true);
      res = Success;
      break;
    }
    case 46:
    {
      // Auto Light false
      CommandCanVector(0x745, 0x765, {"2E104200"}, false, true);
      res = Success;
      break;
    }
    case 47:
    {
      // Auto Light true
      CommandCanVector(0x745, 0x765, {"2E104201"}, false, true);
      res = Success;
      break;
    }
    case 48:
    {
      // Light by EMM false
      CommandCanVector(0x745, 0x765, {"3B4F00"}, false, true);
      res = Success;
      break;
    }
    case 49:
    {
      // Light by EMM true
      CommandCanVector(0x745, 0x765, {"3B4F80"}, false, true);
      res = Success;
      break;
    }
    case 50:
    {
      // max AC current limitation configuration 20A only for slow charger!
      if (!mt_obl_fastchg->AsBool()) {
        CommandCanVector(0x719, 0x739, {"2E614150"}, false, true);
        res = Success;
      } else {
        res = Fail;
      }
      break;
    }
    case 51:
    {
      // max AC current limitation configuration 32A only for slow charger!
      if (!mt_obl_fastchg->AsBool()) {
        CommandCanVector(0x719, 0x739, {"2E614180"}, false, true);
        res = Success;
      } else {
        res = Fail;
      }
      break;
    }
    case 52:
    {
      // SBRLogic_CF Standard
      CommandCanVector(0x743, 0x763, {"2E018500"}, true, false);
      res = Success;
      break;
    }
    case 53:
    {
      // SBRLogic_CF US
      CommandCanVector(0x743, 0x763, {"2E018501"}, true, false);
      res = Success;
      break;
    }
    case 54:
    {
      // FrontSBRInhibition_CF false
      CommandCanVector(0x743, 0x763, {"2E010900"}, true, false);
      res = Success;
      break;
    }
    case 55:
    {
      // FrontSBRInhibition_CF true
      CommandCanVector(0x743, 0x763, {"2E010901"}, true, false);
      res = Success;
      break;
    }
    case 56:
    {
      // Speedmeter ring (Tacho) DayBacklightsPresent_CF false
      CommandCanVector(0x743, 0x763, {"2E011800"}, true, false);
      res = Success;
      break;
    }
    case 57:
    {
      // Speedmeter ring (Tacho) DayBacklightsPresent_CF true
      CommandCanVector(0x743, 0x763, {"2E011801"}, true, false);
      res = Success;
      break;
    }
    case 58:
    {
      // AdditionnalInstrumentPresent_CF false
      CommandCanVector(0x743, 0x763, {"2E018000"}, true, false);
      res = Success;
      break;
    }
    case 59:
    {
      // AdditionnalInstrumentPresent_CF true
      CommandCanVector(0x743, 0x763, {"2E018001"}, true, false);
      res = Success;
      break;
    }
    
    case 60:
    {
      // TPMSPresent_CF false
      CommandCanVector(0x743, 0x763, {"2E010E00"}, true, false);
      res = Success;
      break;
    }
    case 61:
    {
      // TPMSPresent_CF true
      CommandCanVector(0x743, 0x763, {"2E010E01"}, true, false);
      res = Success;
      break;
    }
    
    case 62:
    {
      // DRL + Tail false
      CommandCanVector(0x74d, 0x76d, {"2E035300"}, false, true);
      res = Success;
      break;
    }
    case 63:
    {
      // DRL + Tail true
      CommandCanVector(0x74d, 0x76d, {"2E035301"}, false, true);
      res = Success;
      break;
    }
    case 66:
    {
      // DigitalSpeedometerPresent_CF off
      CommandCanVector(0x743, 0x763, {"2E013900"}, true, false);
      res = Success;
      break;
    }
    case 67:
    {
      // DigitalSpeedometerPresent_CF in mph
      CommandCanVector(0x743, 0x763, {"2E013901"}, true, false);
      res = Success;
      break;
    }
    case 68:
    {
      // DigitalSpeedometerPresent_CF in km/h
      CommandCanVector(0x743, 0x763, {"2E013902"}, true, false);
      res = Success;
      break;
    }
    case 69:
    {
      // DigitalSpeedometerPresent_CF always km/h
      CommandCanVector(0x743, 0x763, {"2E013903"}, true, false);
      res = Success;
      break;
    }
    case 100:
    {
      // ClearDiagnosticInformation.All
      CommandCanVector(0x745, 0x765, {"14FFFFFF"}, true, false);
      res = Success;
      break;
    }
    case 719:
    {
      std::string hexbytes = mt_canbyte->AsString();
      CommandCanVector(0x719, 0x739, {hexbytes}, false, true);
      res = Success;
      break;
    }
    case 743:
    {
      std::string hexbytes = mt_canbyte->AsString();
      CommandCanVector(0x743, 0x763, {hexbytes}, true, false);
      res = Success;
      break;
    }
    case 745:
    {
      std::string hexbytes = mt_canbyte->AsString();
      CommandCanVector(0x745, 0x765, {hexbytes}, false, true);
      res = Success;
      break;
    }
    case 746: // 746 is used for the 74d
    {
      std::string hexbytes = mt_canbyte->AsString();
      CommandCanVector(0x74d, 0x76d, {hexbytes}, false, true);
      res = Success;
      break;
    }
    default:
      res = Fail;
      break;
  }
  // Notify the user about the success/failed of the command
  if (res == Success) {
    char buf[70];
    snprintf(buf, sizeof(buf), "DDT4all command %d executed, %d seconds command cooldown!", number, m_ddt4all_exec);
    std::string msg(buf);
    ESP_LOGI(TAG, "%s", msg.c_str());
    writer->printf("%s", msg.c_str());
  } else {
    char buf[50];
    snprintf(buf, sizeof(buf), "DDT4all command %d failed", number);
    std::string msg(buf);
    ESP_LOGI(TAG, "%s", msg.c_str());
    writer->printf("%s", msg.c_str());
  }
  return res;
}

void OvmsVehicleSmartEQ::xsq_ddt4list(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv) {
  OvmsVehicleSmartEQ* smarteq = GetInstance(writer);
  if (!smarteq)
    return;
  
    smarteq->CommandDDT4List(verbosity, writer);
}

OvmsVehicle::vehicle_command_t OvmsVehicleSmartEQ::CommandDDT4List(int verbosity, OvmsWriter* writer) {
  writer->puts("DDT4all Command list:");
  writer->printf("--Requires car to be unlocked and ignition turned on!\n");
  writer->printf("--Usage: xsq ddt4all <number>\n");
  writer->printf("--Not all commands work with all smart 453 (ICE/ED/EQ)\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Clear diagnostic information(ClearDTC): 100\n");
  writer->printf("  Allow sending DDT4all commands for the next 5 minutes: 999\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Blink indicators 5 times: 0\n");
  writer->printf("  Open tailgate (car has to be unlocked): 1\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable auto wipe: 3\n");
  writer->printf("  Disable auto wipe: 2\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  WiFi restart: 4\n");
  writer->printf("  Cellular modem restart: 5\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--OEM alarm system required!\n");
  writer->printf("  Enable car lock sounds(beep-beep): 7\n");
  writer->printf("  Disable car lock sounds: 6\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Activate rear wiper when shifting to reverse: 9\n");
  writer->printf("  No rear wiping when shifting to reverse: 8\n");
  writer->printf("  -------------------------------\n");  
  writer->printf("--Use car key tailgate button for preconditioning\n");
  writer->printf("  Activate preconditioning by car key's tailgate button: 13\n");
  writer->printf("  No preconditioning by car key buttons: 12\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--Disable car key tailgate button for tailgate opening.\n");
  writer->printf("--This has no effect for convertible rooftop opening,\n");
  writer->printf("--convertible owners should pull the fuse for the rooftop function!\n");
  writer->printf("  Open tailgate by car key's tailgate button: 11\n");
  writer->printf("  No tailgate action by car key's tailgate button: 10\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--Swap tailgate function with preconditioning - all in one go.\n");
  writer->printf("  When using car key's tailgate button open tailgate (no preconditioning): 22\n");
  writer->printf("  When using car key's tailgate button activate preconditioning (no tailgate action): 23\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Remember ECO mode state: 15\n");
  writer->printf("  Always start with ECO mode off: 14\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable charging state screen when locked: 17\n");
  writer->printf("  Disable charging state screen when locked: 16\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Only daytime running lights off true: 19\n");
  writer->printf("  Only daytime running lights off false: 18\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Welcome/Goodbye true: 21\n");
  writer->printf("  Welcome/Goodbye false: 20\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Beep when in reverse gear true: 27\n");
  writer->printf("  Beep when reverse gear false: 26\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  EV startup sound true: 29\n");
  writer->printf("  EV startup sound false: 28\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable car key reminder: 33\n");
  writer->printf("  Disable car key reminder: 32\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable longer speed display: 35\n");
  writer->printf("  Disable longer speed display: 34\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable ambient light settings in MMI car config: 37\n");
  writer->printf("  Disable ambient light settings in MMI car config: 36\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--MMI clock in sync with radio clock(using navigation GPS signal)\n");
  writer->printf("  Clock not displayed: 40\n");
  writer->printf("  Clock displayed and managed: 41\n");
  writer->printf("  Clock displayed but not managed: 42\n");
  writer->printf("  Clock not used: 43\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--If the light sensor and indicator stalk have been retrofitted,\n");
  writer->printf("--set Auto Light 1, 2 and EMM true\n");
  writer->printf("  Auto light 1 true: 45\n");
  writer->printf("  Auto light 1 false: 44\n");
  writer->printf("  Auto light 2 true: 47\n");
  writer->printf("  Auto light 2 false: 46\n");
  writer->printf("  Light by EMM true: 49\n");
  writer->printf("  Light by EMM false: 48\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--Not for 22kW Charger!\n");
  writer->printf("  Max AC current limit 20A: 50\n");
  writer->printf("  Max AC current limit 32A: 51\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Seat belt reminder logic US: 52\n");
  writer->printf("  Seat belt reminder logic Standard: 53\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Front seat belt reminder true: 55\n");
  writer->printf("  Front seat belt reminder false: 54\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Enable speedometer day backlights: 57\n");
  writer->printf("  Disable speedometer day backlights: 56\n");
  writer->printf("  -------------------------------\n");
  writer->printf("--Only valid for ICE engines\n");
  writer->printf("  Additional instrument true: 59\n");
  writer->printf("  Additional instrument false: 58\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  TPMS present true: 61\n");
  writer->printf("  TPMS present false: 60\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Activate tail light with daytime running light: 63\n");
  writer->printf("  Do not activate tail light with daytime running light: 62\n");
  writer->printf("  -------------------------------\n");
  writer->printf("  Display digital speedometer in MMI off: 66\n");
  writer->printf("  Display digital speedometer in mph: 67\n");
  writer->printf("  Display digital speedometer in km/h: 68\n");
  writer->printf("  Always display digital speedometer in km/h: 69\n");
  return Success;
}
