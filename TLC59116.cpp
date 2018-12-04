// some edits to this file
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "TLC59116.h"
#include "TLC59116_Unmanaged.h"

#define TLC59116_LOWLEVEL 0
#define TLC59116_DEV 0
#define TLC59116_WARNINGS 1


// extern "C" void atexit( void ) { } // so I can have statics in a method, i.e. singleton

#ifndef F_CPU
#define F_CPU 16000000L
#endif

// #define WARN Serial.print
#define WARN TLC59116Warn
#define DEV TLC59116Dev
#define LOWD TLC59116LowLevel
#define DEBUG TLC59116Warn

const unsigned char TLC59116::Power_Up_Register_Values[TLC59116_Unmanaged::Control_Register_Max + 1] PROGMEM = {
  TLC59116_Unmanaged::MODE1_OSC_mask | TLC59116_Unmanaged::MODE1_ALLCALL_mask,
  0, // mode2
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // pwm 0..15
  0xff, // grppwm
  0, // grpfreq
  0,0,0,0, // ledout0..3
  (TLC59116_Unmanaged::SUBADR1 << 1), (TLC59116_Unmanaged::SUBADR2 << 1), (TLC59116_Unmanaged::SUBADR3 << 1),
  (TLC59116_Unmanaged::AllCall_Addr << 1),
  TLC59116_Unmanaged::IREF_CM_mask | TLC59116_Unmanaged::IREF_HC_mask | ( TLC59116_Unmanaged::IREF_CC_mask & 0xff),
  0,0 // eflag1..eflag2
  };

void TLC59116::reset_happened() {
  this->reset_shadow_registers();
  }

//
// TLC59116Manager
//
// DEVELOPER NOTE:  You probably want to call this after any serial port setup so that
// you can see the messages.  Make sure TLC59116_WARNINGS is on (as it is defined in 2 places).
//
// @return:  int number of devices found
int TLC59116Manager::init() {
  // Only once. We are mean.
  WARN(F("\nTLCMgr init()\n"));
  if (this->reset_actions & Already) {
    WARN(F("TLCMgr already init'd: ")); WARN((unsigned long)&(this->i2cbus), HEX); WARN();
    //return (int)this->device_ct;
    }
  this->reset_actions |= Already;
  WARN(F("Init <i2cbus "));WARN((unsigned long)&(this->i2cbus), HEX);WARN(F("> "));WARN((unsigned long)this->init_frequency);WARN(F("hz actions:"));WARN(this->reset_actions,BIN);WARN();

  if (this->reset_actions & WireInit) {
    WARN(F("Sending i2cbus.begin\n"));
    this->i2cbus.begin();
  }
  // don't know how to set other WIRE interfaces
  if ((&(this->i2cbus) == &Wire) && (this->init_frequency != 0)) {
      TLC59116Dev(F("TWBR currently: 0d"));TLC59116Dev(TWBR, DEC);WARN(F("  TWPS: "));TLC59116Dev(TWSR & 0b11, DEC); TLC59116Dev();
      TWBR = ((F_CPU / init_frequency) - 16) / 2; // AFTER wire.begin
      WARN(F("Set TWBR: 0d"));WARN(TWBR, DEC);WARN(F("  TWPS: "));WARN(TWSR & 0b11, DEC); WARN();
    }
  else {
      WARN(F("Don't know how to set i2c frequency for non Wire\n"));
  }

  // NB. desired-frequency and actual, may not match
  WARN(F("I2C bus init'd to: ")); WARN(F_CPU / (16 + (2 * TWBR))); WARN(F("hz. F_CPU=")); WARN(F_CPU); WARN();

  int numfound = scan();       // could also use this->device_ct but direct return may be preferable


    /**
     * 12/4/2018 AFTER A FEW DAYS OF TRACING, REBOOTING, DEBUGGING, I GIVE UP.  The reset() function
     * just isn't needed on a fresh boot, but using it causes also sorts of weird glitches
     * and slow startup.   Worse, these things don't manifest with connected on serial port, so it
     * makes it really hard to track down.
     */
//  if (numfound != 0) {
//    if (reset_actions & Reset) {
//      // DEVELOPER NOTE: 12/3/2018 Traced the startup issue down to this next line.
//      reset();
//      }
//  }

  WARN(F("Init complete i2cbus "));WARN((unsigned long)&(this->i2cbus), HEX); WARN();
  return numfound;
  }

int TLC59116Manager::scan() {
  // this code lifted & adapted from Nick Gammon (written 20th April 2011)
  // http://www.gammon.com.au/forum/?id=10896&reply=6#reply6
  // Thanks Nick!
  // DEVELOPER NOTE:  According to datasheet, I2C address is 110(A3)(A2)(A1)(A0),
  // (0x60 or 0d96) when all pins are pulled low

  // TODO: no rescan supported, no add/remove supported
  WARN(F("TLCMgr scan()"));
  DEV(TLC59116::Device); DEV(F(" scanner. Scanning ..."));DEV();

  byte debug_tried = 0;

  this->device_ct = 0;
  for (byte addr = TLC59116_Unmanaged::Base_Addr; addr <= TLC59116_Unmanaged::Max_Addr; addr++) {
    debug_tried++;
    DEV("\nTrying ");DEV(addr,HEX);
    // If we hang here, then you did not do a Wire.begin();, or the frequency is too high, or communications is borked
    // yup, just "ping"
    i2cbus.beginTransmission(addr);
    int stat = i2cbus.endTransmission(); // the trick is: 0 means "something there"

    if (stat == 0) {
      if (addr == TLC59116_Unmanaged::AllCall_Addr) {DEV(F(" AllCall_Addr, skipped")); DEV(); continue; }
      if (addr == TLC59116_Unmanaged::Reset_Addr) {DEV(F(" Reset_Addr, skipped")); DEV(); continue; }

      this->devices[ this->device_ct++ ] = new TLC59116(i2cbus, addr, *this);
      WARN(F("\n\t...found "));WARN(addr,HEX);WARN();

      //::delay(10);  // maybe unneeded?
      } // end of good response
    else if (stat != 2) { // "2" means, "not there"
      WARN(F("\n! Unexpected stat(")); WARN(stat); TLC59116Warn(F(") at address ")); WARN(addr,HEX);WARN();
      }
    } // end of for loop

  DEV(F("\nChecked "));DEV(debug_tried);
  DEV(F(" in range: "));DEV(TLC59116_Unmanaged::Base_Addr,HEX); DEV(".."); DEV(TLC59116_Unmanaged::Max_Addr,HEX);
  DEV();
  
  if (this->device_ct) {
    WARN(F("Found "));
    WARN(this->device_ct);
    WARN(" ");TLC59116Warn(TLC59116_Unmanaged::Device);WARN(F("'s that responded:"));
    for(byte i=0; i<device_ct; i++) {
      WARN(" ");WARN((*this)[i].address(),HEX);
      }
    WARN();
    }
  else {
    WARN(F("None found!\n"));
    }

  return this->device_ct;
  }


/**
 * 12/4/2018 -- i recommend against using this method.  It's weird and seems to cause startup
 * issues and delays that don't make sense.  Things seem to startup just fine without it.
 * @return
 */
int TLC59116Manager::reset() {
  WARN(F("TLCMgr reset()"));WARN();
  i2cbus.beginTransmission(TLC59116_Unmanaged::Reset_Addr);
    // You might think you could:
    // i2cbus.write((const byte*)&TLC59116_Unmanaged::Reset_Bytes, size_t(TLC59116_Unmanaged::Reset_Bytes));
    i2cbus.write( TLC59116_Unmanaged::Reset_Bytes >> 8 ); 
    i2cbus.write( TLC59116_Unmanaged::Reset_Bytes & 0xFF); 
  int rez = TLC59116_Unmanaged::_end_trans(i2cbus);

  if (rez)  { 
    WARN(F("Reset failed with ")); WARN(rez); WARN();
    return rez;
    }

  WARN(F("Reset worked!")); WARN();



//------------------------COMMENTED OUT 12/3/2018---------------------------
// I find that the following blocks of code were presenting startup issues. The
// symptoms are:  crashing on startup, long (10second) delays when not connected to serial
// or just other weird issues.  I also noted that everything starts up just fine
//  WITHOUT the following code....so comment it out and move on.

    // DEVELOPER NOTE:  12/3/2018  I commented out the next line as it was causing my
    // ATMega2560 to crash.  Not sure why...but the program appears fine without it.
    //  The following for loop was already there adn seems to duplicate the functionality.
    //broadcast().reset_happened();

    // This block seems to do the same as above...redundant.
//  for (byte i=0; i< device_ct; i++) {
//      devices[i]->reset_happened();
//  }

//  TLC59116Dev(F("Reset signalled to all"));TLC59116Dev();
//    // DEVELOPER NOTE 12/4/2018
//    // THE FOLLOWING BLOCK OF CODE CAN TAKE 5-10SECONDS TO INITIALIZE....why so slow?
//    // The problem is that with a serial connection, startup is instant...why the difference?
//    // If you have the ->reset_happened() line, you MUST have the next set of lines.
//  if (this->reset_actions & EnableOutputs) {
//      //broadcast().enable_outputs(true, false);  // enable all?,  with_delay?
//      for (byte i=0; i< device_ct; i++) {
//          devices[i]->enable_outputs(true, false);
//      }
//  }
//----------------------------------------------------------------------------------

  return 0;
  }

TLC59116& TLC59116::enable_outputs(bool yes, bool with_delay) {
  WARN(F("TLCMgr enable_outputs()"));WARN();
  if (yes) {
    WARN(F("Enable outputs for "));WARN(address(),HEX);WARN();
    modify_control_register(MODE1_Register,MODE1_OSC_mask, 0x00); // bits off is osc on
    if (with_delay) {
        delayMicroseconds(500);  // would be nice to be smarter about when to do this
    }
  }
  else {
    WARN(F("Disable outputs for "));WARN(address(),HEX);WARN();
    modify_control_register(MODE1_Register,MODE1_OSC_mask, MODE1_OSC_mask); // bits on is osc off
    }

  LOWD(F("Finished en/dis-able outputs"));LOWD();
  return *this;
  }

void TLC59116::modify_control_register(byte register_num, byte value) {
  if (shadow_registers[register_num] != value) {
      shadow_registers[register_num] = value;
      LOWD(F("Modify "));LOWD(register_num,HEX);LOWD(F("=>"));LOWD(value,HEX);LOWD();
      control_register(register_num, value);
      }  
}

void TLC59116::modify_control_register(byte register_num, byte mask, byte bits) {
  byte new_value = set_with_mask(shadow_registers[register_num], mask, bits);
  if (register_num < PWM0_Register || register_num > 0x17) {
    LOWD(F("Modify R"));LOWD(register_num,HEX);LOWD();
    LOWD(F("       ="));LOWD(shadow_registers[register_num],BIN);LOWD();
    LOWD(F("       M"));LOWD(mask,BIN);LOWD();
    LOWD(F("       V"));LOWD(bits,BIN);LOWD();
    LOWD(F("       ="));LOWD(new_value,BIN);LOWD();
    }
  modify_control_register(register_num, new_value);
  }

TLC59116& TLC59116::set_outputs(word pattern, word which) {
  // Only change bits marked in which: to bits in pattern

  // We'll make the desired ledoutx register set
  byte new_ledx[4];
  // need initial value for later comparison
  memcpy(new_ledx, &(this->shadow_registers[LEDOUT0_Register]), 4);

  // count through LED nums, starting from max (backwards is easier)

  for(byte ledx_i=15; ; ledx_i--) {
    if (0x8000 & which) {
      new_ledx[ledx_i / 4] = LEDx_set_mode(
        new_ledx[ledx_i / 4], 
        ledx_i, 
        (pattern & 0x8000) ? LEDOUT_DigitalOn : LEDOUT_DigitalOff
        );
      }
    pattern <<= 1;
    which <<= 1;

    if (ledx_i==0) break; // can't detect < 0 on an unsigned!
    }

  update_registers(new_ledx, LEDOUT0_Register, LEDOUTx_Register(15));
  return *this;
  }

TLC59116& TLC59116::set_outputs(byte led_num_start, byte led_num_end, const byte brightness[] /*[ct]*/ ) {
  // We are going to start with current shadow values, mark changes against them, 
  //  then write the smallest range of registers.
  // We are going to write LEDOUTx and PWMx registers together to prevent flicker.
  // This becomes less efficient if the only changed PWMx is at a large 'x',
  // And if a LEDOUTx needs to be changed to PWM,
  // And/Or if the changed PWMx is sparse.
  // But, pretty efficient if the number of changes (from first change to last) is more than 1/2 the range.

  byte ct = led_num_end - led_num_start + 1;
  LOWD(F("Set PWMs from "));LOWD(led_num_start,HEX),LOWD(F(" for "));LOWD(ct);
    LOWD(F(": "));for(byte i=0;i<ct;i++) {LOWD(brightness[i],HEX);LOWD(F(" "));}
    LOWD();

  // need current values to mark changes against
  // FIXME: We could try to minimize the register_count: to start with the min(PWMx), but math
  byte register_count = /* r0... */ LEDOUTx_Register(Channels-1) + 1;
  LOWD(F("setup buff of 0.."));LOWD(register_count);LOWD(F("-1"));LOWD();
  byte want[register_count]; // I know that TLC59116 is: veryfew...PWMx..LEDOUTx. so waste a few at head
  memcpy(want, this->shadow_registers, register_count);

  // (for ledoutx, Might be able to do build it with <<bitmask, and then set_with_mask the whole thing)
  LOWD(F("  copying led# "));LOWD(led_num_start);LOWD(F(" to% "));LOWD((led_num_start + ct -1)%16);LOWD();
  for(byte ledi=led_num_start; ledi < led_num_start + ct; ledi++) {
    // ledout settings
    byte wrapped_i = ledi % 16; // %16 wraps
    byte out_r = LEDOUTx_Register(wrapped_i);
    want[out_r] = LEDx_set_mode(want[out_r], wrapped_i, LEDOUT_PWM);
    
    // PWM
    want[ PWMx_Register(wrapped_i) ] = brightness[ledi - led_num_start];
    LOWD(F("  setting LEDOUT @"));LOWD(out_r);LOWD(F("="));LOWD(want[out_r],HEX);
    LOWD(F(", PWM @"));LOWD(wrapped_i);LOWD(F("="));LOWD(want[ PWMx_Register(wrapped_i) ]);
    LOWD();
    }

  update_registers(&want[PWM0_Register], PWM0_Register, LEDOUTx_Register(Channels-1));
  return *this;
  }

TLC59116& TLC59116::group_pwm(word bit_pattern, byte brightness) {
  // Doing them together to minimize freq/mode being out of sync
  byte register_count = LEDOUTx_Register(Channels-1) + 1;
  byte want[register_count]; // 0..MODE2_Register...GRPPWM_Register,GRPFREQ_Register,LEDOUTx...; wasting 1

  // not touching PWM registers
  memcpy( &want[PWM0_Register], &shadow_registers[PWM0_Register],  GRPPWM_Register - PWM0_Register);
  // start with extant LEDOUTx values
  memcpy(&want[LEDOUT0_Register], &shadow_registers[LEDOUT0_Register], LEDOUTx_Register(Channels-1) - LEDOUT0_Register+1);

  want[MODE2_Register] = set_with_mask(shadow_registers[MODE2_Register], MODE2_DMBLNK, 0);
  want[GRPPWM_Register] = brightness;
  want[GRPFREQ_Register] = 0; // not actuall "don't care", it's ~dynamic_range
  LEDx_set_mode( &want[LEDOUT0_Register], LEDOUT_GRPPWM, bit_pattern);

  // do it
  update_registers(&want[MODE2_Register], MODE1_Register, register_count-1);
  return *this;
  }

TLC59116& TLC59116::group_blink(word bit_pattern, int blink_delay, int on_ratio) {
  // Doing them together to minimize freq/mode being out of sync
  byte register_count = LEDOUTx_Register(Channels-1) + 1;
  byte want[register_count]; // 0..MODE2_Register...GRPPWM_Register,GRPFREQ_Register,LEDOUTx...; wasting 1

  // not touching PWM registers
  memcpy( &want[PWM0_Register], &shadow_registers[PWM0_Register],  GRPPWM_Register - PWM0_Register);
  // start with extant LEDOUTx values
  memcpy(&want[LEDOUT0_Register], &shadow_registers[LEDOUT0_Register], LEDOUTx_Register(Channels-1) - LEDOUT0_Register+1);

  // Hypoth: DMBLNK resets the grpfreq timer. Nope.
  // want[MODE2_Register] = set_with_mask(shadow_registers[MODE2_Register], MODE2_DMBLNK, ~MODE2_DMBLNK);
  // want[GRPPWM_Register] = 0xFF;
  // want[GRPFREQ_Register] = 0;
  // LEDx_set_mode( &want[LEDOUT0_Register], LEDOUT_PWM, bit_pattern);
  // update_registers(&want[MODE1_Register], MODE1_Register, 1);

  // Hypoth: OSC resets the grpfreq timer. Nope.
  // want[MODE1_Register] = set_with_mask(shadow_registers[MODE1_Register], MODE1_OSC_mask, MODE1_OSC_mask);
  // update_registers(&want[MODE1_Register], MODE1_Register, 1);
  // delayMicroseconds(501);

  // Only Reset resets the grpfreq timer.

  want[MODE2_Register] = set_with_mask(shadow_registers[MODE2_Register], MODE2_DMBLNK, MODE2_DMBLNK);
  want[GRPPWM_Register] = on_ratio;
  want[GRPFREQ_Register] = blink_delay;
  LEDx_set_mode( &want[LEDOUT0_Register], LEDOUT_GRPPWM, bit_pattern);

  // do it
  update_registers(&want[MODE2_Register], MODE2_Register, register_count-1);
  return *this;
  }

void TLC59116::update_registers(const byte want[] /* [start..end] */, byte start_r, byte end_r) {
  // Update only the registers that need it
  // 'want' has to be (shadow_registers & new_value)
  // want[0] is register_value[start_r], i.e. a subset of the full register set
  // Does wire.begin, write,...,end
  // (_i = index of a led 0..15, _r is register_number)
  LOWD(F("Update registers ")); LOWD(start_r,HEX);LOWD(F(".."));LOWD(end_r,HEX);LOWD();

  LOWD(F("Change to ")); LOWD();
  for(byte i=0;i<end_r-start_r+1;i++){if (i>0 && i % 8 ==0) LOWD(); LOWD(want[i],BIN);LOWD(F("/"));LOWD(want[i],HEX); LOWD(" "); } 
  LOWD();

  // now find the changed ones
  //  best case is: no writes
  //  2nd best case is some subrange

  const byte *want_fullset = want - start_r; // now want[i] matches shadow_register[i]

  // First change...
  bool has_change_first = false;
  byte change_first_r; // a register num
  for (change_first_r = start_r; change_first_r <= end_r; change_first_r++) {
    if (want_fullset[change_first_r] != shadow_registers[change_first_r]) {
      has_change_first = true; // found it
      break;
      }
    }

  // Write the data if any changed

  if (!has_change_first) { // might be "none changed"
    LOWD(F("No Diff"));LOWD();
    }
  else {
    // Find last change
    byte change_last_r; // a register num
    for (change_last_r = end_r; change_last_r >= change_first_r; change_last_r--) {
      if (want_fullset[change_last_r] != shadow_registers[change_last_r]) {
        break; // found it
        }
      }
    LOWD(F("Diff "));LOWD(change_first_r,HEX);LOWD(F(".."));LOWD(change_last_r,HEX);
      LOWD(F(" ("));LOWD(change_last_r-change_first_r+1);LOWD(F(")"));
    LOWD();
    for(byte r=change_first_r; r<=change_last_r; r++) {LOWD(shadow_registers[r],HEX);LOWD(" ");} LOWD();
    for(byte r=change_first_r; r<=change_last_r; r++) {LOWD(want_fullset[r],HEX);LOWD(" ");} LOWD();

    // We have a first..last, so write them
    LOWD(F("Write "));LOWD(change_first_r,HEX);LOWD(F("+"));LOWD(change_last_r-change_first_r+1);LOWD(F(": "));
    _begin_trans(Auto_All, change_first_r);
      // TLC59116Warn("  ");
      i2cbus.write(&want_fullset[change_first_r], change_last_r-change_first_r+1);
    _end_trans();
    // update shadow
    LOWD();
    memcpy(&shadow_registers[change_first_r], &want_fullset[change_first_r], change_last_r-change_first_r+1);
    // FIXME: propagate shadow
    LOWD(F("Wrote bulk"));LOWD();
    LOWD(F("S="));for(byte i=0; i <=Control_Register_Max ; i++) { LOWD(shadow_registers[i],HEX);LOWD(F(" ")); } LOWD();
    }
  }

TLC59116& TLC59116::describe_shadow() {
  Serial.print(Device);Serial.print(F(" 0x"));Serial.print(address(),HEX);
  Serial.print(F(" on bus "));Serial.print((unsigned long)&i2cbus, HEX);Serial.println();
  Serial.println(F("Shadow Registers"));
  TLC59116_Unmanaged::describe_registers(shadow_registers); 
  return *this; 
  }

TLC59116& TLC59116::set_address(const byte address[/* sub1,sub2,sub3,all */], byte enable_mask /* MODE1_ALLCALL_mask | MODE1_SUB1_mask... */) {
  // does much:
  // sets the adresses, if != 0
  // enables/disables if corresponding address !=0
  byte want_mode1 = shadow_registers[MODE1_Register];
  byte want_addresses[4];
  memcpy(want_addresses, &shadow_registers[SUBADR1_Register], 4);

  for(byte i=0; i<4; i++) {
    if (address[i] != 0) {
      // actual address
      if (address[i]==Reset_Addr) {
        WARN(F("Ignored attempt to use the Reset_Addr (0x60) for "));
        if (i==3) WARN(F("AllCall"));
        else {
          WARN(F("SUBADR"));WARN(i+1);
          }
        WARN();
        continue;
        }

      want_addresses[i] = address[i];
      if (i==3) {
        // enable'ment
        set_with_mask(&want_mode1, MODE1_ALLCALL_mask, enable_mask);
        }
      else {
        // enable'ment
        set_with_mask(&want_mode1, MODE1_SUB1_mask >> (i), enable_mask);
        }
      }
    }
  update_registers( &want_mode1, MODE1_Register, MODE1_Register);
  update_registers( want_addresses, SUBADR1_Register, AllCall_Addr_Register);
  return *this;
  }


TLC59116& TLC59116::SUBADR_address(byte which, byte address, bool enable) {
  byte want_address[4] = {0,0,0,0};
  if (which==0 || which > 3) { WARN(F("Expected 1..3 for 'which' in SUBADR_address(), saw "));WARN(which);WARN(); return *this;}
  want_address[which-1] = address << 1;
  return set_address(want_address, enable ? MODE1_SUB1_mask >> (which-1) : 0);
  }

TLC59116& TLC59116::allcall_address(byte address, bool enable) {
  byte want_address[4] = {0,0,0,(byte)(address << 1)};    // fixed, narrowing conversion warning
  return set_address(want_address, enable ? MODE1_ALLCALL_mask : 0);
  }

TLC59116& TLC59116::allcall_address_enable() {
  byte want_mode1 = shadow_registers[MODE1_Register];
  set_with_mask(&want_mode1, MODE1_ALLCALL_mask, MODE1_ALLCALL_mask);
  update_registers( &want_mode1, MODE1_Register, MODE1_Register);
  return *this;
  }

TLC59116& TLC59116::allcall_address_disable() {
  byte want_mode1 = shadow_registers[MODE1_Register];
  set_with_mask(&want_mode1, MODE1_ALLCALL_mask, 0);
  update_registers( &want_mode1, MODE1_Register, MODE1_Register);
  return *this;
  }

TLC59116& TLC59116::resync_shadow_registers() {
  fetch_registers(0, Control_Register_Max+1, shadow_registers);  //fixed:  return value not used
  return *this;
  }

TLC59116& TLC59116::set_milliamps(byte ma, int Rext) {
  byte iref_value = best_iref(ma, Rext);
  modify_control_register(IREF_Register,iref_value);
  return *this;
  }

//
// Broadcast
//

TLC59116::Broadcast& TLC59116::Broadcast::enable_outputs(bool yes, bool with_delay) {
  WARN(yes ? F("Enable") : F("Disable")); WARN(F(" outputs for ALL")); WARN();
  TLC59116::enable_outputs(yes, with_delay);
  propagate_register(MODE1_Register);
  return *this;
  }

void TLC59116::Broadcast::propagate_register(byte register_num) {
  byte my_value = shadow_registers[register_num];
  for (byte i=0; i<= manager.device_ct; i++) { manager.devices[i]->shadow_registers[register_num]=my_value; }
  }


