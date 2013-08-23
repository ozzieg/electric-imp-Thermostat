//Thermostat Control
//temp input: dallas one-wire
//outputs: pin5 -> Cool Relay, pin7 -> Heat Relay, pin8 -> Fan Relay

fanOverride <- 0;
setpoint <- 77.0;
deadband <- 0.5;
systemMode <- "off";
minCoolOnTime <- 120000;
minCoolOffTime <- 300000;
minHeatOnTime <- 120000;
minHeatOffTime <- 300000;
minFanRunOffTime <- 120000;

sampleTime <- 15;
pidDirection <- 0; //by default reverse direction; above setpoint error is increased
kp <- -0.2;
ki <- -0.1 * sampleTime;
kd <- -100 / sampleTime;
heatMin <- 75.0;
coolMin <- 75.0;
outMax <- 100;
outMin <- 0.0;

lastTime <- hardware.millis();
timeSinceLast <- 0;
lastInput <- 0.0;
errSum <- 0.0;
coolState <- 0;
coolMode <- 0;
coolOnTime <- 0;
lastCoolOnTime <- 0;
coolOffTime <- 0;
lastCoolOffTime <- 0;
heatState <- 0;
heatMode <- 0;
heatOnTime <- 0;
lastHeatOnTime <- 0;
heatOffTime <- 0;
lastHeatOffTime <- 0;
fanState <- 0;
activateFanRunOffDelay <- false;
fanRunOffTime <- 0;
lastFanRunOffTime <- 0;
temp <- 0;

enum ControlStage {
    ready,
    demandon,
    running,
    demandoff,
    fanrunoff
}

hardware.pin5.configure(DIGITAL_OUT);
hardware.pin7.configure(DIGITAL_OUT);
hardware.pin8.configure(DIGITAL_OUT);
hardware.pin9.configure(ANALOG_IN);
coolPin <- hardware.pin8;
heatPin <- hardware.pin7;
fanPin <- hardware.pin5;

function owReset()
{
    // configure UART for OneWire RESET timing
    hardware.uart12.configure(9600, 8, PARITY_NONE, 1, NO_CTSRTS);
    hardware.uart12.write(0xF0);
//    hardware.uart12.flush();
    
    if ( hardware.uart12.read() == 0xF0 )
    {
        server.log("owDevice not present");
        return 0;
    } else {
//        server.log("owDevice present");
        // Switch UART to OneWire data speed timing
        hardware.uart12.configure(115200, 8, PARITY_NONE, 1, NO_CTSRTS);
        return 1;
    }
}  
function owWrite(byte)
{
// Writebit: 1 = TX 0xFF = RX else error, 0 = TX 0x00 = RX
  local bit = 0x00;
    for (local b=0; b<8; b++, byte=byte>>1)
    {
      bit=byte&0x01?0xFF:0x00;
      hardware.uart12.write(bit);
//      hardware.uart12.flush();
      if (hardware.uart12.read() != bit) server.log("owDevice write error");
    }
} 
function owRead()
{
// Readbit:  TX 0xFF, RX != TX = 0, RX = TX = 1
  local byte = 0x0;
    for (local b=0; b<8; b++)
    {
      hardware.uart12.write(0xFF);
//      hardware.uart12.flush();
      if (hardware.uart12.read() == 0xFF) byte += 0x01 << b;
    }
    return byte;
}

function owGetTemp() {
 local tempL = 0; 
 local tempH = 0; 

 server.log("Reading 18B20");
 
 owReset();
 owWrite(0xCC);  // SKIP ROM
 owWrite(0x44);  // CONVERT_T DS18B20
 imp.sleep(0.8);  // wait min 750 msec for temperature conversion to finish

 owReset();
 owWrite(0xCC); // SKIP ROM
 owWrite(0xBE); // READ_SCRATCHPAD DS18B20
 
 tempL = owRead();
 tempH = owRead()*256;
 owReset();  // use reset to stop reading the rest of the scratchpad

 temp = (tempL + tempH)/16.0; // calculate temperature from LSB and MSB
 temp = (temp * 9/5) + 32.0;//F
 return temp;
}

/**
 * PID Control
 * 
 */
function resetPID(newdirection) {
    errSum = 0;
    lastInput = 0;
    
    if( newdirection != pidDirection ) {
        //if above setpoint, error is reduced
        kp = 0 - kp;
        ki = 0 - ki;
        kd = 0 - kd;
    }
    pidDirection = newdirection;
    server.setpermanentvalues({ savedPIDDirection = pidDirection }); 
}

function compute(input)
{
    /*Compute all the working error variables*/
    local error = setpoint - input;
    server.log("PID error is " + error);
    
    errSum += (error * ki);
    server.log("errSum " + errSum);
    if( errSum > outMax ) errSum = outMax;
    else if( errSum < outMin ) errSum = outMin;
    server.log("PID accumed error is " + errSum);
    
    /*Compute PID Output*/
    local output = kp * error + errSum - kd * (input - lastInput);
    server.log("PID calc = P("+kp*error+") + I("+errSum+") - D("+kd*(input - lastInput)+") = " + output);
    if( output > outMax ) output = outMax;
    else if( output < outMin ) output = outMin;
  
    /*Remember some variables for next time*/
    lastInput = input;
    return output;
}

/**
 * HVAC Control
 */
function resetOutputs() {
    fanPin.write(0);
    fanState = 0;
    coolPin.write(0);
    coolState = 0;
    heatPin.write(0);
    heatState = 0;
}

function resetTimers() {
    coolOnTime = 0;
    lastCoolOnTime = 0;
    coolOffTime = 0;
    lastCoolOffTime = 0;
    heatOnTime = 0;
    lastHeatOnTime = 0;
    heatOffTime = 0;
    lastHeatOffTime = 0;
    activateFanRunOffDelay = false;
    fanRunOffTime = 0;
    lastFanRunOffTime = 0;
}

function resetControl() {
    if( systemMode == "heat" ) {
        resetPID(1);    
    } else if( systemMode == "cool" ) {
        resetPID(0);
    } else {
        resetPID(0);
    }
    heatState = ControlStage.ready;
    coolState = ControlStage.ready;
    resetOutputs();
    resetTimers();
}

function updateFan(newFanState) {
    server.log("calling for fan state: " + newFanState);
    
    //fan control
    if( fanOverride ) {
        fanState = 1;
    } else if( !newFanState ) {
        //calling to turn off
        //was there a runoff called for?
        if( activateFanRunOffDelay ) {
            server.log("in fan run off delay: fanRunOffTime="+fanRunOffTime);
            if( lastFanRunOffTime == 0 ) {
                lastFanRunOffTime = hardware.millis();//start timer now
            }
            fanRunOffTime += hardware.millis() - lastFanRunOffTime;
            if( fanRunOffTime >= minFanRunOffTime ) {
                //passed min run off time 
                server.log("turning off fan");
                fanRunOffTime = 0;
                lastFanRunOffTime = 0;
                activateFanRunOffDelay = false;
                fanState = newFanState;                                    
            }
        } else {
            server.log("turning off fan");
            fanState = newFanState;                
        }
    } else if ( newFanState ) {
        //calling to turn on;
        //we can safely turn on
        server.log("turning on fan");
        fanState = newFanState;
    }
    fanPin.write(fanState);    
    server.log("Fan is " + fanState);
}

function updateCool(newCoolState) {
    server.log("calling for cool state: " + newCoolState);
    //ensure heat pins are off if we are in coolmode
    heatPin.write(0);
    
    //update timers
    if( coolState ) {
        if( lastCoolOnTime == 0 ) {
            lastCoolOnTime = hardware.millis();
        }
        coolOnTime += hardware.millis() - lastCoolOnTime;
    } else {
        if( lastCoolOffTime == 0 ) {
            lastCoolOffTime = hardware.millis();
        }
        coolOffTime += hardware.millis() - lastCoolOffTime;
    }
    
    //is it calling for off?
    if( !newCoolState && coolState ) {
        //check min on timer
        if( coolOnTime >= minCoolOnTime ) {
            //good to turn off
            coolOnTime = 0;
            lastCoolOnTime = 0; //reset timer
            coolState = newCoolState;
            server.log("turning off cool");
        } else {
            server.log("in min cool on guard: coolOnTime=" + coolOnTime);
        }
    }
    //is it calling for on?
    else if ( newCoolState && !coolState ) {
        //check min off timer
        if( coolOffTime >= minCoolOffTime ) {
            //good to turn on
            coolOffTime = 0;
            lastCoolOffTime = 0;//reset timer
            coolState = newCoolState;
            server.log("turning on cool");
            activateFanRunOffDelay = false;
            updateFan(1);
        } else {
            server.log("in min cool off guard: coolOffTime="+coolOffTime);
        }
    }
    coolPin.write(coolState);
    server.log("cool state is: " + coolState);
}

function updateHeat(newHeatState) {
    server.log("calling for heat state: " + newHeatState);
    //ensure that cool pins are off in heat mode
    coolPin.write(0);
    
    //is it calling for off?
    if( !newHeatState ) {
        //check min on timer
        if( lastHeatOnTime == 0 ) {
            lastHeatOnTime = hardware.millis(); //start timer
        }
        heatOnTime += hardware.millis() - lastHeatOnTime;
        if( heatOnTime >= minHeatOnTime ) {
            //good to turn off
            heatOnTime = 0;
            lastHeatOnTime = 0;
            heatState = newHeatState;
            server.log("turning off heat");
        }
    }
    //is it calling for on?
    else if ( newHeatState ) {
        //check min off timer
        if( lastHeatOffTime == 0 ) { 
            lastHeatOffTime = hardware.millis(); //start timer
        }
        heatOffTime += hardware.millis() - lastHeatOffTime;
        if( heatOffTime >= minHeatOffTime ) {
            //good to turn on
            heatOffTime = 0;
            lastHeatOffTime = 0;
            heatState = newHeatState;
            updateFan(1);
            server.log("turning on heat");
        }
    }
    heatPin.write(heatState);
    server.log("heat state is: " + heatState);
}

function coolControl(output) {
    //this is a state machine
    //ready -> demand-on -> running -> demand-off -> off -> ready
    server.log("cool mode: " + coolMode);
    switch( coolMode ) {
        case ControlStage.ready:
            //are we getting a demand to cool?
            //is the PID output calling for cool?
            if( output > coolMin && temp > (setpoint + deadband) ) {
                //increment to next state
                updateCool(1);
                coolMode++;
            } else {
                //ensure that the system is off
                updateCool(0);
                updateFan(0);
            }
            break;
        case ControlStage.demandon:
            //are we still getting demand to turn on?
            if( output > coolMin && temp > (setpoint + deadband) ) {
                //call to turn on
                updateCool(1);
                //has it turned on?                
                if( coolState == 1 ) {
                    //go to next state
                    coolMode++;
                }
            } else {
                //we are in deadband or demand went away
                coolMode--;
            }            
            break;
        case ControlStage.running:
            //are we getting a demand to go off?
            if ( output < coolMin && temp < (setpoint - deadband) ) {
                updateCool(0);
                coolMode++;
            } else {
                updateCool(1); //keep it running
            }
            break;
        case ControlStage.demandoff:
            //are we still demanding to go off?
            if ( output < coolMin && temp < (setpoint - deadband) ) {
                updateCool(0);
                if( coolState == 0 ) {
                    //cool has been turned off; go to fan run off state
                    coolMode++;
                }
            }
            else {
                coolMode--; //go back to running
            }
            break;
        case ControlStage.fanrunoff:
            //cool has been turned off; fan is still running
            activateFanRunOffDelay = true;
            updateFan(0);
            if( fanState == 0 ) {
                coolMode = ControlStage.ready; //all done
            }
            break;
    }
    //save off new cool mode
    server.log("cool mode new: " + coolMode);
}

function heatControl(output) {
    //this is a state machine
    //ready -> demand-on -> running -> demand-off -> off -> ready
    server.log("heat mode: " + heatMode);
    switch( heatMode ) {
        case ControlStage.ready:
            //are we getting a demand to heat?
            //is the PID output calling for heat?
            if( output > heatMin && temp < (setpoint - deadband) ) {
                //increment to next state
                updateHeat(1);
                heatMode++;
            } else {
                //ensure that the system is off
                updateHeat(0);
                updateFan(0);
            }
            break;
        case ControlStage.demandon:
            //are we still getting demand to turn on?
            if( output > heatMin && temp < (setpoint - deadband) ) {
                //call to turn on
                updateHeat(1);
                //has it turned on?                
                if( heatState == 1 ) {
                    //go to next state
                    heatMode++;
                }
            } else {
                //we are in deadband or demand went away
                heatMode--;
            }            
            break;
        case ControlStage.running:
            //are we getting a demand to go off?
            if ( output < heatMin && temp > (setpoint + deadband) ) {
                updateHeat(0);
                heatMode++;
            }
            break;
        case ControlStage.demandoff:
            //are we still demanding to go off?
            if ( output < heatMin && temp > (setpoint + deadband) ) {
                updateHeat(0);
                if( heatState == 0 ) {
                    //both heat and fan turned off; go back to ready state
                    heatMode++;
                }
            }    
            else {
                heatMode--; //go back to running
            }
            break;
        case ControlStage.fanrunoff:
            //heat has been turned off; fan is still running
            activateFanRunOffDelay = true;
            updateFan(0);
            if( fanState == 0 ) {
                heatMode = ControlStage.ready; //all done
            }
            break;
    }
    //save off new heat mode
    server.log("heat mode new: " + heatMode);
}
function hvacControl(temp) {
    local output = compute(temp);
    server.log("PID output is " + output);

    switch( systemMode ) {
        case "cool":
            //we are in cool mode
            server.log("in cool mode");
            //control cool mode
            coolControl(output);
            break;
        case "heat":
            //we are in heat mode
            server.log("in heat mode");
            //heat control mode
            heatControl(output);
            break;
        case "off":
        default:
            //system is off
            server.log("in system off mode");
            resetControl();
            break;
    }    
}

function storeCurrentState() {
    local a = {    
        savedsetpoint = setpoint,
        savedsystemMode = systemMode,
        savedheatMode = heatMode,
        savedcoolMode = coolMode,
        savedfanState = fanState,
        savedheatState = heatState,
        savedcoolState = coolState,
        savedpidDirection = pidDirection,
        savedactivateFanRunOffDelay = activateFanRunOffDelay,
        savedfanRunOffTime = fanRunOffTime,
        savedlastFanRunOffTime = lastFanRunOffTime,
        savedlastTime = lastTime
    };
    server.setpermanentvalues(a);
    server.log("saving off current state.");
}
function restoreState() {
    if( "savedLastTime" in server.permanent ) lastTime = server.permanent.savedlastTime;
    if( "savedsetpoint" in server.permanent ) setpoint = server.permanent.savedsetpoint;
    if( "savedsystemMode" in server.permanent ) systemMode = server.permanent.savedsystemMode;
    if( (hardware.millis() - lastTime) < 10*60000 ) {
        //less than alloted time; restore the system
        if( "savedcoolMode" in server.permanent ) coolMode = server.permanent.savedcoolMode;
        if( "savedheatMode" in server.permanent ) heatMode = server.permanent.savedheatMode;
        if( "savedcoolState" in server.permanent ) coolState = server.permanent.savedcoolState;
        if( "savedheatState" in server.permanent ) heatState = server.permanent.savedheatState;
        if( "savedfanState" in server.permanent ) fanState = server.permanent.savedfanState;
        if( "savedpidDirection" in server.permanent ) resetPID(server.permanent.savedpidDirection);
        if( "savedactivateFanRunOffDelay" in server.permanent ) activateFanRunOffDelay = server.permanent.savedactivateFanRunOffDelay;
        if( "savedfanRunOffTime" in server.permanent ) fanRunOffTime = server.permanent.savedfanRunOffTime;
        if( "savedlastFanRunOffTime" in server.permanent ) lastFanRunOffTime = server.permanent.savedlastFanRunOffTime;         
    }
    server.log("restored previous state.");
}

// Wake up and write to the server 
function main() {
    imp.wakeup(5, main);
    timeSinceLast += hardware.millis() - lastTime;
    
    //execute if sampleTime reached
    if( timeSinceLast >= (sampleTime*1000) ) {
        timeSinceLast = 0;
        
        //read temp
        local temp = owGetTemp();
        
        //run hvac control alg
        hvacControl(temp);
        
        //update agent
        agent.send("data","DallasOneWire,"+temp+"\n"+"CoolRelay,"+coolState+"\n"+"HeatRelay,"+heatState+"\n"+"FanRelay,"+fanState+"\n"+"Setpoint,"+setpoint+"\n");
    }
    lastTime = hardware.millis();        

    //store current state to survive warm boots
    storeCurrentState();    
}

agent.on("fanControl", function(value) {
    fanOverride = value == "on" ? 1 : 0;
    fanPin.write(1);
    server.log("agent set fanControl: " + fanOverride);
}); 

agent.on("changeSetpoint", function(value) {
    local newSetpoint = value.tofloat();
    if( newSetpoint != setpoint ) {
        setpoint = newSetpoint;
        server.log("agent set changeSetpoint: " + value);        
        server.setpermanentvalues({ savedsetpoint = setpoint});
    }
});

agent.on("setMode", function(value) {
    if( value == systemMode ) {
        return;
    }
    if( value == "heat" ) {
       systemMode = "heat";
       resetControl();
    } else if ( value == "cool" ) {
       systemMode = "cool";
       resetControl();
    } else if ( value == "off" ) { 
       systemMode = "off";
       resetControl();
    }
    server.log("agent set mode: " + value);
    server.setpermanentvalues({ savedsystemMode = systemMode }); 
});

imp.configure("Thermostat", [], []);

//restore cold boot values
restoreState();

// Setup to read temperature for the first time
main();