#include <iostream>
#include "main.h"
#include <cstring>
#include <cmath>
// Define five stages in flight
enum flight_status{
    pre_launch,
    burn,
    coast,
    descent,
    standby};
flight_status currentState = flight_status::pre_launch;
//-------------------------------
//Insert division and  subtractions here
//leave potential to change to doubles
int safeSubtract(int augend, int addend){
    return augend + (~addend + 1);
}
float safeSubtractFloat(float augend, float addend) {
    return augend + (addend * -1.0f);
}
float safeDivide(float numerator, float denominator) {
    float x = denominator;
    int i;
    std::memcpy(&i, &x, sizeof(int));
    i = safeSubtract(0x7EEEEEEE,i);
    float y;
    std::memcpy(&y, &i, sizeof(float));
    y = y * (safeSubtractFloat(2.0f, x * y));
    return numerator * y;
}
//-------------------------------
//Insert Kalman filter here
float runKalmanFilter(float input){
//not ready!
    return input;
}
//-------------------------------
//-------------------------------
//Sensors
//-------------------------------
float readAccelerometer(){
    //insert accelerometer drive here
    return 0;
}
float readAltitude(){
    //insert altitude drive here
    return 0;
}
//-------------------------------
//Actuators
//Dummy Actions Below
void activateBuzzer() {
    std::cout << "Buzzer ON" << std::endl;
}

void deployDrogue() {
    std::cout << "Drogue Deployed" << std::endl;
}

void deployMainChute() {
    std::cout << "Main Chute Deployed" << std::endl;
}
void transmitData(){
    std::cout << "Data Transmitting" << std::endl;
}
//-------------------------------
//Insert Buzzer Drive here
//Insert Drogue Drive here
//Insert Main Chute Drive here
//-------------------------------
//Prelaunch 
void handlePreLaunch() {
    float accel = readAccelerometer();
    std::cout << "PRE_LAUNCH";
    if (safeSubtractFloat(accel, 10.0f) > 0) {
        std::cout << "SWITCH TO BURN" << std::endl;
        currentState = flight_status::burn;
    }
}
//-------------------------------
//Burn
void handleBurn() {
    float accel = readAccelerometer();

    if (safeSubtractFloat(accel, 3.0f) < 0) {
        std::cout << "SWITCH TO COAST" << std::endl;
        currentState = flight_status::coast;
    }
}
//-------------------------------
//Coast
void handleCoast() {
    static float prevAltitude = 0;
    static bool firstRun = true;
    
    if (firstRun) {
        prevAltitude = readAltitude();
        firstRun = false;
        return; 
    }
    float currentAltitude = readAltitude();

    float rawVelocity = safeDivide(safeSubtractFloat(currentAltitude, prevAltitude),10);
    prevAltitude = currentAltitude;

    float filteredVelocity = runKalmanFilter(rawVelocity);
    if (filteredVelocity <= 0) {
        std::cout << "SWITCH TO DESCENT" << std::endl;
        currentState = flight_status::descent;
    }
}
//------------------------------
//Descent
void handleDescent() {
    static bool drogueDeployed = false;
    static bool mainchuteDeployed = false;
    if (!drogueDeployed) {
        deployDrogue();
        drogueDeployed = true;
        std::cout << "DROGUE DEPLOYED" << std::endl;
    }

    float altitude = readAltitude();

    if (safeSubtractFloat(altitude, 450.0f) <= 0 && !mainchuteDeployed) {
        deployMainChute();
        mainchuteDeployed = true;
        std::cout << "MAIN CHUTE DEPLOYED" << std::endl;
        currentState = flight_status::standby;
    }
}
//------------------------------
//Standby
void handleStandby(){
    transmitData();
    std::cout << "SYNCING DATA" << std::endl;
}
//------------------------------
//Main
int main() {
    //Change these two to somewhere else, leave main only with switch currentState
    std::cout << "SYS POWER ON..." << std::endl;
    activateBuzzer();
    //
    while (true) {
        switch (currentState) {
            case flight_status::pre_launch:
                handlePreLaunch();
                break;
            case flight_status::burn:
                handleBurn();
                break;
            case flight_status::coast:
                handleCoast();
                break;
            case flight_status::descent:
                handleDescent();
                break;
            case flight_status::standby:
                handleStandby();
                break;
        }
        
        HAL_Delay(10); 
    }

    return 0;
}
