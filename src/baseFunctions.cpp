#include "main.h"

void tareMotorsPositions(){
    leftBack.tare_position();
    leftMiddle.tare_position();
    leftFront.tare_position();
    rightBack.tare_position();
    rightMiddle.tare_position();
    rightFront.tare_position();
}


void powerIntake(double power){
    flywheelL = power;
	flywheelR = power;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void powerDrive(double power, double turn){
	leftBack = power + turn;
	leftMiddle = -power - turn;
	leftFront = power + turn;
	rightBack = power + turn;
	rightMiddle = -power - turn;
	rightFront = power + turn;
}

void wing(bool state){

	if(state == true){
		wings.set_value(true);
	}
	else if(state == false){
		wings.set_value(false);
	}
}

