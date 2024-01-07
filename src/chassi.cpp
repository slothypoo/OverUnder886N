#include "main.h"

void leftArcade(){

   static bool driveToggle = false;

		double leftX = master.get_analog(ANALOG_LEFT_X);
		double leftY = master.get_analog(ANALOG_LEFT_Y);

		if(master.get_digital_new_press(DIGITAL_A) == 1){
			driveToggle = !driveToggle;
		}



		if (!driveToggle) { // Use !driveToggle instead of driveToggle == 0
        leftFront = leftY + leftX;
        leftBack = leftY + leftX;
        leftMiddle = leftY + leftX;
        rightFront = leftY + -leftX;
        rightBack = leftY + -leftX;
        rightMiddle = leftY + -leftX;
    	} 
		else {
        leftFront = -leftY + leftX;
        leftBack = -leftY + leftX;
        leftMiddle = -leftY + leftX;
        rightFront = -leftY + -leftX;
        rightBack = -leftY + -leftX;
        rightMiddle = -leftY + -leftX;
    	}

}


