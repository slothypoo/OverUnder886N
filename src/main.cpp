#include "main.h"
#include "baseFunctions.hpp"
#include "lemlib/api.hpp"

pros::Motor leftFront(-12, pros::E_MOTOR_GEARSET_06);
pros::Motor rightFront(19, pros::E_MOTOR_GEARSET_06);
pros::Motor leftBack(-13, pros::E_MOTOR_GEARSET_06);
pros::Motor rightBack(16, pros::E_MOTOR_GEARSET_06);
pros::Motor leftMiddle(-11, pros::E_MOTOR_GEARSET_06);
pros::Motor rightMiddle(20, pros::E_MOTOR_GEARSET_06);
pros::Motor flywheelL(1, pros::E_MOTOR_GEARSET_06, false);
pros::Motor flywheelR(10, pros::E_MOTOR_GEARSET_06, true);

pros::Imu imu(3);


pros::ADIDigitalOut wings(3);	
pros::ADIDigitalOut blocker(1);
pros::ADIDigitalOut hang(5);
pros::Controller master(pros::E_CONTROLLER_MASTER);
 
// drivetrain motor group
pros::MotorGroup left_side_motors({leftFront, leftBack, leftMiddle});
pros::MotorGroup right_side_motors({rightFront, rightBack, rightMiddle});
 
lemlib::Drivetrain drivetrain(
	&left_side_motors, // left motor group
    &right_side_motors, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    360, // drivetrain rpm is 360
    2 // chase power is 2. If we had traction wheels, it would have been 8
);


pros::Rotation verticalEnc(2); 


lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275,0);
 
// inertial sensor
pros::Imu inertial_sensor(3); 
 
// odometry struct
lemlib::OdomSensors sensors {
    &vertical, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};
 

// lateral motion controller
lemlib::ControllerSettings linearController(15, // proportional gain (kP)
                                            25, // derivative gain (kD)
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             10, // derivative gain (kD)
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
 
 
// create the chassis
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

 
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}



ASSET(farSide1_txt);
ASSET(farSide2_txt);
ASSET(farSide3_txt);
ASSET(AWP1_txt);
ASSET(AWP2_txt);
ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);
ASSET(skills4_txt);
ASSET(skills5_txt);
ASSET(skills6_txt);
ASSET(skills7_txt);
ASSET(skills8_txt);
ASSET(shortAWP1_txt);
ASSET(shortAWP2_txt);
ASSET(safe1_txt);
ASSET(safe2_txt);
ASSET(safe3_txt);
ASSET(start1_txt);
ASSET(start2_txt);
ASSET(sweep1_txt);
ASSET(rush1_txt);
ASSET(rush2_txt);
ASSET(rush3_txt);
ASSET(rush4_txt);




void farSide(){
	chassis.setPose(35.293, -54.731, 0);
	//wing(1);
	//delay(200);
	//wing(0);
	powerIntake(-127);
	delay(100);
	powerIntake(0);
	chassis.follow(farSide1_txt, 15, 1600);// getting middle top triball
	chassis.waitUntil(40);
	powerIntake(127);
	chassis.waitUntilDone();
	powerIntake(0);
	chassis.turnTo(47,-1, 1000);
	chassis.waitUntilDone();
	wing(1);
	chassis.moveTo(46.032, -1.854, 90, 1200, true, 20);
	chassis.waitUntilDone();
	wing(0);
	chassis.moveTo(34, -9, 60, 2000, false, 20);
	chassis.waitUntilDone();
	chassis.turnTo(5,-31, 1000);
	chassis.waitUntilDone();
	chassis.follow(farSide2_txt, 15, 1500); // getting the third triball
	chassis.waitUntil(5);
	powerIntake(127);
	chassis.waitUntilDone();
	powerIntake(0);
	chassis.moveTo(26, -22, 260, 2000, false, 20, 0.3); 
	chassis.turnTo(51,-54, 1200);
	chassis.waitUntilDone();
	chassis.follow(farSide3_txt, 15, 1500);
	chassis.waitUntilDone();
	powerIntake(-127);
	wing(1);
	delay(800);
	wing(0);
	chassis.moveTo(61, -10, 0, 1200, true, 60, 0.6, 127);
	chassis.waitUntil(2);
	powerIntake(0);
	chassis.moveTo(61, -35, 30, 1000, false, 20, 0.1, 127);
	chassis.moveTo(63, -10, 0, 2000, true, 60, 0.1, 127);
	

 	


	//chassis.moveTo(0, 20, 0, 2000, true, 12);
}

void farSideSafe(){

	chassis.setPose(35.293, -54.731, 0);
	chassis.follow(safe1_txt, 15, 1600);
	chassis.waitUntil(2);
	powerIntake(-127);
	delay(200);
	powerIntake(0);
	chassis.waitUntil(20);
	powerIntake(127);
	chassis.waitUntilDone();
	chassis.moveTo(38, -57, 350, 1200, false, 20);
	chassis.turnTo(53,-53, 800);
	powerIntake(-127);
	delay(250);
	powerIntake(0);
	chassis.turnTo(0,-59, 800);
	chassis.moveTo(3, -59, 270, 2000, true, 20);
	chassis.waitUntil(5);
	powerIntake(127);
	chassis.moveTo(9, -59, 270, 2000, false, 20);
	chassis.turnTo(40,-59, 800);
	chassis.waitUntilDone();
	chassis.follow(safe2_txt, 15, 3000);
	chassis.waitUntil(40);
	wing(1);
	delay(200);
	wing(0);
	powerIntake(0);
	chassis.waitUntilDone();
	chassis.moveTo(56, -42, 10, 1000, false, 20);
	chassis.turnTo(7,-25, 1500);
	chassis.moveTo(7, -25, 290, 2000, true, 20);
	chassis.waitUntil(20);
	powerIntake(127);
	chassis.moveTo(16, -27, 290, 2000, false, 20);
	chassis.turnTo(48,-8,1000);
	chassis.waitUntilDone();
	powerIntake(0);
	chassis.follow(safe3_txt, 15, 1600);
	chassis.waitUntilDone();
	chassis.moveTo(37, -8, 0, 2000, false, 20);
	chassis.turnTo(7, -1, 1000);
	chassis.moveTo(7, -1, 0, 2000, true, 20);
	chassis.waitUntil(10);
	powerIntake(127);
	chassis.turnTo(49, 0, 1000);
	chassis.waitUntilDone();
	powerIntake(0);
	chassis.moveTo(49, 0, 0, 2000, true, 20);



	
}




void closeSideAWPSteal() {

	chassis.setPose(-35.293, -54.731, 0);
	powerIntake(-127);
	delay(100);
	chassis.follow(AWP1_txt, 15, 3000);// getting middle top triball
	chassis.waitUntil(1);
	powerIntake(127);
	chassis.waitUntil(10);
	powerIntake(40);
	chassis.waitUntil(30);
	wing(1);
	chassis.waitUntilDone();
	
	chassis.turnTo(-47,-9, 1500, true, 80);
	chassis.moveTo(-47, -9, 270, 2000, true, 20);
	chassis.waitUntil(3);
	wing(0);
	powerIntake(0);
	chassis.moveTo(-22, -18, 220, 1500, false, 20);
	chassis.waitUntilDone();
	chassis.follow(AWP2_txt, 15, 6000);
	chassis.waitUntilDone();
	wing(1);
	delay(800);
	wing(0);
	delay(400);
	chassis.moveTo(-8, -58, 90, 1500, true, 20);
	chassis.waitUntilDone();
	

	 

}

void closeSideAWP() {


	chassis.setPose(-47, -55, 135);
	powerIntake(-127);
	delay(200);
	powerIntake(0);
	chassis.moveTo(-50,-51.2, 135, 800, false, 20);
	chassis.waitUntilDone();
	wing(1);
	delay(400);
	wing(0);
	chassis.follow(shortAWP1_txt, 15, 1000, false);
	chassis.waitUntilDone();
	chassis.follow(shortAWP2_txt, 15, 4000, true);
	chassis.waitUntilDone();
	powerIntake(-127);
	delay(1500);
	powerIntake(0);


	 

}

void skills(){

	chassis.setPose(-47, -55.5, 315);
	
	chassis.follow(start1_txt, 15, 1200);	
	chassis.waitUntilDone();
	chassis.moveTo(-60.5, -43, 0, 1500, false, 20);
	chassis.turnTo(46, -14.1, 1000);
	chassis.waitUntilDone();
	powerIntake(-110);
	delay(26000);
	powerIntake(0);
	chassis.turnTo(-24,-70, 1200);
	chassis.waitUntilDone();
	chassis.follow(skills1_txt, 15, 4000);
	chassis.moveTo(58, -44, 20, 1200, false, 20, 0.1);
	chassis.moveTo(62, -20, 0, 1200, true, 80, 0.1);
	chassis.moveTo(58, -44, 20, 1200, false, 20, 0.1);
	chassis.moveTo(62, -20, 0, 1200, true, 80, 0.1);
	chassis.moveTo(56, -37, 0, 1200, false, 20);
	chassis.turnTo(14,-33, 1000);
	chassis.waitUntilDone();
	chassis.follow(sweep1_txt, 15, 1800);
	chassis.waitUntil(5);
	wing(1);
	chassis.waitUntilDone();
	chassis.turnTo(25,-17, 1000);
	chassis.waitUntilDone();
	chassis.follow(skills2_txt, 15, 1800);
	chassis.waitUntil(8);
	wing(0);
	chassis.waitUntilDone();
	chassis.moveTo(13, -11.5, 90, 1400, false, 20, 0.1);
	chassis.turnTo(13,33, 1000);
	chassis.moveTo(13, 33, 0, 1400, true, 20, 0.1);
	chassis.waitUntilDone();
	chassis.turnTo(47,-5, 1000);
	chassis.waitUntilDone();
	chassis.follow(skills4_txt, 15, 1800);
	chassis.waitUntil(1);
	wing(1);
	chassis.waitUntilDone();	
	wing(0);
	chassis.follow(skills5_txt, 20, 1600, false);
	chassis.waitUntilDone();
	chassis.follow(skills6_txt, 15, 1800);
	chassis.waitUntil(1);
	wing(1);
	chassis.waitUntilDone();
	wing(0);
	chassis.follow(skills7_txt, 15, 1200, false);
	chassis.waitUntilDone();
	chassis.turnTo(41, -61, 1200);
	chassis.waitUntilDone();
	chassis.follow(skills8_txt, 20, 3000);
	chassis.moveTo(58, -44, 20, 1500, false, 20, 0.1);
	chassis.moveTo(60.7, -20, 0, 1200, true, 40, 0.1);
	chassis.moveTo(58, -44, 20, 1500, false, 20, 0.1);




} 

void closeSideRush(){
	
	chassis.setPose(-35.293, -56.09, 0);
	powerIntake(-127);
	wing(1);
	delay(400);
	powerIntake(0);
	wing(0);
	
	chassis.moveTo(-27.3, -10.9, 10, 1500, true, 20, 0.4);
	chassis.waitUntil(8);
	powerIntake(127);
	chassis.waitUntilDone();
	delay(400);
	powerIntake(10);
	chassis.follow(rush2_txt, 15, 3000, false);
	chassis.waitUntilDone();
	chassis.moveTo(-60.5, -43, 0, 1500, false, 20);
	chassis.turnTo(46, -13, 1000);

}



void autonomous() {
//checks
	farSide();
	//closeSideAWP();
	//skills();
	//closeSideRush();

	
	

}




void opcontrol() {

	bool tog = false;
	bool wingState = false;
	bool blockState = false;
	

	while(true){
		
 
		leftArcade(); // Alan Preferred
	
	
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Flywheel/Intake

		if (master.get_digital_new_press(DIGITAL_R2) == true) {
			tog = !tog;
		}

		if(tog == 1){
			flywheelL = -110;
			flywheelR = -110;
		}

		if(tog == 0){
			flywheelL = 0;
			flywheelR = 0;
		}

		if (master.get_digital(DIGITAL_L1) == true && tog == 0){
			flywheelL = 127;
			flywheelR = 127;
		}
		else if(master.get_digital(DIGITAL_L1) == true){
			flywheelL = 0;
			flywheelR = 0;
		}



		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Piston wings



			if(master.get_digital_new_press(DIGITAL_Y) == 1){
				wingState = !wingState;
		    }
			
			if(wingState == false){
				wings.set_value(0);
			}
			else if(wingState == true){
				wings.set_value(1);
			}



		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~blocker


		
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~hang
		
				

			if(master.get_digital_new_press(DIGITAL_RIGHT) == 1){
				blockState = !blockState;
		    }
			
			if(blockState == false){
				hang.set_value(0);
			}
			else if(blockState == true){
				hang.set_value(1);
			}	


		pros::delay(10);

	}

}