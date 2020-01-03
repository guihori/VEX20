#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

double lift_place = 9589;

void move_lift(pros::Motor lift, double ticks) {

	double kp = .2;
	double ki = 0;
	double kd = 10;
	double e_t = 100;

	PID ctrl(kp, ki, kd, e_t);
	ctrl.update_target(ticks+lift.get_position());
	ctrl.update(lift.get_position(),0);

	double dt = 2;
	while(!ctrl.check_arrived())
	{

			double output = ctrl.update(lift.get_position(), dt);
			printf("lift output: %f\n", output);
			lift.move(output*127);


		printf("loop_over!\n");
		pros::delay(dt);
	}
	printf("done moving!\n");
	lift.move(0);
}

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 *l to keep execution time for this mode under a few seconds.
 */
void initialize() {
	printf("initializing\n");
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake_R(2, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift(3, pros::E_MOTOR_GEARSET_36, 1);
	// lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {14, 11, 13, 15};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	intake_R.move(255);
	intake_L.move(255);

	// Grabbing P1
	drivetrain.drive_inches(32, 80);

	// Grabbing Green Cube
	drivetrain.turn_degrees(45);
	drivetrain.drive_inches(4*sqrt(2), 80);

	intake_R.move(0);
	intake_L.move(0);

	// Going back to the wall
	drivetrain.turn_degrees(-45);
	drivetrain.drive_inches(-50,80);
	drivetrain.drive_inches(6,80);

	// Angling for P2
	drivetrain.turn_degrees(90);
	drivetrain.drive_inches(12,80);
	drivetrain.turn_degrees(-90);

	intake_R.move(255);
	intake_L.move(255);

	// Grabbing P2
	drivetrain.drive_inches(42,60);

	intake_R.move(0);
	intake_L.move(0);

	//Angling for placing
	drivetrain.drive_inches(-48, 50);
	drivetrain.drive_inches(3);
	drivetrain.turn_degrees(-95);
	drivetrain.drive_inches(36, 60);

	// Placing
	move_lift(lift, lift_place);

	// Reverse and 180
	drivetrain.drive_inches(-24, 50);
	drivetrain.turn_degrees(180);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	printf("beginning control\n");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake_R(2, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift(3, pros::E_MOTOR_GEARSET_36, 1);
	// lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {14, 11, 13, 15};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	//master.print(1,1, "" + drivetrain.test());

	while (true) {


		double x = master.get_analog(ANALOG_LEFT_X);
		double y = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		printf("Lift position: %f\n", lift.get_position());
		drivetrain.print_position();

		if(turn != 0) {
			drivetrain.turn(turn);
		}
		else {
			drivetrain.drive(y);
		}

		if(master.get_digital(DIGITAL_A)) {
			printf("intake!\n");
			intake_R.move(200);
			intake_L.move(200);
		}
		else if(master.get_digital(DIGITAL_B)) {
			printf("outtake!\n");
			intake_R.move(-200);
			intake_L.move(-200);
		}
		else if(master.get_digital(DIGITAL_Y)){
			printf("STOP TAKING!\n");
			intake_R.move(0);
			intake_L.move(0);
		}

		if(master.get_digital(DIGITAL_L1)) {
			drivetrain.turn_degrees(180);
		}
		if(master.get_digital(DIGITAL_R1)) {
			drivetrain.turn_degrees(-180);
		}

		if(master.get_digital(DIGITAL_UP)) {
			lift.move(150);
		}
		else if(master.get_digital(DIGITAL_DOWN)) {
			lift.move(-150);
		}
		else {
			lift.move(0);
		}

		pros::delay(2);
		//pros::lcd::clear();
	}
}
