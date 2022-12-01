package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "IronEagleBAuto2022_01 (Blocks to Java)", preselectTeleOp = "IronEagle-B-2022_01")
public class IronEagleBAuto2022_01 extends LinearOpMode {

  private DcMotor left_front_motor;
  private DcMotor left_back_motor;
  private DcMotor lift_motor_1;
  private DcMotor lift_motor_2;
  private Servo claw_servo_1;
  private DcMotor right_front_motor;
  private DcMotor right_back_motor;
  private ColorSensor colorSensor1;

  int drivePercentage;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int liftPercentage;
    int FORWARD;
    int REVERSE;
    int HALT;

    left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
    left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
    lift_motor_1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
    lift_motor_2 = hardwareMap.get(DcMotor.class, "lift_motor_2");
    claw_servo_1 = hardwareMap.get(Servo.class, "claw_servo_1");
    right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
    right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    left_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    left_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    lift_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    lift_motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
    claw_servo_1.setPosition(0.5);
    drivePercentage = 60;
    liftPercentage = 100;
    FORWARD = -1;
    REVERSE = 1;
    HALT = 0;
    waitForStart();
    if (opModeIsActive()) {
      // Move Robot for time
      leftDrivePower(FORWARD);
      rightDrivePower(FORWARD);
      sleep(1000);
      leftDrivePower(HALT);
      rightDrivePower(HALT);
      sleep(1000);
      leftDrivePower(REVERSE);
      rightDrivePower(REVERSE);
      sleep(1000);
      leftDrivePower(HALT);
      rightDrivePower(HALT);
    }
  }

  /**
   * Describe this function...
   */
  private double powerAdjust(double power, double percent) {
    double result;

    result = (power * percent) / 100;
    return result;
  }

  /**
   * Describe this function...
   */
  private void leftDrivePower(double power) {
    left_front_motor.setPower(powerAdjust(power, drivePercentage));
    left_back_motor.setPower(powerAdjust(power, drivePercentage));
  }

  /**
   * Describe this function...
   */
  private void rightDrivePower(double power) {
    right_front_motor.setPower(powerAdjust(power, drivePercentage));
    right_back_motor.setPower(powerAdjust(power, drivePercentage));
  }

  /**
   * Describe this function...
   */
  private String getColorSensorText() {
    String text;

    text = " R" + colorSensor1.red() + " G" + colorSensor1.green() + " B" + colorSensor1.blue();
    return text;
  }
}
