package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "IronEagleB2022_01 (Blocks to Java)")
public class IronEagleB2022_01 extends LinearOpMode {

  private DcMotor left_front_motor;
  private DcMotor left_back_motor;
  private DcMotor lift_motor_1;
  private DcMotor lift_motor_2;
  private DcMotor right_back_motor;
  private Servo claw_servo_1;
  private DistanceSensor colorSensor1_DistanceSensor;
  private ColorSensor colorSensor1;
  private DcMotor right_front_motor;

  int drivePercentage;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int liftPercentage;
    double liftPower;
    double colorSensorDistance;
    int colorSensorValue;
    String colorSensorText;

    left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
    left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
    lift_motor_1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
    lift_motor_2 = hardwareMap.get(DcMotor.class, "lift_motor_2");
    right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
    claw_servo_1 = hardwareMap.get(Servo.class, "claw_servo_1");
    colorSensor1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor1");
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

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
    drivePercentage = 80;
    liftPercentage = 100;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Basic Drive of the Robot
        leftDrivePower(gamepad1.left_stick_y);
        rightDrivePower(gamepad1.right_stick_y);
        telemetry.addData("Left Pow", left_back_motor.getPower());
        telemetry.addData("Right Pow", right_back_motor.getPower());
        // Operate the lift motor
        liftPower = powerAdjust(gamepad2.right_stick_y, liftPercentage);
        lift_motor_1.setPower(liftPower);
        lift_motor_2.setPower(liftPower);
        telemetry.addData("Lift Pow", liftPower);
        // Operate Claw
        if (gamepad2.a) {
          claw_servo_1.setPosition(0);
        }
        if (gamepad2.b) {
          claw_servo_1.setPosition(0.3);
        }
        // Sensor Data Testing
        colorSensorDistance = colorSensor1_DistanceSensor.getDistance(DistanceUnit.CM);
        colorSensorValue = colorSensor1.argb();
        colorSensorText = getColorSensorText();
        telemetry.addData("distance", colorSensorDistance);
        telemetry.addData("value", colorSensorValue);
        telemetry.addData("colors", colorSensorText);
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private double powerAdjust(float power, double percent) {
    double result;

    result = (power * percent) / 100;
    return result;
  }

  /**
   * Describe this function...
   */
  private void leftDrivePower(float power) {
    left_front_motor.setPower(powerAdjust(power, drivePercentage));
    left_back_motor.setPower(powerAdjust(power, drivePercentage));
  }

  /**
   * Describe this function...
   */
  private void rightDrivePower(float power) {
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
