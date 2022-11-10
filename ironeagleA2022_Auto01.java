package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "IronEagleA2022_Auto01 (Blocks to Java)")
public class IronEagleA2022_Auto01 extends LinearOpMode {

  private DcMotor leftFrontDrive;
  private DcMotor leftRearDrive;
  private DistanceSensor colorSensor1_DistanceSensor;
  private DcMotor rightFrontDrive;
  private DcMotor rightRearDrive;

  double driveSpeed;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double colorSensor1Distance;
    int targetDistance;

    leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
    colorSensor1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor1");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    driveSpeed = 0.1;
    targetDistance = 6;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Determine distance
        colorSensor1Distance = colorSensor1_DistanceSensor.getDistance(DistanceUnit.CM);
        if (colorSensor1Distance > targetDistance) {
          driveForward();
        } else {
          stop2();
        }
        // Telemetry Update
        telemetry.addData("distance", colorSensor1Distance);
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void driveForward() {
    leftFrontDrive.setPower(driveSpeed);
    rightFrontDrive.setPower(driveSpeed);
    leftRearDrive.setPower(driveSpeed);
    rightRearDrive.setPower(driveSpeed);
  }

  /**
   * Describe this function...
   */
  private void stop2() {
    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(0);
    leftRearDrive.setPower(0);
    rightRearDrive.setPower(0);
  }
}
