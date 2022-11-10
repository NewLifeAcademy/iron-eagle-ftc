package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "IronEagleA2022_01 (Blocks to Java)")
public class IronEagleA2022_01 extends LinearOpMode {

  private DcMotor rightFrontDrive;
  private DcMotor rightRearDrive;
  private ColorSensor colorSensor1;
  private DistanceSensor colorSensor1_DistanceSensor;
  private DcMotor leftFrontDrive;
  private DcMotor leftRearDrive;

  int powerPercentage;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double driveValue;
    double strafeValue;
    int colorSensor1Value;
    double colorSensor1Distance;
    String colorSensor1Text;
    double rotateValue;

    rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
    colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    colorSensor1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor1");
    leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");

    // Reverse motor directions
    rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    // Drive speed power as a percent.
    // Set to 100 for full power.
    powerPercentage = 25;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Read Values
        driveValue = powerAdjust(gamepad1.left_stick_y);
        strafeValue = powerAdjust(gamepad1.left_stick_x);
        rotateValue = powerAdjust(gamepad1.right_stick_x);
        colorSensor1Value = colorSensor1.argb();
        colorSensor1Distance = colorSensor1_DistanceSensor.getDistance(DistanceUnit.CM);
        colorSensor1Text = getColorSensorText();
        // Execute Actions
        drivePower(driveValue, strafeValue, rotateValue);
        // Telemetry Data
        telemetry.addData("drive", driveValue);
        telemetry.addData("strafe", strafeValue);
        telemetry.addData("rotate", rotateValue);
        telemetry.addData("colorSensor1Value", colorSensor1Value);
        telemetry.addData("colorSensor1Distance", colorSensor1Distance);
        telemetry.addData("colorSensor1Text", colorSensor1Text);
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private float powerAdjust(float power) {
    float result;

    result = (power * powerPercentage) / 100;
    return result;
  }

  /**
   * Describe this function...
   */
  private String getColorSensorText() {
    String text;

    text = "";
    text += "R " + colorSensor1.red();
    text += " G " + colorSensor1.blue();
    text += " B " + colorSensor1.green();
    return text;
  }

  /**
   * Describe this function...
   */
  private void drivePower(double drive, double strafe, double rotate) {
    leftFrontDrive.setPower((drive + rotate) - strafe);
    rightFrontDrive.setPower((drive - rotate) + strafe);
    leftRearDrive.setPower(drive + rotate + strafe);
    rightRearDrive.setPower((drive - rotate) - strafe);
  }
}
