package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp


public class LexusLX470 extends LinearOpMode {

private double drive_speed_multiplier = 0.9000000000000000000000001;

private Blinker expansion_Hub_2;
  //  private DcMotor arm_motor;
    //private Gyroscope imu;
    //private DcMotor left_motor;
    //private DcMotor right_motor;
   // private DcMotor compact_motor;
   // private Servo servo_claw_left;
    //private Servo servo_claw_right;
    //private Servo servo_compact;
   // private DcMotorController dc_drive_controller;
    private DcMotor left_back_motor;
    private DcMotor left_front_motor;
    private DcMotor right_back_motor;
    private DcMotor right_front_motor;
    private DcMotor liftmotor1;
    //private DcMotor claw_motor;
    //private DcMotor liftmotor2;
    //private DcMotor intake_motor;
    private DcMotor spin_motor;
    //private Servo move_servo;
    //private Servo grab_servo;
    //private DistanceSensor d_sensor;
    private Servo claw_servo;
    private Servo clawex_servo;
    @Override
    public void runOpMode() {
    expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        //arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
        liftmotor1 = hardwareMap.get(DcMotor.class, "liftmotor1");
        clawex_servo = hardwareMap.get(Servo.class, "clawex_servo");
      //  liftmotor2 = hardwareMap.get(DcMotor.class, "liftmotor2");
      //  intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
       // move_servo = hardwareMap.get(Servo.class, "move_servo" );
      //  grab_servo = hardwareMap.get(Servo.class, "grab_servo");
      //  grab_servo.setDirection(Servo.Direction.REVERSE);
        spin_motor = hardwareMap.get(DcMotor.class, "spin_motor");
        claw_servo = hardwareMap.get(Servo.class, "claw_servo");
        //compact_motor = hardwareMap.get(DcMotor.class, "compact_motor");
       // servo_claw_left = hardwareMap.get(Servo.class, "servo_claw_left");
        //servo_claw_right = hardwareMap.get(Servo.class, "servo_claw_right");
        //servo_claw_right.setDirection(Servo.Direction.REVERSE);
        //servo_compact = hardwareMap.get(Servo.class, "servo_compact");
        
        
        


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        //defining power variables
        double dPower = 0;
        double tPower = 0;
        //double rPower = 0;
        //double aPower = 0;
        //double cPower = 0;
        
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
           
           // telemetry.addData("Left Servo Position", servo_claw_left.getPosition());
           // telemetry.addData("Right Servo Position", servo_claw_left.getPosition());

//claw motor no move
//claw_motor.setPower(0);

 //L F motor
  dPower = this.gamepad1.left_stick_y;
    left_front_motor.setPower(-dPower * drive_speed_multiplier);
    telemetry.addData("Drive Speed", drive_speed_multiplier );
  telemetry.addData("Motor Power", dPower * drive_speed_multiplier);
  
// L B Motor
  //lPower = this.gamepad1.left_stick_y;
  left_back_motor.setPower(-dPower * drive_speed_multiplier);
  
  //R F motor
   //rPower = -this.gamepad1.right_stick_y;
     right_front_motor.setPower(dPower * drive_speed_multiplier);
   //telemetry.addData("Right Motor Power", rPower);
  
  //R B motor
//rPower = this.gamepad1.left_stick_y;
  right_back_motor.setPower(dPower * drive_speed_multiplier);
  
  

  tPower = this.gamepad1.left_stick_x;
  //Left
  if (tPower < -0.1){
  left_back_motor.setPower(tPower * drive_speed_multiplier);
  right_back_motor.setPower(tPower * drive_speed_multiplier);
  left_front_motor.setPower(tPower * drive_speed_multiplier);
  right_front_motor.setPower(tPower * drive_speed_multiplier);
  }
  
  //Right
  if (tPower > 0.1){
  left_back_motor.setPower(tPower * drive_speed_multiplier);
  right_back_motor.setPower(tPower * drive_speed_multiplier);
  left_front_motor.setPower(tPower * drive_speed_multiplier);
  right_front_motor.setPower(tPower * drive_speed_multiplier);
  }
  //Lift
  
  if (this.gamepad2.right_trigger > 0.1) {
                liftmotor1.setPower(this.gamepad2.right_trigger);
               // liftmotor2.setPower(-this.gamepad2.right_trigger);
            }
            else if (this.gamepad2.left_trigger > 0.1) {
                liftmotor1.setPower(-this.gamepad2.left_trigger);
                //liftmotor2.setPower(this.gamepad2.left_trigger);
            }
            else {
                liftmotor1.setPower(0);
               // liftmotor2.setPower(0);
            }
            //green wheel spin
    if (this.gamepad1.x){
        spin_motor.setPower(8);
    }
    if (this.gamepad1.y){
        spin_motor.setPower(0);
    }
    if (this.gamepad1.b){
     spin_motor.setPower(-8);
    }
    
    //Claw
    //Put foam tape on the claw? May add friction and can be compressed.
    
    if(this.gamepad2.a){
        claw_servo.setPosition(0.3);
    }
    if (this.gamepad2.b){
        //claw_servo.setPosition(-3);
        claw_servo.setPosition(0);
    }
    
    //extend  claw
    if(this.gamepad2.y){
     clawex_servo.setPosition(0);
    }
    if(this.gamepad2.x){
        clawex_servo.setPosition(0.35);
    }
   /* if(this.gamepad2.x){
     clawex_servo.setPosition(-270);
    }*/
  
   /*//Intake motor     
    if (this.gamepad2.x)
        intake_motor.setPower(-5);
    }
    if (this.gamepad2.y){
        intake_motor.setPower(0);
    }*/
   /* //Launch
    if (this.gamepad2.a){
        grab_servo.setPosition(0.5);
    }
    if (this.gamepad2.b){
        grab_servo.setPosition(0.1);
    }
    if (this.gamepad2.right_bumper){
        move_servo.setPosition(0.5);
    }
    if (this.gamepad2.left_bumper){
        move_servo.setPosition(0.1);
    }*/
} 
}
}
