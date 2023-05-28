
package org.firstinspires.ftc.teamcode.Op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode.gamepad1;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class EthanOpMode extends LinearOpMode {
  // Declare motors as a private attribute to the
  // class (Edwin isn't sure why it throws an error
  // when they are scoped to runOpMode).
  public DcMotor frontleft;
  public DcMotor frontright;
  public DcMotor backleft;
  public DcMotor backright;
  public DcMotor spinner;
  
  public Servo claw;
  //public DcMotor wave;
  //public BNO055IMU imu;
  
  
  @Override
  public void runOpMode() {
    // Inform the user that the program is initializing.
    telemetry.addData("Status", "Initializing...");
    telemetry.update();
    
    // Get motors from hardwareMap.
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    backright = hardwareMap.get(DcMotor.class, "backright");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    spinner = hardwareMap.get(DcMotor.class, "spinner"); 
    
    claw = hardwareMap.get(Servo.class, "claw");
    double clawPosition = 0.2;
    //wave = hardwareMap.get(DcMotor.class, "wave");
    //imu = hardwareMap.get(BNO055IMU.class, "imu");
    //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
    // Configure motors to go to velocity/power/position.
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //wave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    //imu.initialize(parameters);

    
    // Configure motor response when power is 0.0.
    frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //wave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    // Set the motor directions.
    frontleft.setDirection(DcMotor.Direction.FORWARD);
    frontright.setDirection(DcMotor.Direction.REVERSE);
    backleft.setDirection(DcMotor.Direction.FORWARD);
    backright.setDirection(DcMotor.Direction.REVERSE);
    spinner.setDirection(DcMotor.Direction.REVERSE);
    
    claw.setDirection(Servo.Direction.REVERSE);
    //claw.scaleRange(0, 1.0);
    claw.setPosition(clawPosition);
    //wave.setDirection(D.Direction.FORWARD);
    
    // Inform the user that everything has been initialized.
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    
    boolean twotimesspeed = false;
    boolean slow = false;
    
    while (opModeIsActive()) {
      double f_x;
      double f_y;
      double f_r;
      double f_frontleft;
      double f_frontright;
      double f_backleft;
      double f_backright;
      double speed = 0.5;
      
      boolean abot;
      boolean bbot;
      boolean xbot;
      boolean ybot;
      
      boolean lbump;
      boolean rbump;
      double rtrigger;
      double ltrigger;
      double f_wave = 0.0;
      //double botHeading = -imu.getAngularOrientation().firstAngle;
      
      telemetry.addData("Status", "Running");
      
      // Create motion vector;
      f_x = gamepad1.left_stick_x;
      f_y = gamepad1.left_stick_y;
      f_r = gamepad1.right_stick_x;
      lbump = gamepad2.left_bumper;
      rbump = gamepad2.right_bumper;
      ltrigger = gamepad2.left_trigger;
      rtrigger = gamepad2.right_trigger;
      
      
      abot = gamepad2.a;
      bbot = gamepad2.b;
      ybot = gamepad2.y;
      xbot = gamepad2.x;
      
      boolean speed_bot = gamepad1.dpad_up;
      boolean slow_bot = gamepad1.dpad_down;
      boolean reg_bot = gamepad1.dpad_right;
        
      double z = -gamepad1.left_stick_x;
      double x = gamepad1.left_stick_x;
      double y = -gamepad1.left_stick_y;
      double rotX = gamepad1.right_stick_x;
      
      
      // Give controller joysticks a deadzone.
      if (-0.1 < f_x && f_x < 0.1) {
        f_x = 0;
      }
      if (-0.1 < f_y && f_y < 0.1) {
        f_y = 0;
      }
      if (-0.1 < f_r && f_r < 0.1) {
        f_r = 0;
      }
      
      // Apply weights to each component, such that when
      // f_x and f_y are 45* on a unit circle and f_r is
      // maxed out in a direction, each motor power adds
      // up to 1.0.
      f_x *= 1;
      f_y *= 1;
      f_r *= 1;
      
      // Set up motor powers for mechanum.
      f_frontleft  = y - x - rotX;
     f_frontright = y + x + rotX;
      f_backleft   = y + x - rotX;
      f_backright  = y - x + rotX;
      
      
      if (slow_bot){
        speed = 0.3
      }
      if (speed_bot){
        speed = 0.7
      }
      if (reg_bot){
        speed = 0.5
      }
      
      f_frontleft  = f_frontleft*speed;
      f_frontright = f_frontright*speed;
      f_backleft   = f_backleft*speed;
      f_backright  = f_backright*speed;
      
      //Claw code
      if(rbump)
      {
        claw.setPosition(0);

      }
      if(lbump)
      {
        claw.setPosition(0.2);

      }
      
      //double speed button
      
      frontleft.setPower(f_frontleft);
      frontright.setPower(f_frontright);
      backleft.setPower(f_backleft);
      backright.setPower(f_backright);
      /*if (speed_bot == true){
        speed = 0.7;
        
      }
      if (slow_bot == true){
        speed = 0.01;
      }
      */
      // No need to cap values; DcMotor.setPower already
      // accounts for it.
      
      //Lift System 1
      if (rtrigger > 0.3 && spinner.getTargetPosition() <= 4400)
      {
        spinner.setTargetPosition(spinner.getTargetPosition() + 35);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(1);
      }
      if (ltrigger > 0.3 &&  spinner.getTargetPosition() >= 0)
      {
        spinner.setTargetPosition(spinner.getTargetPosition() - 35);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
      }
      
      //Lift System 2
      if(abot)
      {
        if (spinner.getTargetPosition() >= 1600){
        spinner.setTargetPosition(1700);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
        }
        else{
        spinner.setTargetPosition(1700);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(1);
        }
        
      }
      if(bbot)
      {
        
        if (spinner.getTargetPosition() >= 3100){
        spinner.setTargetPosition(3050);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
        }
        else{
        spinner.setTargetPosition(3050);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(1);
        }
        
      }
      if(ybot)
      {
        if (spinner.getTargetPosition() >= 4450){
        spinner.setTargetPosition(4400);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
        }
        else{
        spinner.setTargetPosition(4400);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(1);
          
        }
      }
      if(xbot)
      {
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
      }
      
      //spinner.setPower(f_spinner);
      //spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      
      // Update telemetry data... remove for release.
      telemetry.addData("f_frontleft", f_frontleft);
      telemetry.addData("f_frontright", f_frontright);
      telemetry.addData("f_backleft", f_backleft);
      telemetry.addData("f_backright", f_backright);
      //telemetry.addData("imu: ", imu.getAngularOrientation());
      telemetry.addData("Target Position: ", spinner.getTargetPosition());
      telemetry.addData("Target Position 2: ", spinner.getCurrentPosition());
      telemetry.addData("Claw Position: ", clawPosition);
      
      telemetry.update();
      sleep(10);
    }
  }
}


                                                                                            
