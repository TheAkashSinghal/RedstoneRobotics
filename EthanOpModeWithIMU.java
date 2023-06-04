package org.firstinspires.ftc.teamcode.Op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
public class EthanOpModeWithIMU extends LinearOpMode {
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
  public IMU imu;
  
  
  @Override
  public void runOpMode() {
    // Inform the user that the program is initializing.
    telemetry.addData("Status", "Initializing...");
    telemetry.update();
    
    // Get motors from hardwareMap.
    frontleft = hardwareMap.get(DcMotor.class, "frontright");
    backleft = hardwareMap.get(DcMotor.class, "backright");
    frontright = hardwareMap.get(DcMotor.class, "frontleft");
    backright = hardwareMap.get(DcMotor.class, "backleft");
    spinner = hardwareMap.get(DcMotor.class, "spinner"); 
    
    claw = hardwareMap.get(Servo.class, "claw");
    double clawPosition = 0.2;
    
    double speed = 0.5;
    
    //wave = hardwareMap.get(DcMotor.class, "wave");
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    
    // Configure motors to go to velocity/power/position.
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //wave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    imu.initialize(parameters);

    
    // Configure motor response when power is 0.0.
    frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //wave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    // Set the motor directions.
    frontleft.setDirection(DcMotor.Direction.REVERSE);
    frontright.setDirection(DcMotor.Direction.FORWARD);
    backleft.setDirection(DcMotor.Direction.REVERSE);
    backright.setDirection(DcMotor.Direction.FORWARD);
    spinner.setDirection(DcMotor.Direction.REVERSE);
    
    claw.setDirection(Servo.Direction.REVERSE);
    
    claw.setPosition(clawPosition);
    //claw.scaleRange(0, 1.0);
    
    //wave.setDirection(DcMotor.Direction.FORWARD);
    
    // Inform the user that everything has been initialized.
    
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
      
      boolean abot;
      boolean bbot;
      boolean xbot;
      boolean ybot;
      
      boolean lbump;
      boolean rbump;
      double rtrigger;
      double ltrigger;
      double f_wave = 0.0;
      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      
      telemetry.addData("Status", "Running");
      
      // Create motion vector;
      f_x = gamepad1.left_stick_x;
      f_y = gamepad1.left_stick_y;
      f_r = gamepad1.right_stick_x;
      rtrigger = gamepad2.right_trigger;
      ltrigger = gamepad2.left_trigger;
      lbump = gamepad2.left_bumper;
      rbump = gamepad2.right_bumper;
      
      
      abot = gamepad2.a;
      bbot = gamepad2.b;
      ybot = gamepad2.y;
      xbot = gamepad2.x;
      
      if (gamepad1.options) {
                imu.resetYaw();
            }
      
      boolean speed_bot = gamepad1.y;
      boolean slow_bot = gamepad1.a;
      boolean reg_bot = gamepad1.b;
      
      double x = gamepad1.left_stick_x * 1.1;
      double y = -gamepad1.left_stick_y;
      double rx = gamepad1.right_stick_x;
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
      
      
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
      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      // Set up motor powers for mechanum.
      f_frontleft = (rotY + rotX + f_r);//denominator;
      f_backleft   = (rotY - rotX + f_r);//denominator;
      f_frontright = (rotY - rotX - f_r);//denominator;
      f_backright  = (rotY + rotX - f_r);//denominator;
      
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
      
      
      frontleft.setPower(f_frontleft);
      frontright.setPower(f_frontright);
      backleft.setPower(f_backleft);
      backright.setPower(f_backright);
      
      // No need to cap values; DcMotor.setPower already
      // accounts for it.
      
      //Lift System
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
      telemetry.addData("Target Position: ", spinner.getTargetPosition());
      telemetry.addData("Target Position 2: ", spinner.getCurrentPosition());

      
      telemetry.update();
      sleep(10);
    }
  }
}
