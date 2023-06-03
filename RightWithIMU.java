/* 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "RiskyRiskyRight", group = "Auto")

public class RiskyRiskyRight extends LinearOpMode {
  private DcMotor frontleftAsDcMotor;
  private DcMotor backleftAsDcMotor;  
  private DcMotor frontrightAsDcMotor;
  private DcMotor backrightAsDcMotor;
  private DcMotor spinner;
  private Servo rightClaw;
  RevBlinkinLedDriver coolLights;

  int frontLeftPos;
  int backLeftPos;
  int frontRightPos;
  int backRightPos;
  int spinnerPos;
  double x = 0;
  double y = 0;
  double servoPosition = 0.0;
  double botHeading = 0.0;
  public IMU imu;
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
     private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/Final.tflite";


    private static final String[] LABELS = {
            "Black",
            "Green",
            "Magenta"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Adxgm9L/////AAABmf4X4r11gU5QjdS+o++UzoZdYE8ZWx5AnTVVr3lhgbm7NXTbtSGDU2CeUqRgcliLekQqIQtK4SCFCGmTrC9fu/fN0Mlnl1ul2djmLaT+4y7bxti+F9IMOFl2bh9yO3qeny+yyv1/uzupVJM522Jt8kEjMl6wklFQCKjow+pCDDvKQ8/HiA/HjIV4qIcc/sqnIJys6BWUt6Oj5c1NuJIIU6L7A8dkYh29xC1DHAt9jnIRefQHr7wo/OjfvqvL6x2VFkh2/o7z600lMwWjRv+X6oQ3df8JvFn3DOaOiw1Qs6pnLo4DcSZrQY0F9Y/RjM4/u+BrtF53QTw188j6t0PTrsh5hWwuUDLnp1WLA0zFZNs/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        
    frontleftAsDcMotor = hardwareMap.get(DcMotor.class, "frontleft");
    backleftAsDcMotor = hardwareMap.get(DcMotor.class, "backleft");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontright");
    backrightAsDcMotor = hardwareMap.get(DcMotor.class, "backright");
    spinner = hardwareMap.get(DcMotor.class, "spinner");
    rightClaw = hardwareMap.get(Servo.class, "claw");
    coolLights = hardwareMap.get(RevBlinkinLedDriver.class, "Lights");
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    
    // Put initialization blocks here.
    frontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    imu.initialize(parameters);
    
    frontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    frontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    spinner.setDirection(DcMotorSimple.Direction.REVERSE);
    rightClaw.setDirection(Servo.Direction.REVERSE);
      
    frontleftAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleftAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontrightAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backrightAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    frontLeftPos = 0;
    backLeftPos = 0;
    frontRightPos = 0;
    backRightPos = 0;
    rightClaw.setPosition(0);
    coolLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null){
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        // Set IMU
        imu.resetYaw();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("IMU:", botHeading);
        telemetry.update();
        //--------------------------------------
        waitForStart();
        //--------------------------------------
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        y = getRuntime();
        spinner.setTargetPosition(200);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
        sleep(300);
        Drive(350, 350, 350, 350, 0.4);
        sleep(600);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(0.5);
        //sleep(300);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
              x = getRuntime() - y;
              telemetry.addData("Time:", x);
              telemetry.update();
              if (x >= 3){
                telemetry.addData("No Objects Detected", x);
                telemetry.addData("Running Black", "");
                
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addData("IMU:", botHeading);
                telemetry.update();
                
                Movement();
                //sleep(20000);
                
                Black_Mission();
                sleep(4000);
              }
              
              
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                                                    telemetry.update();
                          if (recognition.getLabel() == "Magenta"){
                            telemetry.addData("MAGENTA", "IT WORKS!!!");
                            
                            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                            telemetry.addData("IMU:", botHeading);
                            telemetry.update();
                            
                            Movement();
                            //sleep(20000);
                            
                            Magenta_Mission();
                            sleep(10000);
                          } else if (recognition.getLabel() == "Green"){
                            telemetry.addData("GREEN", "IT WORKS!!!");
                            
                            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                            telemetry.addData("IMU:", botHeading);
                            telemetry.update();
                            
                            Movement();
                            //sleep(20000);
                            
                            Green_Mission();
                            sleep(10000);
                          } else if (recognition.getLabel() == "Black"){
                            telemetry.addData("BLACK", "IT WORKS!!!");

                            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                            telemetry.addData("IMU:", botHeading);
                            telemetry.update();
                            
                            Movement();
                            //sleep(20000);
                            
                            Black_Mission();
                            sleep(10000);
                              
                          } 
                          
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    /*
      Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    
    
    private void Drive(int frontLeftTarget, int backLeftTarget, int frontRightTarget, int backRightTarget, double speed) {
    frontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftPos = frontLeftTarget;
    backLeftPos = backLeftTarget;
    frontRightPos = frontRightTarget;
    backRightPos = backRightTarget;
    frontleftAsDcMotor.setTargetPosition(frontLeftPos);
    backleftAsDcMotor.setTargetPosition(backLeftPos);
    frontrightAsDcMotor.setTargetPosition(frontRightPos);
    backrightAsDcMotor.setTargetPosition(backRightPos);
    frontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftAsDcMotor.setPower(speed);
    backleftAsDcMotor.setPower(speed);
    frontrightAsDcMotor.setPower(speed);
    backrightAsDcMotor.setPower(speed);
  }
    
  private void posUpdate(){
      botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
      telemetry.addData("IMU:", botHeading);
      telemetry.update();
    }
  
    private void Check(double target){
      botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
      if (botHeading < target - 1.0){
        while (botHeading < target - 0.6){
          Drive(50, 50, -50, -50, 0.5);
          sleep(50);
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }while (botHeading > target + 0.4){
          Drive(-20, -20, 20, 20, 0.5);
          sleep(50);
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }
      else if (botHeading > target + 1.0){
        while (botHeading > target + 0.6){
          Drive(-50, -50, 50, 50, 0.5);
          sleep(50);
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }while (botHeading < target - 0.4){
          Drive(20, 20, -20, -20, 0.5);
          sleep(50);
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
      }
      sleep(100);
      botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
      telemetry.addData("IMU:", botHeading);
      telemetry.update();
    }
    
    
    
//EXACTLY 30 SECONDS
private void Movement(){

    //Grabs Pre-Loaded Cone
    //rightClaw.setPosition(0);
    //sleep(500);
    //Elevator Up
    
    spinner.setTargetPosition(1850);
    spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    spinner.setPower(1);
    sleep(1000);
    //////////////////////
    Check(0.0);
    //////////////////////
    //Left Shift
    Drive(575, -575, -575, 575, 0.5);
    sleep(800);
    //Forward a little
    Drive(100, 100, 100, 100, 0.5);
    sleep(350);
    //Drop Cone
    rightClaw.setPosition(0.3);
    sleep(300);
//--------------------------------------------------------------------------------------
    //Back away from low goal
    Drive(-315, -315, -315, -315, 0.5);
    sleep(500);
    //Elevator Down
    spinner.setTargetPosition(600);
    spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    spinner.setPower(1);
   //Shift Left
    Drive(575, -575, -575, 575, 0.5);
    sleep(800);
    //////////////////////////////
    Check(0.0);
    //////////////////////////////
    //Forward to parralel with Cone Stack  
    Drive(2150, 2150, 2150, 2150, 0.5);
    sleep(2500);
    Drive(-960, -960, 960, 960, 0.5);
    sleep(1200);
    //////////////////////////
    Check(-90.0);
    /////////////////////////

    //sleep(1000);
  //Forward to cone stack
    Drive(2150, 2150, 2150, 2150, 0.5);
    sleep(2000);
    //////////////////////////
    Check(-90.0);
    /////////////////////////
  //Slow Forward to cone stack
  Drive(170, 170, 170, 170, 0.3);
  sleep(500);

  
  
    //Grabs cone
    rightClaw.setPosition(0);
    sleep(300);
    Drive(-85, -85, -85, -85, 0.3);
    sleep(300);
    //Elevator Up
    spinner.setTargetPosition(4100);
    spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    spinner.setPower(1);
    sleep(2000);
  //Back to open spot
    Drive(-1015, -1015, -1015, -1015, 0.5);
    sleep(1300);

  //90 degree left rotation
    Drive(960, 960, -960, -960, 0.5);
    sleep(1200);
    ////////////////////////
    Check(0.0);
    ///////////////////////
  //shift Left 
    Drive(725, -725, -725, 725, 0.5);
    sleep(950);

  //Drive forward to high junction
    Drive(280, 280, 280, 280, 0.5);
    sleep(1000);

  //Cone drop 
  rightClaw.setPosition(0.3);   
  sleep(300);
  // Lowers arm
  spinner.setTargetPosition(0); 
  spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
  spinner.setPower(0.5);
  // backs away from high junction
  Drive(-230, -230, -230, -230, 0.5); 
  sleep(500);  
   

}
private void Magenta_Mission(){
  ///////////////////////////
    Check(0.0);
    ////////////////////////
//shift left after high junction
    Drive(650, -650, -650, 650, 0.5);
    sleep(1000);
    ///////////////////////////
    Check(0.0);
    ////////////////////////
}


  
private void Green_Mission(){
  ///////////////////////////
    Check(0.0);
    ////////////////////////
  //shift right after high junction
  Drive(-650, 650, 650, -650, 0.5);
  sleep(1000);
  ///////////////////////////
    Check(0.0);
    ////////////////////////
  }

  
private void Black_Mission(){
  ///////////////////////////
    Check(0.0);
    ////////////////////////
  //Shift right 
  Drive(-1900, 1900, 1900, -1900, 0.5);
  sleep(3000);
  ///////////////////////////
    Check(0.0);
    ////////////////////////
  }
}
