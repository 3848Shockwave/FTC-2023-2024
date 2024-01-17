

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Robot: Auto", group = "Robot")

public class TestingEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor FrontLeft = null;
    private DcMotor ArmMotor = null;
    private DcMotor Worm = null;
    Servo ArmServo;
    Servo ClawServoR;
    Servo ClawServoL;
    private boolean Moved = false;
    boolean colorSelected = false;
    boolean stageSelected = false;
    public static boolean redSide = false;
    public static boolean blueSide = false;
    boolean front = false;
    boolean back = false;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeFrom = new ElapsedTime();


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 25.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.6;     // For figuring circumference 3.9 works
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;
    static OpenCvCamera cam = null;
    public static Position pos = new Position();


    @Override
    public void runOpMode() {
        pos.startTime();
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the drive system variables.
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        Worm = hardwareMap.get(DcMotor.class, "Worm");
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");
        ClawServoR = hardwareMap.get(Servo.class, "ClawServoR");
        ClawServoL = hardwareMap.get(Servo.class, "ClawServoL");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!colorSelected&&opModeInInit()) {
            telemetry.addData("Status", "Input color, X for blue, B for red");
            telemetry.update();
            if (gamepad1.b) {
                redSide = true;
                blueSide = false;
                colorSelected = true;
            }
            if (gamepad1.x) {
                blueSide = true;
                redSide = false;
                colorSelected = true;
            }
        }
        while (!stageSelected&&opModeInInit()) {
            telemetry.addData("Status", "Input stage position, Y for front stage, A for back stage");
            telemetry.update();
            if (gamepad1.y) {
                front = true;
                back = false;
                stageSelected = true;
            }
            if (gamepad1.a) {
                front = false;
                back = true;
                stageSelected = true;
            }
        }
        WebcamName camN = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        telemetry.addData("Status", "Waiting for start, Red: "+redSide+" Front: "+front);
        telemetry.update();
        cam = OpenCvCameraFactory.getInstance().createWebcam(camN, cameraMonitorViewId);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("CamStatus","Camera Opened");
            }

            public void onError(int errorCode) {
                telemetry.addData("Error in OpenCV", errorCode);
                telemetry.update();
            }


        });
        waitForStart();
        cam.setPipeline(new PipelineRed(telemetry));
        sleep(1000);

        while (opModeIsActive()&&!Moved) {

                timeFrom.startTime();
                AutoMove(pos.getPosition());
                Moved = true;



        }
        if(Moved) {
            if (cam != null) {

               // cam.stopStreaming();
                cam.closeCameraDevice();

            }
            super.stop();
        }
        telemetry.update();
//       if(isStopRequested()){
//           if (cam != null) {
//               cam.closeCameraDevice();
//              cam.stopStreaming();
//
//           }
//
//           // Additional cleanup if needed
//
//           super.stop();
//       }


    }




    public void AutoMove(int location) {
        if (!Moved) {
            if (redSide&&back) {//RedFrontStage

                // DetectID = 477;

                telemetry.addData("DETECT", location);
                telemetry.update();
                if (location ==1) {

                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    // driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 , 20 , 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -10 * .75, 5.0);

                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    // for worm: 1 unit = 2.25 degrees
                    driveTurn(160, false, 1.0, 5.0);
                    double distance2 = 19;
                    double move2 = distance2 * adjust;
                    encoderDrive(DRIVE_SPEED, move2, move2, 5.0);
                    driveTurn(85, false, 1.0, 5.0);
                    double distance3 = 15;
                    double move3 = distance3 * adjust;
                    encoderDrive(DRIVE_SPEED, move3, move3, 5.0);
                    driveTurn(90, true, 1.0, 5.0);
                    double distance4 = 6;
                    double move4 = distance4 * adjust;
                    encoderDrive(DRIVE_SPEED, move4, move4, 5.0);
                    ArmMotor.setPower(1);
                    sleep(500);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoL.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);

                    Moved = true;
                    // break;
                }
                if (location ==2) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 18 * .75, 18 * .75, 5.0);
                    driveTurn(15, false, 1.0, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2000);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2300);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    driveTurn(15, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -13 * .75, -13 * .75, 5.0);
                    driveTurn(87, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, 36 * .75, 36 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(500);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoL.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);

                    Moved = true;
                    //break;

                }
                if (location ==3) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
                    driveTurn(87.5, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -11 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    encoderDrive(DRIVE_SPEED, 8 * .75, 8 * .75, 5.0);
                    driveTurn(90, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, 17 * .75, 17 * .75, 5.0);
                    driveTurn(94, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, 37 * .75, 37 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(500);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoL.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    encoderDrive(DRIVE_SPEED, 3.5 * .75, 3.5 * .75, 5.0);
                    Moved = true;
                    // break;
                }

                boolean alreadyDetect = true;
                Moved = true;
            }
            //  }
            if (blueSide&back) {//blue back

                // DetectID = 477;

                telemetry.addData("DETECT", location);
                telemetry.update();
                if (location == 1) {

                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    // driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 * .80, 20 * .80, 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -11 * .75, 5.0);

                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    // for worm: 1 unit = 2.25 degrees
                    driveTurn(90, true, 1.0, 5.0);
                    double distance2 = 12;
                    double move2 = distance2 * adjust;
                    encoderDrive(DRIVE_SPEED, move2, move2, 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    double distance3 = 15;
                    double move3 = distance3 * adjust;
//                    encoderDrive(DRIVE_SPEED, move3, move3, 5.0);
//                    driveTurn(90, true, 1.0, 5.0);
//                    double distance4 = 7;
//                    double move4 = distance4 * adjust;
//                    encoderDrive(DRIVE_SPEED, move4, move4, 5.0);
                    ArmMotor.setPower(1);
                    sleep(500);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoL.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);

                    Moved = true;
                    // break;
                }
                if (location == 2) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 18 , 18 , 5.0);
                    driveTurn(15, false, 1.0, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    driveTurn(15, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -13 * .75, -13 * .75, 5.0);
                    driveTurn(75, true, 1.0, 5.0);

                    encoderDrive(DRIVE_SPEED, 40 * .75, 40 * .75, 5.0);
                   // for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(1000);
                    ClawServoL.setPosition(.4);
                    sleep(1000);
                    Moved = true;
                    //break;

                }
                if (location == 3) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
                    driveTurn(87.5, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -11 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    encoderDrive(DRIVE_SPEED, 8 * .75, 8 * .75, 5.0);
                    driveTurn(90, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, 17 * .75, 17 * .75, 5.0);
                    driveTurn(94, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, 37 * .75, 37 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(500);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoL.setPosition(.5);
                    sleep(500);
                    ArmServo.setPosition(1);
                    ArmMotor.setPower(-1);
                    sleep(1000);
                    ArmMotor.setPower(0);
                    encoderDrive(DRIVE_SPEED, 3.5 * .75, 3.5 * .75, 5.0);
                    Moved = true;
                    // break;
                }


                Moved = true;
                //}
            }
            if (redSide &&front){
                if (location==1){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    // driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 , 20 , 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -10 * .75, 5.0);

                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    Moved=true;
                }
                if (location==2){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 18 * .75, 18 * .75, 5.0);
                    driveTurn(15, false, 1.0, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2000);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2300);
                    ClawServoR.setPosition(.5);
                    Moved =true;
                }
                if (location==3){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
                    driveTurn(87.5, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -11 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    Moved =true;
                }


            }
            if (blueSide &&front){
                if (location==1){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    // driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 * .80, 20 * .80, 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -10 * .75, 5.0);

                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    Moved=true;
                }
                if (location==2){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 18 * .75, 18 * .75, 5.0);
                    driveTurn(15, false, 1.0, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2000);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2300);
                    ClawServoR.setPosition(.5);
                    Moved =true;
                }
                if (location==3){
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
                    driveTurn(87.5, false, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -11 * .75, 5.0);
                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees
                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    Moved=true;
                }
            }
        }
    }

    public void ArmDrive(double speed,
                         double wormRotate, double extend,
                         double timeoutS) {
        int newWormTarget;
        int newExtendTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newWormTarget = Worm.getCurrentPosition() + (int) (wormRotate / 2.25 * COUNTS_PER_INCH);
            newExtendTarget = ArmMotor.getCurrentPosition() + (int) (extend * COUNTS_PER_INCH);

            Worm.setTargetPosition(newWormTarget);
            ArmMotor.setTargetPosition(newExtendTarget);

            // Turn On RUN_TO_POSITION
            Worm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            // runtime.reset();
            Worm.setPower(Math.abs(speed));
            ArmMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


            // Stop all motion;
            Worm.setPower(0);
            ArmMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            Worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //sleep(250);   // optional pause after each move.
        }

    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftTarget = FrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = FrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            FrontLeft.setTargetPosition(newLeftTarget);
            FrontRight.setTargetPosition(newRightTarget);
            BackLeft.setTargetPosition(newLeftTarget);
            BackRight.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        FrontLeft.getCurrentPosition(), FrontRight.getCurrentPosition(), BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            BackLeft.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            // Turn off RUN_TO_POSITION
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

    public void driveTurn(double turnDegrees, boolean Left, double speed, double maxTime) {
        int LoR = 1;
        if (Left) {
            LoR = -1;
        }
        double turnDegConv = .19444444;
        double turn = turnDegrees * turnDegConv * LoR;
        encoderDrive(speed, turn, -turn, maxTime);
    }


}

class PipelineRed extends OpenCvPipeline {

    private final Telemetry telemetry;

    public PipelineRed(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    Scalar lowerBoundBlue = new Scalar(50, 150, 100);
    Scalar upperBoundBlue = new Scalar(255, 255, 130);
    Scalar lowerBoundRed = new Scalar(37, 37, 70);
    Scalar upperBoundRed = new Scalar(0, 85, 255);
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat centerCrop;
    Mat output = new Mat();
    Mat masks = new Mat();
    Mat maty = new Mat();
    Mat blueChannel = new Mat();
    Mat redChannel = new Mat();
    Scalar rectColor = new Scalar(255, 0, 0);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_BGR2YCrCb);

        // Define the regions of interest
        Rect leftR = new Rect(1, 1, 212, 479);
        Rect centerR = new Rect(214, 1, 212, 479);
        Rect rightR = new Rect(428, 1, 212, 479);

        input.copyTo(output);
        Imgproc.rectangle(output, leftR, rectColor, 2);
        Imgproc.rectangle(output, centerR, rectColor, 2);
        Imgproc.rectangle(output, rightR, rectColor, 2);

        // Extract color information from the regions
        leftCrop = YCbCr.submat(leftR);
        centerCrop = YCbCr.submat(centerR);
        rightCrop = YCbCr.submat(rightR);
        int pixelCountLeft = 0;
        int pixelCountCenter = 0;
        int pixelCountRight = 0;
//        if (TestingEncoder.redSide) {
//             pixelCountLeft = countPixelsInRange(leftCrop, lowerBoundRed, upperBoundRed);
//             pixelCountCenter = countPixelsInRange(centerCrop, lowerBoundRed, upperBoundRed);
//             pixelCountRight = countPixelsInRange(rightCrop, lowerBoundRed, upperBoundRed);
//        }
//        if (TestingEncoder.blueSide) {
            pixelCountLeft = countPixelsInRange(leftCrop, lowerBoundBlue, upperBoundBlue);
            pixelCountCenter = countPixelsInRange(centerCrop, lowerBoundBlue, upperBoundBlue);
            pixelCountRight = countPixelsInRange(rightCrop, lowerBoundBlue, upperBoundBlue);
      //  }



        // Print pixel counts for debugging using telemetry
        telemetry.addData("Left pixels", pixelCountLeft);
        telemetry.addData("Center pixels", pixelCountCenter);
        telemetry.addData("Right pixels", pixelCountRight);
        if (pixelCountLeft!=0||pixelCountCenter!=0||pixelCountRight!=0) {


            // Determine position based on pixel counts
            if (pixelCountLeft > pixelCountRight && pixelCountLeft > pixelCountCenter) {
                telemetry.addData("OpenCV", "Right");
                TestingEncoder.pos.setPosition(3);
                telemetry.addData("Position", "3");
                TestingEncoder.cam.closeCameraDevice();
                telemetry.update();

            } else if (pixelCountRight > pixelCountLeft && pixelCountRight > pixelCountCenter) {
                telemetry.addData("OpenCV", "Left");
                TestingEncoder.pos.setPosition(1);
                telemetry.addData("Position", "1");
                telemetry.update();
                TestingEncoder.cam.closeCameraDevice();

            } else if (pixelCountCenter > pixelCountLeft && pixelCountCenter > pixelCountRight) {
                telemetry.addData("OpenCV", "Center");
                TestingEncoder.pos.setPosition(2);
                telemetry.addData("Position", "2");
            } else {
                telemetry.addData("OpenCV", "None");
                telemetry.addData("Position", "No position detected");
            }
        }
        double avgLeft = 0;
        double avgCenter = 0;
        double avgRight = 0;

        if (TestingEncoder.redSide ) {
            avgLeft = calculateAverageRed(leftCrop);
             avgCenter = calculateAverageRed(centerCrop);
             avgRight = calculateAverageRed(rightCrop);
        }
        if (TestingEncoder.blueSide) {
                avgLeft = calculateAverageBlue(leftCrop);
                avgCenter = calculateAverageBlue(centerCrop);
                avgRight = calculateAverageBlue(rightCrop);
        }
        telemetry.addData("Left avg", avgLeft);
        telemetry.addData("Center avg", avgCenter);
        telemetry.addData("Right avg", avgRight);
        if(pixelCountLeft==0&&pixelCountCenter==0&&pixelCountRight==0){
            if (avgLeft > avgRight && avgLeft > avgCenter) {
                telemetry.addData("OpenCV", "Left");
                TestingEncoder.pos.setPosition(3);
                System.out.println("Pos 3");
            } else if (avgRight > avgLeft && avgRight > avgCenter) {
                telemetry.addData("OpenCV", "Right");
                TestingEncoder.pos.setPosition(1);
                System.out.println("Pos 1");
            } else if (avgCenter > avgLeft && avgCenter > avgRight) {
                telemetry.addData("OpenCV", "Center");
                TestingEncoder.pos.setPosition(2);
                System.out.println("Pos 2");
            } else {
                telemetry.addData("OpenCV", "None");
            }
        }
        telemetry.update();
        TestingEncoder.cam.closeCameraDevice();
        return output;


    }

//method to count pixels in color range using openCv

    private int countPixelsInRange(Mat mat, Scalar lowerBound, Scalar upperBound) {
        Core.inRange(mat, lowerBound, upperBound, masks);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(masks, contours, maty, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours.size();
    }
    private double calculateAverageBlue(Mat mat) {

        Core.extractChannel(mat, blueChannel, 0); // Extract blue channel

        Scalar mean = Core.mean(blueChannel);
        return mean.val[0]; // Average blue value
    }
    private double calculateAverageRed(Mat mat) {

        Core.extractChannel(mat, redChannel, 2); // Extract red channel

        Scalar mean = Core.mean(redChannel);
        return mean.val[0]; // Average red value
    }
}
