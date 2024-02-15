

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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


@Autonomous(name = "Robot: Auto (BAZ)", group = "Robot")

public class TestingEncoderBaz extends LinearOpMode {

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

        // BAZ APRIL TAGS

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // vision portal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        // END BAZ APRIL TAGS


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

        while (!colorSelected && opModeInInit()) {
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
        while (!stageSelected && opModeInInit()) {
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


        waitForStart();
        sleep(1000);

        // MAIN LOOP
        while (opModeIsActive() && !Moved) {
            // BAZ APRIL TAGS
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x); // left-right
                telemetry.addData("y", tag.ftcPose.y); // front-back
                telemetry.addData("z", tag.ftcPose.z); // up-down
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);
                telemetry.addData("id", tag.id);

            }
            telemetry.update();
            // END BAZ APRIL TAGS

            timeFrom.startTime();
//            AutoMove(pos.getPosition());
//            Moved = true;

        }

        if (Moved) {
            if (cam != null) {

                // cam.stopStreaming();
                cam.closeCameraDevice();

            }
            super.stop();
        }
        telemetry.update();
    }


    public void AutoMove(int location) {
        if (!Moved) {
            if (redSide && back) {//RedFrontStage

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
                    encoderDrive(DRIVE_SPEED, 20, 20, 5.0);
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
                if (location == 2) {
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
            if (blueSide && back) { //blue back // augie please

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
                    encoderDrive(DRIVE_SPEED, 18, 18, 5.0);
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
            if (redSide && front) {
                if (location == 1) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    // driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20, 20, 5.0);
                    driveTurn(85, true, 1.0, 5.0);
                    encoderDrive(DRIVE_SPEED, -11 * .75, -10 * .75, 5.0);

                    ArmMotor.setPower(1);
                    sleep(2300);
                    ArmMotor.setPower(0);// for worm: 1 unit = 2.25 degrees

                    ArmServo.setPosition(.6);
                    sleep(2000);
                    ClawServoR.setPosition(.5);
                    sleep(500);
                    Moved = true;
                }
                if (location == 2) {
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
                    Moved = true;
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
                    Moved = true;
                }


            }
            if (blueSide && front) {
                if (location == 1) {
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
                    Moved = true;
                }
                if (location == 2) {
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
                    Moved = true;
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
                    Moved = true;
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


//method to count pixels in color range using openCv

