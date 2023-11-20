/* Copyright (c) 2017 FIRST. All rights reserved.
 *
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

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.IMU;
import com.sun.tools.javac.comp.Check;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Robot: Auto", group = "Robot")

public class TestingEncoder extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
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
    private boolean AlreadyDetect = false;
    private int DetectID = 0;
    IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeFrom = new ElapsedTime();
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

    @Override
    public void runOpMode() {


//
//        // Check if currentDetections is not null and not empty
//        if (currentDetections != null && !currentDetections.isEmpty()) {
//            int detectionCount = 0;
//            for (AprilTagDetection detection : currentDetections) {
//                detectionCount++;
//
//                // Display x, y, z values for each detection
//                telemetry.addData("Detection " + detectionCount, "X: %.2f, Y: %.2f, Z: %.2f inches",
//                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
//
//                // ... additional processing for each detection if needed
//            }
//
//            telemetry.addData("Total Detections", detectionCount);
//
//        } else {
//            telemetry.addData("No AprilTags Detected", " ");
//        }

        initAprilTag();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Loop and update the dashboard


        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

        // Check to see if heading reset is requested
        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        } else {
            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        }

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetryAprilTag();
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
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

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
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
        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at",  "%7d :%7d",
//                FrontLeft.getCurrentPosition(),
//                FrontRight.getCurrentPosition(), BackLeft.getCurrentPosition(),
//                BackRight.getCurrentPosition());
//        //telemetry.addData("inches per",  "%7d :%7d",COUNTS_PER_INCH);
//        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        while (opModeIsActive() && !Moved && !AlreadyDetect) {
            timeFrom.startTime();
            List<AprilTagDetection> currentDetections = aprilTag != null ? aprilTag.getDetections() : null;
//            driveTurn(45, false, 1.0, 5.0);
//            currentDetections = aprilTag != null ? aprilTag.getDetections() : null;
//            sleep(2000);
//            driveTurn(90, true, 1.0, 5.0);
//            currentDetections = aprilTag != null ? aprilTag.getDetections() : null;
//            sleep(2000);
            //  Check if currentDetections is not null and not empty
            sleep(5000);

            driveTurn(45, true, 1.0, 5.0);
            sleep(5000);
            currentDetections = aprilTag != null ? aprilTag.getDetections() : null;
            if (currentDetections != null && !currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 477) {
                        DetectID = 477;

                    }
                }
                driveTurn(45, false, 1.0, 5.0);
            }


            if (currentDetections != null && !currentDetections.isEmpty()) {
                int detectionCount = 0;  // Counter for the number of detections
                //if (Moved==0) {
                for (AprilTagDetection detection : currentDetections) {
                    sleep(1000);
                    // Process each detection
                    detectionCount++;

                    // Display x, y, z values for each detection
                    telemetry.addData("Detection " + detectionCount + " (ID " + detection.id + ")",
                            "X: %.2f, Y: %.2f, Z: %.2f inches",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);

                    // ... additional processing for each detection if needed


                    AlreadyDetect = true;
                    Moved = true;
                    //}
                    if (detection.id == 477) {//RedBackStage
                        AutoMove(477, (int) detection.ftcPose.x, false);
                    }

                    if (detection.id == 372) {//BlueBackStage
                        AutoMove(372, (int) detection.ftcPose.x, false);
                    }
                    if (detection.id == 249) {//BlueFrontStage

                    }
                    if (detection.id == 119) {//RedBackStage

                    }
                    // }
                }


                telemetry.addData("Total Detections", detectionCount);
            } else {
                assert currentDetections != null;
                timeFrom.seconds();
                driveTurn(45, true, 1.0, 5.0);
                currentDetections = aprilTag != null ? aprilTag.getDetections() : null;
                AutoMove(currentDetections.get(1).id, (int) currentDetections.get(1).ftcPose.x, true);

            }


            telemetry.update();  // Update telemetry

        }


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(DRIVE_SPEED,  47*.75,  47*.75, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // driveTurn(90,false,1.0,5.0);
        //encoderDrive(TURN_SPEED,   turn, -turn, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
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
                telemetryAprilTag();
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
        visionPortal.close();
    }

    public void AutoMove(int ID, int XPos, boolean HasRot) {
        if (!Moved) {
            if (ID == 477) {//RedFrontStage

                // DetectID = 477;

                telemetry.addData("DETECT", "477");
                telemetry.update();
                if (HasRot) {

                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
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
                if (XPos < 2) {
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
                if (XPos > 4.5) {
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

                AlreadyDetect = true;
                Moved = true;
                //}
            }
            if (ID == 372) {//RedFrontStage

                // DetectID = 477;

                telemetry.addData("DETECT", "477");
                telemetry.update();
                if (HasRot) {

                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    driveTurn(45, false, 1.0, 5.0);
                    double distance1 = 20;
                    double adjust = .75;
                    double move1 = distance1 * adjust;
                    encoderDrive(DRIVE_SPEED, 20 * .75, 20 * .75, 5.0);
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
                    driveTurn(90, true, 1.0, 5.0);
                    double distance2 = 15;
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
                if (XPos < 2) {
                    ClawServoL.setPosition(-1);
                    ClawServoR.setPosition(1);
                    encoderDrive(DRIVE_SPEED, 18 * .75, 18 * .75, 5.0);
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
                    encoderDrive(DRIVE_SPEED, -14 * .75, -14 * .75, 5.0);
                    driveTurn(87, true, 1.0, 5.0);
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

                    Moved = true;
                    //break;

                }
                if (XPos > 4.5) {
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

                AlreadyDetect = true;
                Moved = true;
                //}
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
        visionPortal.close();
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

    private void initAprilTag() {

        // Create the AprilTag processor.
        AprilTagMetadata RedBack;
        AprilTagMetadata RedFront;
        AprilTagMetadata LeftBack;
        AprilTagMetadata LeftFront;


        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        RedBack = new AprilTagMetadata(372, "RB", 3.5, DistanceUnit.INCH);
        RedFront = new AprilTagMetadata(477, "RF", 3.5, DistanceUnit.INCH);
        LeftBack = new AprilTagMetadata(119, "LB", 3.5, DistanceUnit.INCH);
        LeftFront = new AprilTagMetadata(249, "LF", 3.5, DistanceUnit.INCH);

        myAprilTagLibraryBuilder.addTag(RedBack);
        myAprilTagLibraryBuilder.addTag(RedFront);
        myAprilTagLibraryBuilder.addTag(LeftBack);
        myAprilTagLibraryBuilder.addTag(LeftFront);


        myAprilTagLibrary = myAprilTagLibraryBuilder.build();
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        aprilTag = myAprilTagProcessorBuilder
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        //Size sz = new Size(640,480);
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(sz);
        builder.setCameraResolution(new Size(640, 480));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        // builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}

