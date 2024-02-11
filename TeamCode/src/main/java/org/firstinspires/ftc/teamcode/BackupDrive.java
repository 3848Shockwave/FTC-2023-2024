package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Backup", group = "Linear OpMode")
public class BackupDrive extends LinearOpMode {

    final double speedLimit = 1;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime controlTime = new ElapsedTime();
    Servo ArmServo;
    Servo ClawServoR;
    Servo ClawServoL;
    Servo LaunchServo;
    boolean WristAngleMove;
    boolean angleToggle = false;
    boolean checkOne = false;
    boolean checkTwo = false;
    boolean clawR = false;
    boolean clawL = false;
    double wristAngle = 0;
    double armAngle = 0;
    double extensionInches = 0;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor FrontLeft = null;
    private DcMotor ArmMotor = null;
    private DcMotor Worm = null;

    public static double mapRange(double value, double fromMin, double fromMax, double toMin, double toMax) {
        // Ensure the value is within the source range
        value = Math.max(fromMin, Math.min(value, fromMax));

        // Calculate the normalized position of the value in the source range
        double normalized = (value - fromMin) / (fromMax - fromMin);

        // Map the normalized value to the target range
        return toMin + normalized * (toMax - toMin);
    }

    @Override
    public void runOpMode() {


        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        Worm = hardwareMap.get(DcMotor.class, "Worm");
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");
        ClawServoR = hardwareMap.get(Servo.class, "ClawServoR");
        ClawServoL = hardwareMap.get(Servo.class, "ClawServoL");
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        Worm.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            armAngle = mapRange((-1 * (((double) Worm.getCurrentPosition() / 28) * 19.2) * 360), -24000, 2000000, 0, 180);
            telemetry.addData("angle", -1 * (((double) Worm.getCurrentPosition() / 28) * 19.2) * 360);
            //22.2816920329 is how many counts per inch extension maybe
            //1050 counts per inch.
            extensionInches= Math.abs( ArmMotor.getCurrentPosition());
            if (extensionInches<0){
                extensionInches=0;

            }
            else {
                extensionInches = -1*((double) ArmMotor.getCurrentPosition() /1053);//FIX
            }
            telemetry.addData("extension", extensionInches);

            wristAngle = mapRange(ArmServo.getPosition(), .3, 1, 0, 180);

            double max;
            double max1;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.right_stick_y;
            double axial2 = gamepad1.left_stick_y;
            // Note: pushing stick forward gives negative value
            double lateral = gamepad1.right_trigger;
            double lateral1 = gamepad1.left_trigger;
            double lateral2 = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                yaw = -.14;
            }
            if (gamepad1.dpad_down) {
                yaw = .14;
            }
            double worm = gamepad2.left_stick_y;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = ((axial2 + axial) + ((-1 * lateral) + lateral1 + lateral2) + yaw);
            double rightFrontPower = ((axial2 + axial) - ((-1 * lateral) + lateral1 + lateral2) - yaw);
            double leftBackPower = ((axial2 + axial) - ((-1 * lateral) + lateral1 + lateral2) + yaw);
            double rightBackPower = ((axial2 + axial) + ((-1 * lateral) + lateral1 + lateral2) - yaw);
            double wormPower = worm;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max1 = Math.max(Math.abs(worm), Math.abs(rightFrontPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;

            }
            if (max1 > 1.0) {
                worm /= max1;
            }
            if (!gamepad2.y) {
                if (gamepad2.left_bumper) {
                    ArmMotor.setPower(1);
                    //sleep(50);
                }
            }
            if (gamepad2.right_bumper) {
                ArmMotor.setPower(-1);
                //sleep(50);

            }
            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                ArmMotor.setPower(0);
            }
            if (gamepad2.right_stick_y < 0 && ArmServo.getPosition() + .006 <= 1) {
                ArmServo.setPosition(ArmServo.getPosition() + .006);

                sleep(1);
            }
            if (gamepad2.right_stick_y > 0 && ArmServo.getPosition() - .006 >= .3) {
                ArmServo.setPosition(ArmServo.getPosition() - .006);
                sleep(1);
            }
            if (gamepad2.x) {

                if (ClawServoL.getPosition() != -1 || ClawServoR.getPosition() != 1) {
                    ClawServoR.setPosition(1);
                    ClawServoL.setPosition(-1);

                }
                sleep(350);
                if (ClawServoL.getPosition() == -1 || ClawServoR.getPosition() == 1) {
                    if (ArmServo.getPosition() != 1) {
                        ArmServo.setPosition(1);
                    }

                }

                ArmMotor.setPower(0);

            }
            if (gamepad2.a) {

                ArmMotor.setPower(0);
                if (ClawServoL.getPosition() != .9) {
                    ClawServoL.setPosition(.9);
                }
                if (ClawServoR.getPosition() != .4) {
                    ClawServoR.setPosition(.4);
                }
                if (ArmServo.getPosition() != .575) {
                    ArmServo.setPosition(.575);
                }
            }
            if (gamepad2.b && !clawR) {
                if (ClawServoR.getPosition() == .4) ClawServoR.setPosition(.9);
                else ClawServoR.setPosition(.4);
                clawR = true;
            } else if (!gamepad2.b) clawR = false;
            if (!gamepad2.left_bumper) {
                if ((gamepad2.y) && !clawL) {
                    if (ClawServoL.getPosition() == .2) ClawServoL.setPosition(.9);
                    else ClawServoL.setPosition(.2);
                    clawL = true;
                } else if (!gamepad2.y) clawL = false;
            }
            if (gamepad2.dpad_up) {
                checkOne = true;
            }
            if (gamepad2.left_trigger > 0) {
                checkTwo = true;
            }
            if(gamepad2.left_stick_button && !angleToggle) {
                controlTime.reset();
                controlTime.startTime();
                if(WristAngleMove) WristAngleMove= false;
                else WristAngleMove = true;

                angleToggle = true;
            } else if(!gamepad2.left_stick_button) angleToggle = false;
           if (controlTime.seconds()>8){
               WristAngleMove=false;
               angleToggle=false;
           }



            if (WristAngleMove) {
                telemetry.addData("Status", "Wrist Move Disabled");
                wristAngle = armAngle+25;
                ArmServo.setPosition(mapRange(wristAngle, 0, 120, 1, .3));

            }



            if (checkOne && checkTwo) {
                LaunchServo.setPosition(0);
            }
//            if (armAngle<5&&gamepad2.left_stick_y>0){
//                wormPower=0;
//            }
            telemetry.addData("Status",extensionInches);
            telemetry.update();
            // Send calculated power to wheels
            FrontLeft.setPower(speedLimit * leftFrontPower);
            FrontRight.setPower(speedLimit * rightFrontPower);
            BackLeft.setPower(speedLimit * leftBackPower);
            BackRight.setPower(speedLimit * rightBackPower);
            Worm.setPower(wormPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);

        }

    }

}



