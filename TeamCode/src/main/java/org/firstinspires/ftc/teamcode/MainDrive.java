package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main Drive", group="Linear OpMode")
public class MainDrive extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private final ElapsedTime runtime = new ElapsedTime();
        private DcMotor BackRight = null;
        private DcMotor BackLeft = null;
        private DcMotor FrontRight = null;
        private DcMotor FrontLeft = null;
        private DcMotor ArmMotor = null;
        private DcMotor Worm = null;
        Servo ArmServo;
        Servo ClawServoR;
        Servo ClawServoL;
        Servo LaunchServo;
    boolean checkOne = false;
    boolean checkTwo = false;
    boolean clawR = false;
    boolean clawL = false;
    boolean checkThree = false;
    boolean Reload = false;

        @Override
        public void runOpMode() {


            FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
            BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
            FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            BackRight = hardwareMap.get(DcMotor.class, "BackRight");
            ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
            Worm = hardwareMap.get(DcMotor.class, "Worm");
            ArmServo = hardwareMap.get(Servo.class, "ArmServo");
            ClawServoR = hardwareMap.get(Servo.class, "ClawServoR");
            ClawServoL = hardwareMap.get(Servo.class, "ClawServoL");
            //LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = gamepad1.right_stick_y;
                double axial2   = gamepad1.left_stick_y;// Note: pushing stick forward gives negative value
                double lateral =  gamepad1.right_trigger;
                double lateral1 =  gamepad1.left_trigger;
                double lateral2 =  -gamepad1.left_stick_x;
                double yaw     =  -gamepad1.right_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  =.75* ((axial2+axial) + ((-1*lateral)+lateral1+lateral2) + yaw);
                double rightFrontPower =.75* ((axial2+axial) - ((-1*lateral)+lateral1+lateral2) - yaw);
                double leftBackPower   =.75* ((axial2+axial) - ((-1*lateral)+lateral1+lateral2) + yaw);
                double rightBackPower  =.75*((axial2+axial) + ((-1*lateral)+lateral1+lateral2) - yaw);

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                if (gamepad2.left_trigger>0){
                    ArmMotor.setPower(1);
                    sleep(50);
                    ArmMotor.setPower(0);
                }
                if (gamepad2.right_trigger>0){
                    ArmMotor.setPower(-1);
                    sleep(50);
                    ArmMotor.setPower(0);
                }
                if (gamepad2.left_stick_y<0){
                    Worm.setPower(-1);
                    sleep(50);
                    Worm.setPower(0);
                }
                if (gamepad2.left_stick_y>0){
                    Worm.setPower(1);
                    sleep(50);
                    Worm.setPower(0);
                }
                if (gamepad2.right_stick_y>0&&ArmServo.getPosition()+.2<1){
                    ArmServo.setPosition(ArmServo.getPosition()+.02);

                    sleep(50);
                }
                if (gamepad2.right_stick_y<0&&ArmServo.getPosition()-.2>0){
                    ArmServo.setPosition(ArmServo.getPosition()-.02);
                    sleep(50);
                }
                if (gamepad1.dpad_up){
                    checkOne=true;
                }
                if((gamepad2.b || gamepad1.b)&& !clawR) {
                    if(ClawServoR.getPosition() == .5) ClawServoR.setPosition(1);
                    else ClawServoR.setPosition(.5);
                    clawR = true;
                } else if(!gamepad2.b||!gamepad1.b) clawR = false;
                if((gamepad2.y ||gamepad1.y)&& !clawL) {
                    if(ClawServoL.getPosition() == .4) ClawServoL.setPosition(-1);
                    else ClawServoL.setPosition(.4);
                    clawL = true;
                } else if(!gamepad2.y||gamepad1.y) clawL = false;

                if (gamepad1.b){
                    checkThree=true;
                }
                if (gamepad1.x){
                    Reload=true;
                }
                if (gamepad1.left_bumper){
                    checkTwo=true;
                }
                if(checkOne&&checkTwo&&checkThree){
                    LaunchServo.setPosition(0);
                }
                if(checkOne&&checkTwo&&Reload){
                    LaunchServo.setPosition(1);
                }

                // Send calculated power to wheels
                FrontLeft.setPower(.75*leftFrontPower);
                FrontRight.setPower(.75*rightFrontPower);
                BackLeft.setPower(.75*leftBackPower);
                BackRight.setPower(.75*rightBackPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());

            }
        }}



