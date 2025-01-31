package Zakk;

import android.hardware.Sensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Teleop", group = "Competition")
public class HopeThisWorks extends OpMode {
    /*
     * Declare Hardware
     */

    // IMU
    private IMU imu;

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    // Arm
    private DcMotor Arm;
    private DcMotor RotateArm;
    CRServo Crotate;
    CRServo Claw;

    // SlowMode Drive
    private boolean slowModeDriveOn = true;
    private boolean buttonSlowDriveIsPressed = false;
    private final double SLOW_DRIVE = 0.4;
    private final double FAST_DRIVE = 1.0; // 0.9;
    private double percentToSlowDrive = SLOW_DRIVE;

    // SineDrive
    private boolean sineDriveOn = true;
    private boolean buttonSineIsPressed = false;
    private double modifyBySine = Math.sin(Math.PI / 4);

    // Encoder Constants
    private static final double TICKS_PER_REVOLUTION = 1120; // Example for an AndyMark NeveRest motor
    private static final double INCHES_PER_REVOLUTION = 4 * Math.PI; // Assuming a 4-inch wheel
    private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REVOLUTION;

    @Override
    public void init() {
        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");
        Arm = hardwareMap.dcMotor.get("Arm");
        RotateArm = hardwareMap.dcMotor.get("RotateArm");
        Crotate = hardwareMap.crservo.get("Crotate");
        Claw = hardwareMap.crservo.get("Claw");

        resetEncoders();

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        RotateArm.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();
    }

    @Override
    public void loop() {
        /*
         * Gamepad Controls
         */

        // Gamepad 1
        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;
        double oneRightStickYPower = gamepad1.right_stick_y;
        boolean oneButtonA = gamepad1.a;
        boolean oneButtonB = gamepad1.b;
        boolean oneButtonX = gamepad1.x;
        boolean oneButtonY = gamepad1.y;
        boolean onePadUp = gamepad1.dpad_up;
        boolean onePadDown = gamepad1.dpad_down;
        boolean onePadLeft = gamepad1.dpad_left;
        boolean onePadRight = gamepad1.dpad_right;
        float oneTriggerLeft = gamepad1.left_trigger;
        float oneTriggerRight = gamepad1.right_trigger;
        boolean oneBumperLeft = gamepad1.left_bumper;
        boolean oneBumperRight = gamepad1.right_bumper;
        boolean oneBack = gamepad1.back;
        boolean oneStart = gamepad1.start;

        // Gamepad 2
        double twoLeftStickYPower = gamepad2.left_stick_y;
        double twoLeftStickXPower = gamepad2.left_stick_x;
        double twoRightStickXPower = gamepad2.right_stick_x;
        double twoRightStickYPower = gamepad2.right_stick_y;
        boolean twoButtonA = gamepad2.a;
        boolean twoButtonB = gamepad2.b;

        /*
         * Reset Encoders
         */
        if (twoButtonA) {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Arm Encoder", "Reset");
        }

        if (twoButtonB) {
            RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("RotateArm Encoder", "Reset");
        }

        /*
         * Do Stuff Here!
         */

        ToggleSlowModeDrive(oneButtonA);
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);
        ArmSystemControl(twoLeftStickYPower, twoRightStickXPower, twoRightStickYPower);

        //controlArmWithEncoders(twoLeftStickYPower, twoLeftStickXPower);

        telemetry.update();
    }

    /*
     * Methods
     */

    private void ProMotorControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        double powerLeftY = left_stick_y;   // DRIVE : Backward -1 <---> 1 Forward
        double powerLeftX = -left_stick_x * -1; // STRAFE:     Left -1 <---> 1 Right
        double powerRightX = right_stick_x; // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerLeftX, powerLeftY);
        double robotAngle = Math.atan2(powerLeftY, powerLeftX) - Math.PI / 4;
        double leftX = powerRightX;
        final double v1 = r * Math.cos(robotAngle) / modifyBySine + leftX;
        final double v2 = r * Math.sin(robotAngle) / modifyBySine - leftX;
        final double v3 = r * Math.sin(robotAngle) / modifyBySine + leftX;
        final double v4 = r * Math.cos(robotAngle) / modifyBySine - leftX;

        WheelFrontLeft.setPower(v1 * percentToSlowDrive);
        WheelFrontRight.setPower(v2 * percentToSlowDrive);
        WheelBackLeft.setPower(v3 * percentToSlowDrive);
        WheelBackRight.setPower(v4 * percentToSlowDrive);

        telemetry.addData("Wheel Front Left", v1 * percentToSlowDrive);
        telemetry.addData("Wheel Front Right", v2 * percentToSlowDrive);
        telemetry.addData("Wheel Back Left", v3 * percentToSlowDrive);
        telemetry.addData("Wheel Back Right", v4 * percentToSlowDrive);
    }

//    private void controlArmWithEncoders(double armPower, double rotatePower) {
//        int armTicks = Arm.getCurrentPosition();
//        int rotateTicks = RotateArm.getCurrentPosition();
//
//        double armInches = armTicks / TICKS_PER_INCH;
//        double rotateInches = rotateTicks / TICKS_PER_INCH;
//
//        if (rotateInches <= 0.5 && armInches >= 38) {
//            Arm.setPower(0); // Stop the Arm motor
//        } else {
//            Arm.setPower(armPower); // Allow movement otherwise
//        }
//
//        if (rotateInches > 2) {
//            Arm.setPower(armPower); // Allow unrestricted movement
//        }
//
//        RotateArm.setPower(rotatePower);
//
//        telemetry.addData("Arm Inches", armInches);
//        telemetry.addData("Rotate Inches", rotateInches);
//        telemetry.addData("Arm Power", armPower);
//        telemetry.addData("Rotate Power", rotatePower);
//    }

    private void ToggleSlowModeDrive(boolean button) {
        if (button && !buttonSlowDriveIsPressed) {
            buttonSlowDriveIsPressed = true;
            slowModeDriveOn = !slowModeDriveOn;
        }
        if (!button) {
            buttonSlowDriveIsPressed = false;
        }

        if (slowModeDriveOn) {
            percentToSlowDrive = SLOW_DRIVE;
            telemetry.addData("Drive Mode", "Slow: " + percentToSlowDrive + "% Power");
        } else {
            percentToSlowDrive = FAST_DRIVE;
            telemetry.addData("Drive Mode", "Fast: " + percentToSlowDrive + "% Power");
        }
    }

    private void ArmSystemControl(double ArmPower,double CrotatePower,double ClawPower){
        Arm.setPower(ArmPower);
        Crotate.setPower(CrotatePower);
        Claw.setPower(ClawPower);
    }

    private void resetEncoders() {
        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
