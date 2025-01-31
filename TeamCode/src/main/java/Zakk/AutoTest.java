package Zakk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Interfaces.Drive;



@Autonomous(name = "AutoBlue", group = "competition")
public class AutoTest extends LinearOpMode {

    private Drive _Drive;
    public static double DriveDistance = 24;
    public static double Power = .4;
    public static double StopDistance = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        telemetry.addData("=D", "Ready to start!");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            // Drive Forward for 24 inches or until 10cm from something
            /*
            [Blue/Red] [A=Crowd Side/B=Stage Side] [1=First Tile/2=Second Tile]
            Blue A1 = DD = 1.75 / P = .18 | Turn 90D Left | DD = 106 / P = .25
            Blue A2 = DD = 25.5 / P = .18 | Turn 90D Left | DD = 106 / P = .25

             */
            _Drive.Straight(Drive.Direction.FORWARD, DriveDistance, Power, StopDistance);
            sleep(500);
            _Drive.StrafeRight(6000, .4);
            sleep(500);
//            _Drive.Intake(IDrive.Direction.FORWARD,4000,.35);
//            sleep(1000);
            //           _Drive.Straight(IDrive.Direction.BACKWARD,6,Power,StopDistance);

            // break will exit the loop for us
            break;
        }
    }
}