//Name: TeleOpJacob
//Purpose: Version of the 24-25 that has most commented
//bits of code removed along with unused variables being removed
//also removed unused library's


//Import required library's
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@TeleOp(name = "TeleOpJacob", group = "Drive")
public class TeleOpDrive extends LinearOpMode {

    //Main Drive Motors
    //****************************************//
    //BR means "back right"				   	  //
    //BL means "back left"					  //
    //FR means "front right"				  //
    //FL means "front left"					  //
    //****************************************//

    /*
    Where the motors are plugged into
    Control Hub:
    BRMoto - Motor Port 0
    BLMoto - Motor Port 1
    FLMoto - Motor Port 2
    FRMoto - Motor Port 3

    Expansion Hub:
    string - Motor Port 1
    arm - Motor Port 2
    wrist - Servo Port 1
    claw - Servo Port 0
    */
    private final DcMotor FLMoto = hardwareMap.dcMotor.get("FLMoto");
    private final DcMotor FRMoto = hardwareMap.dcMotor.get("BRMoto");
    private final DcMotor BLMoto = hardwareMap.dcMotor.get("BLMoto");
    private final DcMotor BRMoto = hardwareMap.dcMotor.get("FRMoto");

    // Map Sensors
    private final NormalizedColorSensor color = hardwareMap.get(NormalizedColorSensor.class, "colorRange");

    double speed = 1.5;
    //variables for claw states

    //****************************************//
    // Map all robot hardware				  //
    //****************************************//


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


        //*****************************************//
        // Put initialization blocks here.			 //
        //*****************************************//

        //***************************************************//
        // Set direction of the main drive motors		     //
        //***************************************************//
        FLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        FRMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BRMoto.setDirection(DcMotorSimple.Direction.REVERSE);


        //Set the zero power behavior of the main drive motors to FLOAT
        FLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (color instanceof SwitchableLight) {
            ((SwitchableLight) color).enableLight(true);
        }

        //The waitForStart() function will wait for the start button will begin
        waitForStart();

        //******************************************//
        // Run code while op mode is active			//
        //******************************************//
        while (opModeIsActive()) {
            //**************************************************************//
            //Telemetry Code the stuff that appears on the right side of the//
            //driver hub													//
            //**************************************************************//
            telemetry.addData("Status", "opModeIsActive");

            //gamepad1

            //regular drive controls
            //sets the drive power to gamepad1 left_stick_y for the left drive train
            //and gamepad1 right_stick_y for the right side drive train then it sets it
            //all to be multiplied by the speed modifier
            FRMoto.setPower(gamepad1.right_stick_y * speed);
            FLMoto.setPower(gamepad1.left_stick_y * speed);
            BRMoto.setPower(gamepad1.right_stick_y * speed);
            BLMoto.setPower(gamepad1.left_stick_y * speed);

            //Strafe Right using gamepad1 right_trigger
            FRMoto.setPower(-gamepad1.right_trigger);
            FLMoto.setPower(-gamepad1.right_trigger);
            BRMoto.setPower(gamepad1.right_trigger);
            BLMoto.setPower(gamepad1.right_trigger);

            // Strafe Left using gamepad1 left_trigger
            FRMoto.setPower(gamepad1.left_trigger);
            FLMoto.setPower(gamepad1.left_trigger);
            BRMoto.setPower(-gamepad1.left_trigger);
            BLMoto.setPower(-gamepad1.left_trigger);
        }
    }
}
