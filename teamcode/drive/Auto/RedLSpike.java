package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.armsNStuff;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "RedLeftSpike", group = "RED")
public class RedLSpike extends LinearOpMode {
   // armsNStuff arm;

    OpenCvCamera webcam;
    GamePropLeft.gamePropPosition propPosition = GamePropLeft.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {

      //  arm=new armsNStuff(hardwareMap);

        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d wallPose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        Pose2d firstparkingPose = new Pose2d(0, 0, Math.toRadians(0.0));
        Pose2d leftPose = new Pose2d(0, 0, 0);
        Pose2d parkingPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left", "Left");
               /* wallPose1 = new Pose2d(22,-6.75, Math.toRadians(-90));
                wallPose2 = new Pose2d(48.5,14, Math.toRadians(-90));
                nitPose = new Pose2d(53,-82, Math.toRadians(-90));
                innitPose = new Pose2d(38,-82, Math.toRadians(-90));*/
                leftPose = new Pose2d(19,9, Math.toRadians(0));
                purplePixelPose = new Pose2d(19.5,9, Math.toRadians(0));

                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
               /* wallPose1 = new Pose2d(20,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(48,15, Math.toRadians(-90));
                nitPose = new Pose2d(56,-82, Math.toRadians(-90));
                innitPose = new Pose2d(34,-82, Math.toRadians(-90));*/
                purplePixelPose = new Pose2d(27, -3, Math.toRadians(0.0));
                leftPose = new Pose2d(25, -3, Math.toRadians(0.0));
                break;
            case RIGHT:
               /* wallPose1 = new Pose2d(20,9, Math.toRadians(-90));
                wallPose2 = new Pose2d(48,15, Math.toRadians(-90));
                nitPose = new Pose2d(56,-82, Math.toRadians(-90));
                innitPose = new Pose2d(28.3,-82, Math.toRadians(-90));*/
                purplePixelPose = new Pose2d(29, -6, Math.toRadians(-90.0));
                leftPose = new Pose2d(29, -1, Math.toRadians(-90.0));
        }
        /*Pose2d firstparkingPose = new Pose2d(24, -25, 0); //UPDATE
        Pose2d parkingPose = new Pose2d(24, -65, 0); //UPDATE
        Pose2d pickupPose = new Pose2d(24, -25, 0); //UPDATE*/


        telemetry.update();
        sleep(2000);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .lineToLinearHeading(leftPose)
                .waitSeconds(1)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                                .back(8)
               /* .strafeLeft(10)
                //add in movement to grab another pixel
                .lineToLinearHeading(wallPose)
                .lineToLinearHeading(firstparkingPose)
                .waitSeconds(0.25)
                .forward(7)
                .lineToLinearHeading(parkingPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {arm.backdrop(1);})
                //Parking in the back
                .waitSeconds(2)*/
                .build());
    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropLeft());
    }
	
	public GamePropLeft.gamePropPosition getPropPosition() {
		return GamePropLeft.position; 
	}
	
}