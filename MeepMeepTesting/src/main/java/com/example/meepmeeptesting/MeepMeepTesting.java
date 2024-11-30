package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        TrajectoryActionBuilder path1 = myBot.getDrive().actionBuilder(new Pose2d(9, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(113))
                .lineToYLinearHeading(-32.5, Math.toRadians(270))
                .waitSeconds(3)

                //move to the cage
                ;


        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-40, Math.toRadians(270))
                //move back from the cage

                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(35, Math.toRadians(0))

                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-20, Math.toRadians(0))

                .setTangent(Math.toRadians(45))
                .lineToYLinearHeading(-10,Math.toRadians(0))
                // first push point

                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-50, Math.toRadians(0))

                .setTangent(Math.toRadians(90))
                .lineToY(-10)

                .setTangent(Math.toRadians(0))
                .lineToX(57)

                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-50, Math.toRadians(0))

                .setTangent(Math.toRadians(180))
                .lineToX(36)

                .setTangent(Math.toRadians(227))
                .lineToX(29)
                ;

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .lineToX(40)
                ;

        TrajectoryActionBuilder path4 = path3.endTrajectory().fresh()
                .setTangent(Math.toRadians(148))
                .lineToYLinearHeading(-32.5, Math.toRadians(270))
                .waitSeconds(3)
                ;

        TrajectoryActionBuilder path5 = path4.endTrajectory().fresh()
                .setTangent(Math.toRadians(318))
                .lineToYLinearHeading(-57.5, Math.toRadians(0))
                ;

        TrajectoryActionBuilder path6 = path5.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .lineToX(40)
                ;

        TrajectoryActionBuilder path7 = path6.endTrajectory().fresh()
                .setTangent(Math.toRadians(146))
                .lineToYLinearHeading(-32.5, Math.toRadians(270))
                .waitSeconds(3)

                ;

        TrajectoryActionBuilder path8 = path7.endTrajectory().fresh()
                .setTangent(Math.toRadians(315))
                .lineToYLinearHeading(-57.5, Math.toRadians(0))
                ;

        TrajectoryActionBuilder path9 = path8.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .lineToX(40)
                ;

        TrajectoryActionBuilder path10 = path9.endTrajectory().fresh()
                .setTangent(Math.toRadians(148))
                .lineToYLinearHeading(-32.5, Math.toRadians(270))
                .waitSeconds(3)

                ;

        myBot.runAction(
                new SequentialAction(
                        path1.build(),
                        path2.build(),
                        path3.build(),
                        path4.build(),
                        path5.build(),
                        path6.build(),
                        path7.build(),
                        path8.build(),
                        path9.build(),
                        path10.build()
                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}