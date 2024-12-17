package rrpathgen.trajectory;

import rrpathgen.RRPathGenMain;
import rrpathgen.data.Marker;
import rrpathgen.data.Node;

import java.util.Comparator;
import java.util.List;

import static rrpathgen.RRPathGenMain.getCurrentManager;

public class NewRRTrajectory extends OldRRTrajectory{


    @Override
    public Node.Type[] getValidNodeTypes() {
        return new Node.Type[]{
                Node.Type.splineTo,
                Node.Type.splineToSplineHeading,
                Node.Type.splineToLinearHeading,
                Node.Type.splineToConstantHeading,
//                Node.Type.lineTo,
//                Node.Type.lineToSplineHeading,
//                Node.Type.lineToLinearHeading,
//                Node.Type.lineToConstantHeading,
        };
    }

    @Override
    public Node.Type[] getValidMarkerTypes() {
        return new Node.Type[]{
//                Node.Type.addTemporalMarker
        };
    }


    @Override
    public String constructExportString() {
        Node node = getCurrentManager().getNodes().get(0);
        double x = RRPathGenMain.toInches(node.x);
        double y = RRPathGenMain.toInches(node.y);

        StringBuilder sb = new StringBuilder();
        if(RRPathGenMain.exportPanel.addDataType) sb.append("TrajectorySequence ");
        sb.append(String.format("%s = drive.trajectorySequenceBuilder(new Pose2d(%.2f, %.2f, Math.toRadians(%.2f)))%n",getCurrentManager().name, x, -y, (node.robotHeading +90)));
        //sort the markers
        List<Marker> markers = getCurrentManager().getMarkers();
        markers.sort(Comparator.comparingDouble(n -> n.displacement));
        for (Marker marker : markers) {
            sb.append(String.format(".UNSTABLE_addTemporalMarkerOffset(%.2f,() -> {%s})%n", marker.displacement, marker.code));
        }
        boolean prev = false;
        for (int i = 0; i < getCurrentManager().size(); i++) {
            node = getCurrentManager().get(i);
            if(node.equals(getCurrentManager().getNodes().get(0))) {
                if(node.reversed != prev){
                    sb.append(String.format(".setReversed(%s)%n", node.reversed));
                    prev = node.reversed;
                }
                continue;
            }
            x = RRPathGenMain.toInches(node.x);
            y = RRPathGenMain.toInches(node.y);


            switch (node.getType()){
                case splineTo:
                    sb.append(String.format(".splineTo(new Vector2d(%.2f, %.2f), Math.toRadians(%.2f))%n", x, -y, (node.splineHeading +90)));
                    break;
                case splineToSplineHeading:
                    sb.append(String.format(".splineToSplineHeading(new Pose2d(%.2f, %.2f, Math.toRadians(%.2f)), Math.toRadians(%.2f))%n", x, -y, (node.robotHeading +90), (node.splineHeading +90)));
                    break;
                case splineToLinearHeading:
                    sb.append(String.format(".splineToLinearHeading(new Pose2d(%.2f, %.2f, Math.toRadians(%.2f)), Math.toRadians(%.2f))%n", x, -y, (node.robotHeading +90), (node.splineHeading +90)));
                    break;
                case splineToConstantHeading:
                    sb.append(String.format(".splineToConstantHeading(new Vector2d(%.2f, %.2f), Math.toRadians(%.2f))%n", x, -y, (node.splineHeading +90)));
                    break;
                case lineTo:
                    sb.append(String.format(".lineTo(new Vector2d(%.2f, %.2f))%n", x, -y));
                    break;
                case lineToSplineHeading:
                    sb.append(String.format(".lineToSplineHeading(new Pose2d(%.2f, %.2f, Math.toRadians(%.2f)))%n", x, -y, (node.robotHeading +90)));
                    break;
                case lineToLinearHeading:
                    sb.append(String.format(".lineToLinearHeading(new Pose2d(%.2f, %.2f, Math.toRadians(%.2f)))%n", x, -y, (node.robotHeading +90)));
                    break;
                case lineToConstantHeading:
                    sb.append(String.format(".lineToConstantHeading(new Vector2d(%.2f, %.2f))%n", x, -y, (node.splineHeading +90)));
                    break;
                case addTemporalMarker:
                    break;
                default:
                    sb.append("couldn't find type");
                    break;
            }
            if(node.reversed != prev){
                sb.append(String.format(".setReversed(%s)%n", node.reversed));
                prev = node.reversed;
            }
        }
        sb.append(String.format(".build();%n"));
        if(RRPathGenMain.exportPanel.addPoseEstimate) sb.append(String.format("drive.setPoseEstimate(%s.start());", getCurrentManager().name));
        return sb.toString();
    }
}
