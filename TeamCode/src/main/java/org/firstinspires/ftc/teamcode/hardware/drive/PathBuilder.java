package org.firstinspires.ftc.teamcode.hardware.drive;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Function;

public class PathBuilder <DataType> implements IPath<DataType> {
    public static class PathData <DataType> {
        public PathPoint<DataType> start, end;

        public PathData(PathPoint<DataType> _start, PathPoint<DataType> _end) {
            start = _start;
            end = _end;
        }
    }

    ArrayList<IPath<DataType>> path;
    float duration;
    PathPoint<DataType> start, end;

    ArrayList<Float> cumulativeEndTimes;

    public PathBuilder(PathPoint<DataType> _start) {
        path = new ArrayList<>();
        duration = 0f;

        start = _start;
        end = _start;

        cumulativeEndTimes = new ArrayList<>();
    }

    void addNextEndTime(float nextDuration) {
        if (cumulativeEndTimes.isEmpty()) {
            cumulativeEndTimes.add(nextDuration);
        }
        else {
            cumulativeEndTimes.add(cumulativeEndTimes.get(cumulativeEndTimes.size() - 1) + nextDuration);
        }
    }

    public void addPath(@NonNull Function<PathData<DataType>, IPath<DataType>> pathBuilder) {
        IPath<DataType> newPath = pathBuilder.apply(new PathData<>(start, end));
        path.add(newPath);
        end = IPath.last(newPath);
        duration += newPath.getDuration();
        addNextEndTime(newPath.getDuration());
    }

    @Override
    public float getDuration() {
        return duration;
    }

    @Override
    public PathPoint<DataType> getPoint(float elapsed) {
        if (path.isEmpty() || elapsed <= 0) {
            return start;
        }

        if (elapsed >= duration) {
            return end;
        }

        int index = Collections.binarySearch(cumulativeEndTimes, elapsed);

        if (index < 0) {
            index = -index - 1; // java lib is so stupid -_- lmao
        }

        IPath<DataType> subPath = path.get(index);
        float localTime = elapsed - (index == 0 ? 0f : cumulativeEndTimes.get(index - 1));

        return subPath.getPoint(localTime);
    }
}
