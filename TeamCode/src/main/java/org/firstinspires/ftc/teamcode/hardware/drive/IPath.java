package org.firstinspires.ftc.teamcode.hardware.drive;

import androidx.annotation.NonNull;

public interface IPath <DataType> {
    float getDuration();
    PathPoint<DataType> getPoint(float elapsed);

    static <Ty> PathPoint<Ty> first(@NonNull IPath<Ty> p) {
        return p.getPoint(0f);
    }

    static <Ty> PathPoint<Ty> last(@NonNull IPath<Ty> p) {
        return p.getPoint(p.getDuration());
    }
}
