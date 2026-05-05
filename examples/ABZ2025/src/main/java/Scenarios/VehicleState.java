package Scenarios;

import com.google.gson.annotations.SerializedName;

public class VehicleState {
    public String role;          // "EGO" 或 "NPC"
    public double dist;          // 距离
    public int lane;             // 车道号
    public double speed;         // 当前速度

    // Python 里叫 target_speed，Java 规范推荐驼峰命名 targetSpeed
    // 用 @SerializedName 可以完美桥接这两个名字！
    @SerializedName("target_speed")
    public double targetSpeed;

    public double acceleration;  // 加速度

    // 为了方便打印查看，重写一下 toString 方法
    @Override
    public String toString() {
        return String.format("[%s] Dist: %.1f, Lane: %d, Spd: %.1f, Acc: %.2f, TargetSpd: %.1f",
                role, dist, lane, speed, acceleration, targetSpeed);
    }
}