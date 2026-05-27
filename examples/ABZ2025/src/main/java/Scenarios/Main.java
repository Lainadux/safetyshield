package Scenarios;

public class Main {
    public static void main(String[] args) throws Exception {
        //RandomStarkRun.main(args);
        String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs4/crash_initial_20260527_204759_797_45.json";
        RecoverStarkRun.main(new String[]{filePath});
        //GenerateInitialLogsRun.main(args);
    }
}
