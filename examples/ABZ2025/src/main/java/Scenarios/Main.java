package Scenarios;

public class Main {
    public static void main(String[] args) throws Exception {
        //RandomStarkRun.main(args);
        String filePath = "examples/ABZ2025/src/main/java/Scenarios/logs3/safe_initial_20260527_012623_186_2.json";
        RecoverStarkRun.main(new String[]{filePath});
        //GenerateInitialLogsRun.main(args);
    }
}
