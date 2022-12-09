package test.com.eprosima.fastrtps;

import org.junit.jupiter.api.Test;

import com.eprosima.integration.Command;
import com.eprosima.integration.IDL;
import com.eprosima.integration.TestManager;
import com.eprosima.integration.TestManager.TestLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.nio.file.Paths;
import java.nio.file.Files;

public class FastRTPSGenTest
{
    private static final String INPUT_PATH = "thirdparty/idl-parser/test/idls";
    private static final String OUTPUT_PATH = "build/test/integration";

    private static boolean isUnix()
    {
        String os = System.getProperty("os.name").toLowerCase();
        return os.contains("nix") || os.contains("nux") || os.contains("aix");
    }

    @Test
    public void runTests()
    {
        if (!isUnix())
        {
            System.out.println("WARNING: The tests are only available with an unix system");
            return;
        }

        //Configure idl tests
        TestManager tests = new TestManager(TestLevel.RUN, "share/fastrtpsgen/java/fastrtpsgen", INPUT_PATH,
                        OUTPUT_PATH + "/idls", "CMake");
        tests.removeTests(IDL.ARRAY_NESTED, IDL.SEQUENCE_NESTED);
        boolean testResult = tests.runTests();
        System.exit(testResult ? 0 : -1);
    }

}
