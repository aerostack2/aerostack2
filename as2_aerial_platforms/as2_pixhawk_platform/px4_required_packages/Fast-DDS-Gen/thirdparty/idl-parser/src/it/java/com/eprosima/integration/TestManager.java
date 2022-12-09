package com.eprosima.integration;

import java.util.ArrayList;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

public class TestManager
{
    public enum TestLevel
    {
        PREPARE(0),
        GENERATE(1),
        CONFIGURE(2),
        COMPILE(3),
        RUN(4);

        private int value;

        TestLevel(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return value;
        }
    }

    public static final IDL[] ALL_IDLS = IDL.values();

    private TestLevel level;
    private ArrayList<IDL> idls;
    private String generatorName;
    private String inputPath;
    private String outputPath;
    private String exampleArch;
    private List<String> cMakeArgs;
    private boolean errorOutputOnly;

    public TestManager(TestLevel level, String generatorName, String inputPath, String outputPath)
    {
        this.level = level;
        this.idls = new ArrayList<IDL>(Arrays.asList(ALL_IDLS));
        this.generatorName = generatorName;
        this.inputPath = inputPath;
        this.outputPath = outputPath;
        this.exampleArch = null;
        this.cMakeArgs = new ArrayList<String>();
        this.errorOutputOnly = true;
    }

    public TestManager(TestLevel level, String generatorName, String inputPath, String outputPath, String exampleArch)
    {
        this.level = level;
        this.idls = new ArrayList<IDL>(Arrays.asList(ALL_IDLS));
        this.generatorName = generatorName;
        this.inputPath = inputPath;
        this.outputPath = outputPath;
        this.exampleArch = exampleArch;
        this.cMakeArgs = new ArrayList<String>();
        this.errorOutputOnly = true;
    }

    public void showOutputOnlyAtErrors(boolean value)
    {
        errorOutputOnly = value;
    }

    public void addCMakeArguments(String... args)
    {
        cMakeArgs.addAll(Arrays.asList(args));
    }

    public void addTests(IDL... args)
    {
        for(IDL idl: Arrays.asList(args))
        {
            idls.add(idl);
        }
    }

    public void removeTests(IDL... args)
    {
        Iterator<IDL> it = idls.iterator();
        while (it.hasNext())
        {
            if(Arrays.asList(args).contains(it.next()))
            {
                it.remove();
            }
        }
    }

    public boolean runTests()
    {
        for(IDL idl: idls)
        {
            Test test = new Test(idl, outputPath, errorOutputOnly);
            if(!run(test))
            {
                return false;
            }
        }

        return true;
    }

    private boolean prepare(Test test)
    {
        printHeader(test.getIDL(), TestLevel.PREPARE);
        return printlnStatus(test.prepare());
    }

    public boolean generate(Test test)
    {
        boolean precondition = prepare(test);
        if(precondition && level.getValue() >= TestLevel.GENERATE.getValue())
        {
            printHeader(test.getIDL(), TestLevel.GENERATE);

            if (exampleArch == null)
            {
                return printlnStatus(
                    test.generate(generatorName, inputPath, level == TestLevel.RUN));
            }
            else
            {
                return printlnStatus(
                    test.generate(generatorName, inputPath, exampleArch, level == TestLevel.RUN));
            }
        }

        return precondition;
    }

    public boolean configure(Test test)
    {
        boolean precondition = generate(test);
        if(precondition && level.getValue() >= TestLevel.CONFIGURE.getValue())
        {
            printHeader(test.getIDL(), TestLevel.CONFIGURE);
            return printlnStatus(test.configure(cMakeArgs));
        }

        return precondition;
    }

    public boolean compile(Test test)
    {
        boolean precondition = configure(test);
        if(precondition && level.getValue() >= TestLevel.COMPILE.getValue())
        {
            printHeader(test.getIDL(), TestLevel.COMPILE);
            return printlnStatus(test.compile());
        }

        return precondition;
    }

    private boolean run(Test test)
    {
        boolean precondition = compile(test);
        if(precondition && level.getValue() >= TestLevel.RUN.getValue())
        {
            printHeader(test.getIDL(), TestLevel.RUN);

            return printlnStatus(test.run());
        }

        return precondition;
    }

    private void printHeader(IDL idl, TestLevel level)
    {
        System.out.println("\n\n>>> " + idl.toString() + " TEST: " + level.toString() + "...");
    }

    private boolean printlnStatus(boolean status)
    {
        System.out.println("    RESULT: " + (status ? "OK!" : "ERROR"));
        return status;
    }
}
