package com.eprosima.integration;

import java.io.File;
import java.util.List;

public class Test
{

    private IDL idl;
    private String outputPath;
    private boolean errorOutputOnly;

    public Test(IDL idl, String outputPath, boolean errorOutputOnly)
    {
        this.idl = idl;
        this.outputPath = outputPath + "/" + idl.toString().toLowerCase();
        this.errorOutputOnly = errorOutputOnly;
    }
;
    public IDL getIDL()
    {
        return idl;
    }

    public boolean prepare()
    {
        File outputPathFolder = new File(outputPath + "/build");
        boolean prepared = false;
        if(outputPathFolder.exists() && outputPathFolder.isDirectory())
        {
            prepared = true;
        }
        else
        {
            prepared= outputPathFolder.mkdirs();
        }

        if(prepared)
        {
            System.out.println("Done!");
        }
        return prepared;
    }

    public boolean generate(String generatorName, String inputPath, String exampleArch, boolean testFlag)
    {
        String program = "java -jar " + generatorName + ".jar";
        String flags = " -replace -example" + " " + exampleArch + (testFlag ? " -test" : "");
        String output = " -d " + outputPath;
        String idlPaths = "";
        for(IDL aux = idl; aux != null; aux = aux.getRequired())
        {
            idlPaths += " " + inputPath + "/" + aux.toString().toLowerCase() + ".idl";
        }

        String command = program + flags + output + idlPaths;
        return Command.execute(command, null, errorOutputOnly);
    }

    public boolean generate(String generatorName, String inputPath, boolean testFlag)
    {
        String program = "java -jar " + generatorName + ".jar";
        String flags = " -replace -example" + (testFlag ? " -test" : "");
        String output = " -d " + outputPath;
        String idlPaths = "";
        for(IDL aux = idl; aux != null; aux = aux.getRequired())
        {
            idlPaths += " " + inputPath + "/" + aux.toString().toLowerCase() + ".idl";
        }

        String command = program + flags + output + idlPaths;
        return Command.execute(command, null, errorOutputOnly);
    }

    public boolean configure(List<String> cMakeArguments)
    {
        String arguments = cMakeArguments.toString().replaceFirst("\\[", " ").replaceAll(",|\\]", "");
        return Command.execute("cmake .." + arguments, outputPath + "/build", errorOutputOnly);
    }

    public boolean compile()
    {
        return Command.execute("make", outputPath + "/build", errorOutputOnly);
    }

    public boolean run()
    {
        boolean exitStatus = Command.execute("./" + idl.toString().toLowerCase() + "SerializationTest", outputPath + "/build", errorOutputOnly);
        if(!exitStatus)
        {
            return false;
        }

        return true;
    }
}
