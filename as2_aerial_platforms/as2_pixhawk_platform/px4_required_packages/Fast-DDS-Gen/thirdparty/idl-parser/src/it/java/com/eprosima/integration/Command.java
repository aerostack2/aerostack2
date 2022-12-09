package com.eprosima.integration;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public class Command
{
    private Command() {}

    public static boolean execute(String command, String from, boolean errorOutputOnly)
    {
        try
        {
            System.out.println("Executing command: " + command);
            List<String> arguments = Arrays.asList(command.split(" "));
            ProcessBuilder processBuilder = new ProcessBuilder(arguments);
            processBuilder.directory(from != null ? new File(from) : null);
            processBuilder.redirectErrorStream(true);

            Process process = processBuilder.start();

            ArrayList<String> output = new ArrayList<String>();
            {
                BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
                String line = "";
                while ((line = reader.readLine()) != null)
                {
                    output.add(line);
                }
            }
            process.waitFor();

            boolean status = process.exitValue() == 0;

            if (!status || !errorOutputOnly)
            {
                for (String line : output)
                {
                    System.out.println(line);
                }
            }

            return status;
        }
        catch (IOException e)
        {
            System.err.println("Error executing: " + command);
            e.printStackTrace();
            return false;
        }
        catch(InterruptedException e)
        {
            System.err.println("Error Interrupted execution: " + command);
            e.printStackTrace();
            return false;
        }
    }
}
