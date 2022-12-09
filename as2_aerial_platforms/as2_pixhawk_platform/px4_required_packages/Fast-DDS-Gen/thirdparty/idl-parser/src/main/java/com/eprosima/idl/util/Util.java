// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package com.eprosima.idl.util;

import java.io.File;

public class Util
{
    public static String getIDLFileNameOnly(String idlFilename)
    {
        int index = -1;
        String auxString = idlFilename, returnedValue = null;

        index = idlFilename.lastIndexOf(File.separator);

        if(index == -1)
        {
            index = idlFilename.lastIndexOf('/');
        }

        if(index != -1)
        {
            auxString = idlFilename.substring(index + 1);
        }

        // Remove '.idl'
        returnedValue = auxString.substring(0, auxString.length() - 4);

        return returnedValue;
    }

    public static String getIDLFileOnly(String idlFileURL)
    {
        int index = -1;
        String returnedValue = null;

        index = idlFileURL.lastIndexOf(File.separator);

        if(index == -1)
            index = idlFileURL.lastIndexOf('/');

        if(index != -1)
            returnedValue = idlFileURL.substring(index + 1);
        else
            returnedValue = idlFileURL;

        return returnedValue;
    }

    public static String getIDLFileDirectoryOnly(String idlFileURL)
    {
        int index = -1;
        String returnedValue = null;

        index = idlFileURL.lastIndexOf(File.separator);

        if(index == -1)
            index = idlFileURL.lastIndexOf('/');

        if(index != -1)
            returnedValue = idlFileURL.substring(0, index + 1);

        return returnedValue;
    }

    public static String stringTrimAll(String str)
    {
        String trimstr = str.replaceAll("\\s+", "").toUpperCase();
        return trimstr;
    }
}
