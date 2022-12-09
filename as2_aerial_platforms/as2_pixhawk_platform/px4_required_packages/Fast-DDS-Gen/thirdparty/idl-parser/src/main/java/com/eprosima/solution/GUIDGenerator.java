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

package com.eprosima.solution;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

public class GUIDGenerator
{
    static MessageDigest digester = null;

    synchronized public static String genGUID(String name)
    {
	    if(digester == null)
        {
		    try
            {
			    digester = MessageDigest.getInstance("SHA-1");
			}
            catch (NoSuchAlgorithmException nsae)
            {
			    nsae.printStackTrace();
			}
		}

	    assert(digester != null) : "Digester Was null";

	    digester.reset();

	    byte digest[] = digester.digest(name.toLowerCase().getBytes());

	    assert(digest.length >= 16): "Digest too short";

	    StringBuffer buf = new StringBuffer();
	    String hex = null;
	    for(int i = 0; i < 16; i++)
        {
		    hex = Integer.toHexString(digest[i]);
		    if(hex.length() == 1){
			    buf.append('0');
			}else if(hex.length()> 2){
			    hex = hex.substring(hex.length() - 2);
			}
		    buf.append(hex.toUpperCase());
		    if(i == 3 || i == 5 || i == 7 || i == 9){
			    buf.append('-');
			}
		}
	    return buf.toString();
	}
}
