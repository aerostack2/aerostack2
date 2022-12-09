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

import java.util.ArrayList;
import java.util.LinkedHashSet;

import com.eprosima.solution.GUIDGenerator;
import com.eprosima.idl.util.Util;

public class Project
{
    public Project(String name, String file, LinkedHashSet<String> dependencies)
	{
	    m_name = name;
	    m_file = file;
	    m_dependencies = dependencies;
	    m_commonsrcfiles = new ArrayList<String>();
	    m_commonincludefiles = new ArrayList<String>();
	}

    public void setParent(Solution sol)
	{
	    m_parent = sol;
	}

    public Solution getParent()
    {
        return m_parent;
    }

    public String getName()
	{
	    return m_name;
	}

    public String getFile()
	{
	    return m_file;
	}

    public void addCommonSrcFile(String file)
	{
	    m_commonsrcfiles.add(file);
	}

    public ArrayList<String> getCommonSrcFiles()
	{
	    return m_commonsrcfiles;
	}

    public void addCommonIncludeFile(String file)
	{
	    m_commonincludefiles.add(file);
	}

    public ArrayList<String> getCommonIncludeFiles()
	{
	    return m_commonincludefiles;
	}

	/*!
	 * @brief Used in string templates.
	 */
    public ArrayList<String> getDependencies()
	{
	    ArrayList<String> array = new ArrayList<String>(m_dependencies);

	    for(int count = 0; count < array.size(); ++count)
		{
		    array.set(count, Util.getIDLFileNameOnly(array.get(count).toString()));
		}

	    return array;
	}

	/*!
	 * @brief Used in string templates.
	 */
    public String getGuid()
	{
	    return GUIDGenerator.genGUID(m_file);
	}

	/*!
	 * @brief Used in string templates.
	 */
    public ArrayList<String> getDependenciesGuids()
	{
	    ArrayList<String> deps = new ArrayList<String>(m_dependencies);
	    ArrayList<String> array = new ArrayList<String>();

	    for(int count = 0; count < deps.size(); ++count)
		{
		    if(!m_parent.getOS().contains("Windows") ||
				    m_parent.existsProject(deps.get(count)))
			{
				//System.out.println("Adding GUID of " + deps.get(count));
			    array.add(GUIDGenerator.genGUID(deps.get(count)));
			}
		}

	    return array;
	}

    public ArrayList<String> getFullDependencies()
	{
	    return new ArrayList<String>(m_dependencies);
	}

    private String m_name = null;
    private String m_file = null;
    private ArrayList<String> m_commonsrcfiles = null;
    private ArrayList<String> m_commonincludefiles = null;
    private LinkedHashSet<String> m_dependencies = null;
    String m_guid = null;
    Solution m_parent = null;
}
