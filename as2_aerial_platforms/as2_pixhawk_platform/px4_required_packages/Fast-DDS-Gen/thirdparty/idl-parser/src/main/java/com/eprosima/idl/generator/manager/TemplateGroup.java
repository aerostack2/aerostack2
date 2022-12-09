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

package com.eprosima.idl.generator.manager;

import java.util.Iterator;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.Map.Entry;
import java.util.List;
import java.util.ArrayList;

import org.antlr.stringtemplate.StringTemplate;
import org.antlr.stringtemplate.StringTemplateGroup;

import com.eprosima.log.Log;

public class TemplateGroup
{
    private Map<String, StringTemplate> m_templates = null;
    private Map<String, List<StringTemplate>> m_extensionstemplates = null;

    public TemplateGroup()
    {
        m_templates = new HashMap<String, StringTemplate>();
        m_extensionstemplates = new HashMap<String, List<StringTemplate>>();
    }

    public void addTemplate(String groupname, StringTemplate template)
    {
        m_templates.put(groupname, template);
    }

    public void addTemplate(String groupname, StringTemplate template, List<StringTemplate> extensionstemplates)
    {
        addTemplate(groupname, template);
        m_extensionstemplates.put(groupname + "_" + template.getName(), extensionstemplates);
    }

    public StringTemplate getTemplate(String groupname)
    {
        StringTemplate template = m_templates.get(groupname);

        //If there is extensiones, add them before return the template.
        if(m_extensionstemplates.containsKey(groupname + "_" + template.getName()))
        {
            List<StringTemplate> extemplates = new ArrayList<StringTemplate>();
            List<StringTemplate> extensions = m_extensionstemplates.get(groupname + "_" + template.getName());

            for(StringTemplate extension : extensions)
            {
                extemplates.add(extension);
            }

            template.setAttribute("extensions", extemplates);
        }

        return template;
    }

    public void setAttribute(String attribute, TemplateGroup tg)
    {
        if(tg != null)
        {
            Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
            Iterator<Entry<String, StringTemplate>> it = set.iterator();

            while(it.hasNext())
            {
                Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

                // Call setAttribute
                StringTemplate template = tg.getTemplate(m.getKey());

                if(template != null)
                {
                    Log.printDebug("setting attribute (TemplateGroup) to template group " + m.getKey() + " from " + template.getName() + " to " + m.getValue().getName());
                    m.getValue().setAttribute(attribute, template.toString());
                }
            }
        }
    }

    public void setAttribute(String attribute, Object obj1)
    {
        Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
        Iterator<Entry<String, StringTemplate>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

            // Call setAttribute
            Log.printDebug("setting attribute (obj1) to template group " + m.getKey() + " to " + m.getValue().getName());
            StringTemplate template = m.getValue();
            template.setAttribute(attribute, obj1);
            // Update extensions
            List<StringTemplate> extensions = m_extensionstemplates.get(m.getKey() + "_" + template.getName());
            if(extensions != null)
            {
                for(StringTemplate extension : extensions)
                {
                    extension.setAttribute(attribute, obj1);
                }
            }
        }
    }

    public void setAttribute(String attribute, Object obj1, Object obj2)
    {
        Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
        Iterator<Entry<String, StringTemplate>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

            // Call setAttribute
            Log.printDebug("setting attribute (obj1,obj2) to template group " + m.getKey() + " to " + m.getValue().getName());
            m.getValue().setAttribute(attribute, obj1, obj2);
        }
    }

    public void setAttribute(String attribute, Object obj1, Object obj2, Object obj3)
    {
        Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
        Iterator<Entry<String, StringTemplate>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

            // Call setAttribute
            Log.printDebug("setting attribute (obj1,obj2,obj3) to template group " + m.getKey() + " to " + m.getValue().getName());
            m.getValue().setAttribute(attribute, obj1, obj2, obj3);
        }
    }

    public void setAttribute(String attribute, Object obj1, Object obj2, Object obj3, Object obj4)
    {
        Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
        Iterator<Entry<String, StringTemplate>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

            // Call setAttribute
            Log.printDebug("setting attribute (obj1,obj2,obj3,obj4) to template group " + m.getKey() + " to " + m.getValue().getName());
            m.getValue().setAttribute(attribute, obj1, obj2, obj3, obj4);
        }
    }

    public void setAttribute(String attribute, Object obj1, Object obj2, Object obj3, Object obj4, Object obj5)
    {
        Set<Entry<String, StringTemplate>> set = m_templates.entrySet();
        Iterator<Entry<String, StringTemplate>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplate> m = (Map.Entry<String, StringTemplate>)it.next();

            // Call setAttribute
            Log.printDebug("setting attribute (obj1,obj2,obj3,obj4,obj5) to template group " + m.getKey() + " to " + m.getValue().getName());
            m.getValue().setAttribute(attribute, obj1, obj2, obj3, obj4, obj5);
        }
    }
}
