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

import com.eprosima.idl.generator.manager.TemplateGroup;
import com.eprosima.log.ColorMessage;
import com.eprosima.idl.parser.typecode.TypeCode;
import com.eprosima.idl.context.Context;

import java.util.Iterator;
import java.util.Map;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;
import java.util.List;
import java.util.ArrayList;

import org.antlr.stringtemplate.*;
import org.antlr.stringtemplate.language.DefaultTemplateLexer;

public class TemplateManager
{
    class TemplateErrorListener implements StringTemplateErrorListener
    {
        public void error(String arg0, Throwable arg1)
        {
            System.out.println(ColorMessage.error() + arg0);
            arg1.printStackTrace();
        }

        public void warning(String arg0)
        {
            System.out.println(ColorMessage.warning() + arg0);
        }
    }

    static private String m_loaderDirectories = "com/eprosima/idl/templates";
    private Map<String, StringTemplateGroup> m_groups = null;
    private Map<String, List<TemplateExtension>> m_extensions = null;
    private StringTemplateGroup strackgr_ = null;

    public TemplateManager(String stackTemplateNames, Context ctx, boolean generate_typesc)
    {
        StringTemplateGroupLoader loader = new CommonGroupLoader(m_loaderDirectories, new TemplateErrorListener());
        StringTemplateGroup.registerGroupLoader(loader);

        // Load IDL types for stringtemplates
        TypeCode.idltypesgr = StringTemplateGroup.loadGroup("idlTypes", DefaultTemplateLexer.class, null);
        if (generate_typesc)
        {
            TypeCode.cpptypesgr = StringTemplateGroup.loadGroup("TypesCInterface", DefaultTemplateLexer.class, null);
        }
        else
        {
            TypeCode.cpptypesgr = StringTemplateGroup.loadGroup("Types", DefaultTemplateLexer.class, null);
        }
        TypeCode.ctypesgr = StringTemplateGroup.loadGroup("CTypes", DefaultTemplateLexer.class, null);
        TypeCode.javatypesgr = StringTemplateGroup.loadGroup("JavaTypes", DefaultTemplateLexer.class, null);
        TypeCode.ctx = ctx;

        m_groups = new HashMap<String, StringTemplateGroup>();
        m_extensions = new HashMap<String, List<TemplateExtension>>();

        // Load specific template rules.
        if(stackTemplateNames != null && !stackTemplateNames.isEmpty())
        {
            int index = -1, lastIndex = 0;
            String templateName = null;

            while((index = stackTemplateNames.indexOf(':', lastIndex)) != -1)
            {
                templateName = stackTemplateNames.substring(lastIndex, index);
                strackgr_ = StringTemplateGroup.loadGroup(templateName, DefaultTemplateLexer.class, strackgr_);
                lastIndex = index + 1;
            }

            templateName = stackTemplateNames.substring(lastIndex, stackTemplateNames.length());
            strackgr_ = StringTemplateGroup.loadGroup(templateName, DefaultTemplateLexer.class, strackgr_);
        }
    }

    static public void setGroupLoaderDirectories(String paths)
    {
        m_loaderDirectories += ":" + paths;
    }

    public void changeCppTypesTemplateGroup(String templateName)
    {
        TypeCode.cpptypesgr = StringTemplateGroup.loadGroup(templateName, DefaultTemplateLexer.class, null);
    }

    public void addGroup(String groupname)
    {
        StringTemplateGroup group = StringTemplateGroup.loadGroup(groupname, DefaultTemplateLexer.class, strackgr_);
        m_groups.put(groupname, group);
    }

    public void addGroup(String groupname, List<TemplateExtension> extensions)
    {
        addGroup(groupname);

        for(TemplateExtension extension : extensions)
        {
            String str = groupname + "_" + extension.getRuleName();
            List<TemplateExtension> list = null;

            if(m_extensions.containsKey(str))
            {
                list = m_extensions.get(str);
            }
            else
            {
                list = new ArrayList<TemplateExtension>();
            }

            // Set stack groups.
            extension.setCommonGroup(strackgr_);
            list.add(extension);
            m_extensions.put(str, list);
        }
    }

    public TemplateGroup createTemplateGroup(String templatename)
    {
        TemplateGroup tg = new TemplateGroup();
        Set<Entry<String, StringTemplateGroup>> set = m_groups.entrySet();
        Iterator<Entry<String, StringTemplateGroup>> it = set.iterator();

        while(it.hasNext())
        {
            Map.Entry<String, StringTemplateGroup> m = (Map.Entry<String, StringTemplateGroup>)it.next();

            // Obtain instance
            StringTemplate template = m.getValue().getInstanceOf(templatename);

            if(!m_extensions.containsKey(m.getKey() + "_" + template.getName()))
            {
                tg.addTemplate(m.getKey(), template);
            }
            else
            {
                List<TemplateExtension> extensions = m_extensions.get(m.getKey() + "_" + template.getName());
                List<StringTemplate> extensionstemplates = new ArrayList<StringTemplate>();

                for(TemplateExtension extension : extensions)
                {
                    extensionstemplates.add(extension.createStringTemplate());
                }

                tg.addTemplate(m.getKey(), template, extensionstemplates);
            }
        }

        return tg;
    }

    public StringTemplate createStringTemplate(String templatename)
    {
        return strackgr_.getInstanceOf(templatename);
    }

    public StringTemplateGroup createStringTemplateGroup(String templateGroupName)
    {
        return StringTemplateGroup.loadGroup(templateGroupName, DefaultTemplateLexer.class, strackgr_);
    }
}
