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

package com.eprosima.idl.parser.tree;

import java.util.Map;
import java.util.HashMap;
import java.util.Collection;

public class Annotation
{
    public Annotation(AnnotationDeclaration declaration)
    {
        m_declaration = declaration;
        m_members = new HashMap<String, AnnotationMember>();

        for(AnnotationMember ann : declaration.getMembers())
        {
            AnnotationMember member = new AnnotationMember(ann);
            m_members.put(member.getName(), member);
        }
    }

    public String getName()
    {
        if(m_declaration != null)
        {
            return m_declaration.getName();
        }

        return null;
    }

    public boolean addValue(String value)
    {
        if(m_members.size() != 1)
            return false;

        ((AnnotationMember)m_members.values().toArray()[0]).setValue(value);

        return true;
    }

    public boolean addValue(String attribute, String value)
    {
        AnnotationMember member = m_members.get(attribute);

        if(member != null)
        {
            member.setValue(value);
        }
        else
            return false;

        return true;
    }

    public String getValue()
    {
        if(m_members.size() != 1) return null;

        return ((AnnotationMember)m_members.values().toArray()[0]).getValue();
    }

    public String getValue(String attribute)
    {
        return m_members.get(attribute).getValue();
    }

    public Map<String, AnnotationMember> getValues()
    {
        return m_members;
    }

    public Collection<AnnotationMember> getValueList()
    {
        return m_members.values();
    }

    private HashMap<String, AnnotationMember> m_members = null;
    private AnnotationDeclaration m_declaration = null;
}
