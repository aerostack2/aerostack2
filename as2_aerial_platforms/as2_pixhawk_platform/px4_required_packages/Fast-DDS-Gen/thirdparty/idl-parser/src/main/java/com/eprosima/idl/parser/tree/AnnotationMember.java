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

import com.eprosima.idl.parser.typecode.Member;
import com.eprosima.idl.parser.typecode.EnumMember;
import com.eprosima.idl.parser.typecode.EnumTypeCode;
import com.eprosima.idl.parser.typecode.TypeCode;

public class AnnotationMember
{
    public AnnotationMember(String name, TypeCode typecode, String value)
    {
        m_typecode = typecode;
        m_name = name;
        m_value = value;
    }

    public AnnotationMember(AnnotationMember ann)
    {
        m_typecode = ann.m_typecode;
        m_name = ann.m_name;
        m_value = ann.m_value;
    }

    public String getName()
    {
        return m_name;
    }

    /*
     * @brief This function is used with (previous c) types because array names contains [].
     */

    public TypeCode getTypecode()
    {
        return m_typecode;
    }

    public String getValue()
    {
        if (m_typecode.isIsType_c()) // Enum
        {
            EnumTypeCode enumTC = (EnumTypeCode)m_typecode;
            int idx = 0;
            for (Member m : enumTC.getMembers())
            {
                if (m.getName().equals(m_value))
                {
                    return Integer.toString(idx);
                }
                idx++;
            }
        }
        if (m_typecode.isIsType_d()) // String
        {
            if (m_value != null)
            {
                if (m_value.startsWith("\"") && m_value.endsWith("\""))
                {
                    return m_value.substring(1, m_value.length() - 1);
                }
            }
        }
        return m_value;
    }

    public void setValue(String value)
    {
        m_value = value;
    }

    private String m_name = null;

    private TypeCode m_typecode = null;

    private String m_value = null;
}
