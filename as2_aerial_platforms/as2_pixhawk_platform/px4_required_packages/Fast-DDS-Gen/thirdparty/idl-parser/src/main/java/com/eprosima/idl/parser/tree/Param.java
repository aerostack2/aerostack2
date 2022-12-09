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

import com.eprosima.idl.parser.typecode.TypeCode;

public class Param
{
    public enum Kind
    {
        IN_PARAM,
        OUT_PARAM,
        INOUT_PARAM
    };

    public boolean isInput()
    {
        if(m_kind == Kind.IN_PARAM || m_kind == Kind.INOUT_PARAM)
        {
            return true;
        }

        return false;
    }

    public boolean isOutput()
    {
        if(m_kind == Kind.OUT_PARAM || m_kind == Kind.INOUT_PARAM)
        {
            return true;
        }

        return false;
    }

    public boolean isOnlyOutput()
    {
        if(m_kind == Kind.OUT_PARAM)
        {
            return true;
        }

        return false;
    }

    public String getComment()
    {
        if(m_kind == Kind.IN_PARAM)
            return "in";
        else if(m_kind == Kind.OUT_PARAM)
            return "out";
        else if(m_kind == Kind. INOUT_PARAM)
            return "inout";

        return "error";
    }

    public Param(String name, TypeCode typecode, Kind kind)
    {
        m_name = name;
        m_typecode = typecode;
        m_kind = kind;
    }

    public Param(String name, Definition definition, Kind kind)
    {
        m_name = name;
        m_definition = definition;
        m_kind = kind;
    }

    public String getName()
    {
        return m_name;
    }

    public TypeCode getTypecode()
    {
        return m_typecode;
    }

    public Definition getDefinition()
    {
        return m_definition;
    }

    public void setParent(Object obj)
    {
        m_parent = obj;
    }

    public Object getParent()
    {
        return m_parent;
    }

    private Kind m_kind = Kind.IN_PARAM;
    private String m_name = null;
    private TypeCode m_typecode = null;
    private Definition m_definition = null;
    private Object m_parent = null;
}
