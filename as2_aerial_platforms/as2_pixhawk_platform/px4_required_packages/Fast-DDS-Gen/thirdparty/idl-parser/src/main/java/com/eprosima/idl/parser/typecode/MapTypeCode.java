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

package com.eprosima.idl.parser.typecode;

import org.antlr.stringtemplate.StringTemplate;
import com.eprosima.idl.parser.tree.Definition;


public class MapTypeCode extends ContainerTypeCode
{
    public MapTypeCode(String maxsize)
    {
        super(Kind.KIND_MAP);
        m_maxsize = maxsize;
    }

    @Override
    public boolean isIsType_e(){return true;}

    @Override
    public String getTypeIdentifier()
    {
        return "TI_PLAIN_MAP_SMALL";
    }

    @Override
    public boolean isPlainType() { return true; }

    @Override
    public boolean isIsMapType() { return true; }

    @Override
    public String getCppTypename()
    {
        StringTemplate st = getCppTypenameFromStringTemplate();
        st.setAttribute("key", getKeyTypeCode().getCppTypename());
        st.setAttribute("value", getValueTypeCode().getCppTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    @Override
    public String getCTypename()
    {
        StringTemplate st = getCTypenameFromStringTemplate();
        st.setAttribute("key", getKeyTypeCode().getCTypename());
        st.setAttribute("value", getValueTypeCode().getCTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    @Override
    public String getJavaTypename()
    {
        StringTemplate st = getJavaTypenameFromStringTemplate();
        st.setAttribute("key", getKeyTypeCode().getJavaTypename());
        st.setAttribute("value", getValueTypeCode().getJavaTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    @Override
    public String getIdlTypename()
    {
        StringTemplate st = getIdlTypenameFromStringTemplate();
        st.setAttribute("key", getKeyTypeCode().getIdlTypename());
        st.setAttribute("value", getValueTypeCode().getIdlTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    public String getMaxsize()
    {
        if(m_maxsize == null)
            return "100";

        return m_maxsize;
    }

    public TypeCode getKeyTypeCode()
    {
        return m_keyTypeCode;
    }

    public void setKeyTypeCode(TypeCode keyTypeCode)
    {
        m_keyTypeCode = keyTypeCode;
    }

    public TypeCode getValueTypeCode()
    {
        return m_valueTypeCode;
    }

    public void setValueTypeCode(TypeCode valueTypeCode)
    {
        m_valueTypeCode = valueTypeCode;
    }

    public Definition getKeyDefinition()
    {
        return m_keyDefinition;
    }

    public void setKeyDefinition(Definition keyDefinition)
    {
        m_keyDefinition = keyDefinition;
    }

    public Definition getValueDefinition()
    {
        return m_valueDefinition;
    }

    public void setValueDefinition(Definition valueDefinition)
    {
        m_valueDefinition = valueDefinition;
    }

    private TypeCode m_keyTypeCode = null;
    private TypeCode m_valueTypeCode = null;
    private Definition m_keyDefinition = null;
    private Definition m_valueDefinition = null;
    private String m_maxsize = null;
}
