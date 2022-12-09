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

import com.eprosima.idl.util.Pair;
import org.antlr.stringtemplate.StringTemplate;


public class StringTypeCode extends TypeCode
{
    public StringTypeCode(int kind, String maxsize)
    {
        super(kind);
        m_maxsize = maxsize;
    }

    @Override
    public boolean isIsType_d(){return true;}

    @Override
    public String getTypeIdentifier()
    {
        switch(getKind())
        {
            case Kind.KIND_STRING:
                return "TI_STRING8_SMALL";
            case Kind.KIND_WSTRING:
                return "TI_STRING16_SMALL";
            default:
                return "TK_None";
        }
    }

    @Override
    public boolean isPlainType() { return true; }

    @Override
    public boolean isIsStringType() { return getKind() == Kind.KIND_STRING; }

    @Override
    public boolean isIsWStringType() { return getKind() == Kind.KIND_WSTRING; }

    @Override
    public String getCppTypename()
    {
        return getCppTypenameFromStringTemplate().toString();
    }

    @Override
    public String getCTypename()
    {
        StringTemplate st = getCTypenameFromStringTemplate();
        st.setAttribute("maxsize", getMaxsize());
        return st.toString();
    }

    @Override
    public String getJavaTypename()
    {
        return getJavaTypenameFromStringTemplate().toString();
    }

    @Override
    public String getIdlTypename()
    {
        return getIdlTypenameFromStringTemplate().toString();
    }

    @Override
    public String getInitialValue()
    {
        return getInitialValueFromStringTemplate();
    }

    public String getMaxsize()
    {
        if(m_maxsize == null)
            return "255";

        return m_maxsize;
    }

    public Pair<Integer, Integer> getMaxSerializedSize(int currentSize, int lastDataAligned)
    {
        int lcurrentSize = currentSize;

        // Length
        if(4 <= lastDataAligned)
        {
            lcurrentSize += 4;
        }
        else
        {
            int align = (4 - (lcurrentSize % 4)) & (4 - 1);
            lcurrentSize += 4 + align;
        }

        if(m_maxsize == null)
        {
            return new Pair<Integer, Integer>(lcurrentSize + 255 + 1, 1);
        }
        else
        {
            return new Pair<Integer, Integer>(lcurrentSize + Integer.parseInt(m_maxsize) + 1, 1);
        }
    }

    public int getMaxSerializedSizeWithoutAlignment(int currentSize)
    {
        if(m_maxsize == null)
        {
            return currentSize + 4 + 255 + 1;
        }
        else
        {
            return currentSize + 4 + Integer.parseInt(m_maxsize) + 1;
        }
    }

    private String m_maxsize = null;
}
