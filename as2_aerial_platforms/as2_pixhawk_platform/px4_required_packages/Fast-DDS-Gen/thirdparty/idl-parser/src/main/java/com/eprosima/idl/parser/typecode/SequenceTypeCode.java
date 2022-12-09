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

import com.eprosima.idl.parser.typecode.StringTypeCode;
import com.eprosima.idl.parser.typecode.MapTypeCode;
import org.antlr.stringtemplate.StringTemplate;


public class SequenceTypeCode extends ContainerTypeCode
{
    public SequenceTypeCode(String maxsize)
    {
        super(Kind.KIND_SEQUENCE);
        m_maxsize = maxsize;
    }

    @Override
    public boolean isIsType_e(){return true;}

    @Override
    public String getTypeIdentifier()
    {
        return "TI_PLAIN_SEQUENCE_SMALL";
    }

    @Override
    public boolean isPlainType() { return true; }

    @Override
    public boolean isIsSequenceType() { return true; }

    @Override
    public String getCppTypename()
    {
        StringTemplate st = getCppTypenameFromStringTemplate();
        st.setAttribute("ctx", ctx);
        st.setAttribute("type", getContentTypeCode().getCppTypename());
        String contenttype = getContentTypeCode().getCppTypename().replaceAll("::", "_");
        if(getContentTypeCode() instanceof StringTypeCode)
        {
            contenttype = contenttype.replace("*", "_ptr_") + ((StringTypeCode)getContentTypeCode()).getMaxsize();
        }
        st.setAttribute("contenttype", contenttype);
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    @Override
    public String getCTypename()
    {
        StringTemplate st = getCTypenameFromStringTemplate();
        st.setAttribute("type", getContentTypeCode().getCTypename());
        st.setAttribute("maxsize", getMaxsize());
        return st.toString();
    }

    public String getCTypeDimensions()
    {
        String dimensions = "[" + getMaxsize()  + "]";
        if(getContentTypeCode() instanceof StringTypeCode)
        {
            dimensions += "[" + ((StringTypeCode)getContentTypeCode()).getMaxsize() + "]";
        }

        return dimensions;
    }

    @Override
    public String getJavaTypename()
    {
        StringTemplate st = getJavaTypenameFromStringTemplate();
        st.setAttribute("type", getContentTypeCode().getJavaTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    @Override
    public String getIdlTypename()
    {
        StringTemplate st = getIdlTypenameFromStringTemplate();
        st.setAttribute("type", getContentTypeCode().getIdlTypename());
        st.setAttribute("maxsize", m_maxsize);
        return st.toString();
    }

    public String getMaxsize()
    {
        if(m_maxsize == null)
            return "0";

        return m_maxsize;
    }

    public boolean isUnbound()
    {
        return getMaxsize().equals("0");
    }

    private String m_maxsize = null;
}
