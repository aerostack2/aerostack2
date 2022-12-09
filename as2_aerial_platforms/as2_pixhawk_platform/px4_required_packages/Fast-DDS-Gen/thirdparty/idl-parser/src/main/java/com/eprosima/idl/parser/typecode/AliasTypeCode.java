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
import java.util.List;
import org.antlr.stringtemplate.StringTemplate;



public class AliasTypeCode extends ContainerTypeCode
{
    public AliasTypeCode(String scope, String name)
    {
        super(Kind.KIND_ALIAS);
        m_scope = scope;
        m_name = name;
    }

    @Override
    public TypeCode getContentTypeCode()
    {
        if(super.getContentTypeCode() instanceof AliasTypeCode)
        {
            AliasTypeCode alias = (AliasTypeCode)super.getContentTypeCode();
            return alias.getContentTypeCode();
        }
        else if(super.getContentTypeCode() instanceof ContainerTypeCode)
        {
            ContainerTypeCode container = (ContainerTypeCode)super.getContentTypeCode();
            return container.getContentTypeCode();
        }

        return super.getContentTypeCode();
    }

    public boolean isUnbound()
    {
        if(super.getContentTypeCode() instanceof SequenceTypeCode)
        {
            return ((SequenceTypeCode)super.getContentTypeCode()).isUnbound();
        }
        else
        {
            return false;
        }
    }

    @Override
    public boolean isObjectType() { return true; }

    public TypeCode getTypedefContentTypeCode()
    {
        return super.getContentTypeCode();
    }

    public String getName()
    {
        return m_name;
    }

    public String getScopedname()
    {
        if(m_scope.isEmpty())
            return m_name;

        return m_scope + "::" + m_name;
    }

    public String getROS2Scopedname()
    {
        if(m_scope.isEmpty())
            return m_name;

        return m_scope + "::dds_::" + m_name + "_";
    }

    public String getScope()
    {
        return m_scope;
    }

    public boolean getHasScope()
    {
        return !m_scope.isEmpty();
    }

    @Override
    public String getCppTypename()
    {
        StringTemplate st = getCppTypenameFromStringTemplate();
        st.setAttribute("name", getScopedname());
        return st.toString();
    }

    @Override
    public String getCTypename()
    {
        StringTemplate st = getCTypenameFromStringTemplate();
        st.setAttribute("name", getScopedname());
        return st.toString();
    }

    @Override
    public String getJavaTypename()
    {
        StringTemplate st = getJavaTypenameFromStringTemplate();
        st.setAttribute("name", getTypedefContentTypeCode().getJavaTypename());
        return st.toString();
    }

    @Override
    public String getIdlTypename()
    {
        StringTemplate st = getIdlTypenameFromStringTemplate();
        st.setAttribute("name", getScopedname());
        return st.toString();
    }

    @Override
    public String getStType()
    {
        return super.getContentTypeCode().getStType();
    }

    @Override
    public boolean isPrimitive()
    {
        return super.getContentTypeCode().isPrimitive();
    }

    @Override
    public boolean isIsStringType() { return super.getContentTypeCode().isIsStringType(); }

    @Override
    public boolean isIsWStringType() { return super.getContentTypeCode().isIsWStringType(); }

    @Override
    public String getInitialValue()
    {
        return super.getContentTypeCode().getInitialValue();
    }

    public Pair<Integer, Integer> getMaxSerializedSize(int currentSize, int lastDataAligned)
    {
        // TODO
        return null;
    }

    public int getMaxSerializedSizeWithoutAlignment(int currentSize)
    {
        // TODO
        return 0;
    }

    /*** Functions to know the type in string templates ***/
    @Override
    public boolean isIsType_d()
    {
        return super.getContentTypeCode().isIsType_d();
    }

    @Override
    public boolean isIsType_c()
    {
        return super.getContentTypeCode().isIsType_c();
    }

    @Override
    public boolean isIsType_f()
    {
        return super.getContentTypeCode().isIsType_f();
    }

    public boolean isIsType_e()
    {
        return super.getContentTypeCode().isIsType_e();
    }

    @Override
    public boolean isIsSequenceType() { return super.getContentTypeCode().isIsSequenceType(); }

    public boolean isIsType_10()
    {
        return true;
    }

    @Override
    public String getTypeIdentifier()
    {
        return "EK_MINIMAL";
    }

    /*** End of functions to know the type in string templates ***/

    /*** Functions that alias has to export because some typecodes have them*/
    public String getMaxsize()
    {
        return super.getContentTypeCode().getMaxsize();
    }

    public String getSize()
    {
        return super.getContentTypeCode().getSize();
    }

    public List<String> getDimensions()
    {
        if(super.getContentTypeCode() instanceof ArrayTypeCode)
            return ((ArrayTypeCode)super.getContentTypeCode()).getDimensions();

        return null;
    }
    /*** End of functions that alias has to export because some typecodes have them*/

    private String m_name = null;

    private String m_scope = null;
}
