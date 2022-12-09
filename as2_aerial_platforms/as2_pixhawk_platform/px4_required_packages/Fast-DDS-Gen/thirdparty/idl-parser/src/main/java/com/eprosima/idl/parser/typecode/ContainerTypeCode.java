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

import com.eprosima.idl.parser.tree.Definition;

public abstract class ContainerTypeCode extends TypeCode
{
    protected ContainerTypeCode(int kind)
    {
        super(kind);
    }

    @Override
    public abstract String getCppTypename();

    @Override
    public abstract String getCTypename();

    @Override
    public abstract String getIdlTypename();

    public TypeCode getContentTypeCode()
    {
        return m_contentTypeCode;
    }

    public Definition getContentDefinition()
    {
        return m_contentDefinition;
    }

    public void setContentTypeCode(TypeCode contentTypeCode)
    {
        m_contentTypeCode = contentTypeCode;
    }

    public void setContentDefinition(Definition contentDefinition)
    {
        m_contentDefinition = contentDefinition;
    }

    public int getDepth()
    {
        int ret = 1;

        if (m_contentTypeCode.isPrimitive()) {
    	    return ret;
    	} else {
    	    if (m_contentTypeCode instanceof ContainerTypeCode) {
    		    ret += ((ContainerTypeCode) m_contentTypeCode).getDepth();
    		}
    	}

        return ret;
    }

    private TypeCode m_contentTypeCode = null;
    private Definition m_contentDefinition = null;
}
