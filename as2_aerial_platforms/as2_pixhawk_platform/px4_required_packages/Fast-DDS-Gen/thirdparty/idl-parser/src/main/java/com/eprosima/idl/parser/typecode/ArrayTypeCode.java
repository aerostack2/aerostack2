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

import java.util.ArrayList;
import java.util.List;
import org.antlr.stringtemplate.StringTemplate;



public class ArrayTypeCode extends ContainerTypeCode
{
    public ArrayTypeCode()
    {
        super(Kind.KIND_ARRAY);
        m_dimensions = new ArrayList<String>();
    }

    @Override
    public boolean isIsType_f(){return true;}

    @Override
    public String getTypeIdentifier()
    {
        return "TI_PLAIN_ARRAY_SMALL";
    }

    @Override
    public boolean isPlainType() { return true; }

    @Override
    public boolean isIsArrayType() { return true; }

    @Override
    public String getCppTypename()
    {
        StringTemplate first = null, second = null, fin = null;
        String prevf = null, prevs = null;

        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            first = cpptypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_first");
            second = cpptypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_second");
            second.setAttribute("size", m_dimensions.get(count));

            if(prevf != null)
            {
                first.setAttribute("prev", prevf);
            }
            if(prevs != null)
            {
                second.setAttribute("prev", prevs);
            }

            prevf = first.toString();
            prevs = second.toString();
        }

        fin = getCppTypenameFromStringTemplate();
        fin.setAttribute("firs", prevf);
        fin.setAttribute("secon", prevs);
        fin.setAttribute("type", getContentTypeCode().getCppTypename());

        return fin.toString();
    }

    @Override
    public String getCTypename()
    {
        StringTemplate first = null, second = null, fin = null;
        String prevf = null, prevs = null;

        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            first = ctypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_first");
            second = ctypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_second");
            second.setAttribute("size", m_dimensions.get(count));

            if(prevf != null)
            {
                first.setAttribute("prev", prevf);
            }
            if(prevs != null)
            {
                second.setAttribute("prev", prevs);
            }

            prevf = first.toString();
            prevs = second.toString();
        }

        fin = getCTypenameFromStringTemplate();
        fin.setAttribute("firs", prevf);
        fin.setAttribute("secon", prevs);
        fin.setAttribute("type", getContentTypeCode().getCTypename());

        return fin.toString();
    }
    public String getCTypeDimensions()
    {
        String dimensions = getArrayExtension();

        if(getContentTypeCode() instanceof StringTypeCode)
        {
            dimensions += "[" + ((StringTypeCode)getContentTypeCode()).getMaxsize() + "]";
        }

        return dimensions;
    }

    @Override
    public String getJavaTypename()
    {
        StringTemplate first = null, second = null, fin = null;
        String prevf = null, prevs = null;

        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            first = cpptypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_first");
            second = cpptypesgr.getInstanceOf("type_" + Integer.toHexString(Kind.KIND_ARRAY) + "_second");
            second.setAttribute("size", m_dimensions.get(count));

            if(prevf != null)
            {
                first.setAttribute("prev", prevf);
            }
            if(prevs != null)
            {
                second.setAttribute("prev", prevs);
            }

            prevf = first.toString();
            prevs = second.toString();
        }

        fin = getJavaTypenameFromStringTemplate();
        fin.setAttribute("firs", prevf);
        fin.setAttribute("secon", prevs);
        fin.setAttribute("type", getContentTypeCode().getJavaTypename());

        return fin.toString();
    }

    public String getIdlTypename()
    {
        return getContentTypeCode().getIdlTypename();
    }

    public void addDimension(String dimension)
    {
        m_dimensions.add(dimension);
    }

    public List<String> getDimensions()
    {
        return m_dimensions;
    }

    public String getSize()
    {
        String ret = "";

        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            if(ret.isEmpty())
                ret += "(";
            else
                ret += " * ";

            ret += m_dimensions.get(count);
        }

        if(!ret.isEmpty())
            ret += ")";

        return ret;
    }

    // TODO Used in stringtemplate for RTI DDS types.
    public String getArrayExtension()
    {
        String ret = "";

        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            ret += "[" + m_dimensions.get(count) + "]";
        }

        return ret;
    }

    /*public Pair<Integer, Integer> getMaxSerializedSize(int currentSize, int lastDataAligned)
    {
        int lcontainTypeSize = getContentTypeCode().getSize();
        int lcontainTypeAlign = 0;
        int larraySize = 1;

        // Element contained type.
        if(lcontainTypeSize > 4)
        {
            lcontainTypeAlign = (lcontainTypeSize - (currentSize % lcontainTypeSize)) & (lcontainTypeSize - 1);
        }

        // Calculate array size.
        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            larraySize *= Integer.parseInt(m_dimensions.get(count));
        }

        return new Pair<Integer, Integer>(currentSize + lcontainTypeAlign + (larraySize *  lcontainTypeSize), lcontainTypeSize);
    }

    public int getMaxSerializedSizeWithoutAlignment(int currentSize)
    {
        int lcontainTypeSize = getContentTypeCode().getSize();
        int larraySize = 1;

        // Calculate array size.
        for(int count = 0; count < m_dimensions.size(); ++count)
        {
            larraySize *= Integer.parseInt(m_dimensions.get(count));
        }

        return currentSize + (larraySize * lcontainTypeSize);
    }*/

    private List<String> m_dimensions;
}
