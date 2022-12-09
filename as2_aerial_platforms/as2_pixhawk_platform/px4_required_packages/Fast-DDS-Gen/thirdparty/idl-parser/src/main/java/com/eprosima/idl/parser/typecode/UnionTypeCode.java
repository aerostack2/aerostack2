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



public class UnionTypeCode extends MemberedTypeCode
{
    public UnionTypeCode(String scope, String name)
    {
        super(Kind.KIND_UNION, scope, name);
        m_discriminatorTypeCode = null;
    }

    public UnionTypeCode(String scope, String name, TypeCode discriminatorTypeCode)
    {
        super(Kind.KIND_UNION, scope, name);
        m_discriminatorTypeCode = discriminatorTypeCode;
    }

    public void setDiscriminatorType(TypeCode discriminatorTypeCode)
    {
        m_discriminatorTypeCode = discriminatorTypeCode;
    }

    @Override
    public String getTypeIdentifier()
    {
        return "EK_MINIMAL";
    }

    @Override
    public boolean isObjectType() { return true; }

    @Override
    public boolean isIsUnionType() {return true; }

    /*!
     * @return 0 is ok, -1 the member is repeated, -2 is another default member.
     */
    public int addMember(UnionMember member)
    {
        if(member.isDefault())
        {
            if(m_defaultindex == -1)
                m_defaultindex = getMembers().size();
            else
                return -2;
        }

        // Generate labels
        List<String> internal_labels = member.getInternalLabels();
        List<String> labels = null;
        List<String> javalabels = null;

        if(m_discriminatorTypeCode.getKind() == Kind.KIND_ENUM)
        {
            EnumTypeCode enum_type = (EnumTypeCode)m_discriminatorTypeCode;
            labels = new ArrayList<String>();
            javalabels = new ArrayList<String>();

            for(int count = 0; count < internal_labels.size(); ++count)
            {
                labels.add(enum_type.getScope() + "::" + internal_labels.get(count));
                javalabels.add(javapackage + enum_type.getJavaScopedname() + "." + internal_labels.get(count));
            }
        }
        else
        {
            labels = internal_labels;
            javalabels = internal_labels;
        }

        member.setLabels(labels);
        member.setJavaLabels(javalabels);

        if(!addMember((Member)member))
            return -1;

        return 0;
    }

    public Member getDefaultMember()
    {
        if(m_defaultindex != -1)
            return getMembers().get(m_defaultindex);

        return null;
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
        st.setAttribute("name", getJavaScopedname());
        return st.toString();
    }

    @Override
    public String getIdlTypename()
    {
        StringTemplate st = getIdlTypenameFromStringTemplate();
        st.setAttribute("name", getScopedname());
        return st.toString();
    }

    public void setDefaultvalue(String value)
    {
        m_defaultValue = value;
    }

    public void setJavaDefaultvalue(String value)
    {
        m_javaDefaultValue = value;
    }

    // Used in stringtemplates
    public String getDefaultvalue()
    {
        return m_defaultValue;
    }

    // Used in stringtemplates
    public String getJavaDefaultvalue()
    {
        return m_javaDefaultValue;
    }

    // Used in stringtemplates
    public TypeCode getDiscriminator()
    {
        return m_discriminatorTypeCode;
    }

    // Used in stringtemplates
    public List<String> getTotallabels()
    {
        List<String> returnList = new ArrayList<String>();
        List<Member> mlist = getMembers();
        List<String> labels = null;

        for(int count = 0; count < mlist.size(); ++count)
        {
            if(count != m_defaultindex)
            {
                labels = ((UnionMember)mlist.get(count)).getLabels();
                for(int i = 0; i < labels.size(); ++i)
                    returnList.add(labels.get(i));
            }
        }

        return returnList;
    }

    /*public Pair<Integer, Integer> getMaxSerializedSize(int currentSize, int lastDataAligned)
    {
        List<Member> members = getMembers();
        int lcurrentSize = currentSize, lmaxSize = 0;
        int llastDataAligned = 0;

        Pair<Integer, Integer> dpair = m_discriminatorTypeCode.getMaxSerializedSize(lcurrentSize, lastDataAligned);
        lcurrentSize = dpair.first();

        for(int count = 0; count < members.size(); ++count)
        {
            Pair<Integer, Integer> pair = members.get(count).getTypecode().getMaxSerializedSize(lcurrentSize, dpair.second());

            if(pair.first() > lmaxSize)
            {
                lmaxSize = pair.first();
                llastDataAligned = pair.second();
            }
        }

        return new Pair<Integer, Integer>(lmaxSize, llastDataAligned);
    }

    public int getMaxSerializedSizeWithoutAlignment(int currentSize)
    {
        List<Member> members = getMembers();
        int lcurrentSize = currentSize, lmaxSize = 0;

        lcurrentSize = m_discriminatorTypeCode.getMaxSerializedSizeWithoutAlignment(lcurrentSize);

        for(int count = 0; count < members.size(); ++count)
        {
            int aux = members.get(count).getTypecode().getMaxSerializedSizeWithoutAlignment(lcurrentSize);

            if(aux > lmaxSize)
            {
                lmaxSize = aux;
            }
        }

        return lmaxSize;
    }

    public String getMaxSerializedSize()
    {
        Pair<Integer, Integer> pair = getMaxSerializedSize(0, 0);
        return pair.first().toString();
    }

    public String getMaxSerializedSizeWithoutAlignment()
    {
        return Integer.toString(getMaxSerializedSizeWithoutAlignment(0));
    }*/

    private TypeCode m_discriminatorTypeCode = null;

    private int m_defaultindex = -1;

    private String m_defaultValue = null;

    private String m_javaDefaultValue = null;
}
