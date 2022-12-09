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
import com.eprosima.idl.parser.tree.Inherits;
import com.eprosima.idl.context.Context;

import org.antlr.stringtemplate.StringTemplate;

import java.util.List;
import java.util.ArrayList;


public class StructTypeCode extends MemberedTypeCode implements Inherits
{
    public StructTypeCode(String scope, String name)
    {
        super(Kind.KIND_STRUCT, scope, name);
        superTypes_ = new ArrayList<StructTypeCode>();
    }

    @Override
    public String getTypeIdentifier()
    {
        return "EK_MINIMAL";
    }

    @Override
    public boolean isObjectType() { return true; }

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
        st.setAttribute("name", getCScopedname());
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

    @Override
    public boolean isIsType_a(){
        return true;
    }

    @Override
    public void addInheritance(Context ctx, TypeCode parent)
    {
        if (parent instanceof StructTypeCode)
        {
            superTypes_.add((StructTypeCode)parent);
        }
    }

    @Override
    public ArrayList<TypeCode> getInheritances()
    {
        ArrayList<TypeCode> result = new ArrayList<TypeCode>();
        for (StructTypeCode parent : superTypes_)
        {
            result.add(parent);
        }
        return result;
    }

    @Override
    public List<Member> getMembers()
    {
        return getMembers(false);
    }

    public List<Member> getMembers(boolean includeParents)
    {
        List<Member> allMembers = new ArrayList<Member>();

        if (includeParents)
        {
            for (StructTypeCode p : superTypes_)
            {
                allMembers.addAll(p.getMembers());
            }
        }

        allMembers.addAll(super.getMembers());
        return allMembers;
    }

    public List<Member> getAllMembers() // Alias for getMembers(true) for stg
    {
        return getMembers(true);
    }

    private ArrayList<StructTypeCode> superTypes_;
}
