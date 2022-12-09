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

import com.eprosima.idl.context.Context;
import com.eprosima.idl.parser.typecode.Member;

import java.util.ArrayList;
import java.util.List;
import org.antlr.v4.runtime.Token;

public class Exception extends TreeNode implements Export, Definition
{
    public Exception(String scopeFile, boolean isInScope, String scope, String name, Token token)
    {
        super(scopeFile, isInScope, scope, name, token);

        m_members = new ArrayList<Member>();
    }

	@Override
    public boolean isIsModule()
    {
        return false;
    }

	@Override
    public boolean isIsInterface()
    {
        return false;
    }

	@Override
    public boolean isIsTypeDeclaration()
    {
        return false;
    }

	@Override
    public boolean isIsConstDeclaration()
    {
        return false;
    }

    public void setParent(Object obj)
    {
        m_parent = obj;
    }

    public Object getParent()
    {
        return m_parent;
    }

    @Override
    public boolean isIsOperation()
    {
        return false;
    }

    @Override
    public boolean isIsException()
    {
        return true;
    }

	@Override
    public boolean isIsAnnotation()
    {
        return false;
    }

    public List<Member> getMembers()
    {
        return m_members;
    }

    public int addMember(Member member)
    {
        m_members.add(member);
        return m_members.size() - 1;
    }

    @Override
    public boolean resolve(Context ctx)
    {
        return true;
    }

    private Object m_parent = null;
    private List<Member> m_members = null;
}
