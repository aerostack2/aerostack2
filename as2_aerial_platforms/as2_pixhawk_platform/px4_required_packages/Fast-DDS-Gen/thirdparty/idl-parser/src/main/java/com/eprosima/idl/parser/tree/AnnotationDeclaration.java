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
import com.eprosima.idl.parser.typecode.TypeCode;
import com.eprosima.idl.parser.typecode.EnumTypeCode;;
import com.eprosima.idl.parser.typecode.AliasTypeCode;
import com.eprosima.idl.parser.tree.ConstDeclaration;

import java.util.List;
import java.util.ArrayList;
import java.util.Vector;
import java.util.HashMap;
import org.antlr.v4.runtime.Token;

public class AnnotationDeclaration extends TreeNode implements Definition
{
    public AnnotationDeclaration(String scopeFile, boolean isInScope, String scope, String name, Token token)
    {
        super(scopeFile, isInScope, scope, name, token);
        m_members = new HashMap<String, AnnotationMember>();
        m_enums = new Vector<EnumTypeCode>();
        m_consts = new Vector<ConstDeclaration>();
        m_typedefs = new Vector<AliasTypeCode>();
    }

    public List<AnnotationMember> getMembers()
    {
        return new ArrayList<AnnotationMember>(m_members.values());
    }

    public void addMembers(AnnotationDeclaration annotation)
    {
        m_members.putAll(annotation.m_members);
    }

    public boolean addMember(AnnotationMember member)
    {
        if(!m_members.containsKey(member.getName()))
        {
            m_members.put(member.getName(), member);
            return true;
        }

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
    public boolean isIsModule()
    {
        return false;
    }

    @Override
    public boolean isIsException()
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

	@Override
    public boolean isIsAnnotation()
    {
        return true;
    }

    public void addEnums(Vector<TypeCode> enums)
    {
        for (TypeCode tc : enums)
        {
            m_enums.add((EnumTypeCode)tc);
        }
    }

    public void addConstDecl(ConstDeclaration consts)
    {
        m_consts.add(consts);
    }

    public void addTypeDefs(Vector<TypeCode> typedefs)
    {
        for (TypeCode tc : typedefs)
        {
            m_typedefs.add((AliasTypeCode)tc);
        }
    }

    public List<EnumTypeCode> getEnums()
    {
        return new ArrayList<EnumTypeCode>(m_enums);
    }

    public EnumTypeCode getEnum(String name)
    {
        for (EnumTypeCode etc : m_enums)
        {
            if (etc.getName().equals(name))
            {
                return etc;
            }
        }
        return null;
    }

    public List<ConstDeclaration> getConstDecls()
    {
        return new ArrayList<ConstDeclaration>(m_consts);
    }

    public ConstDeclaration getConstDecl(String name)
    {
        for (ConstDeclaration cdcl : m_consts)
        {
            if (cdcl.getName().equals(name))
            {
                return cdcl;
            }
        }
        return null;
    }

    public List<AliasTypeCode> getTypeDefs()
    {
        return new ArrayList<AliasTypeCode>(m_typedefs);
    }

    public AliasTypeCode getTypeDef(String name)
    {
        for (AliasTypeCode atc : m_typedefs)
        {
            if (atc.getName().equals(name))
            {
                return atc;
            }
        }
        return null;
    }

    public TypeCode getTypeCode(String name)
    {
        TypeCode result = getTypeDef(name);
        if (result == null)
        {
            result = getEnum(name);
        }
        return result;
    }

    private Object m_parent = null;
    private HashMap<String, AnnotationMember> m_members = null;
    private Vector<EnumTypeCode> m_enums = null;
    private Vector<ConstDeclaration> m_consts = null;
    private Vector<AliasTypeCode> m_typedefs = null;
}
