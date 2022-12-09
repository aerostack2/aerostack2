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

import org.antlr.v4.runtime.Token;

public class ConstDeclaration extends TreeNode implements Definition, Export
{
    public ConstDeclaration(String scopeFile, boolean isInScope, String scope, String name, TypeCode typecode, String value, Token token)
    {
        super(scopeFile, isInScope, scope, name, token);

        m_typecode = typecode;
        m_value = value;
        // Set as parent to the Typecode.
        m_typecode.setParent(this);
    }

    public TypeCode getTypeCode()
    {
        return m_typecode;
    }

    public String getValue()
    {
        return m_value;
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
    public boolean isIsOperation()
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
        return true;
    }

    @Override
    public boolean resolve(Context ctx)
    {
        return true;
    }

	@Override
    public boolean isIsAnnotation()
    {
        return false;
    }

    private TypeCode m_typecode = null;
    private String m_value = null;
    private Object m_parent = null;
}
