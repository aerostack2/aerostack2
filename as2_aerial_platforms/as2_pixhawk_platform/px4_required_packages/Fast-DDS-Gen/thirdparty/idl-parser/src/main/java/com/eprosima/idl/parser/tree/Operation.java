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
import com.eprosima.idl.parser.tree.Param;

import java.util.ArrayList;
import org.antlr.v4.runtime.Token;

public class Operation extends TreeNode implements Export
{
    public Operation(String scopeFile, boolean isInScope, String scope, String name, Token tk)
    {
        super(scopeFile, isInScope, scope, name, tk);

        m_params = new ArrayList<Param>();
        m_exceptions = new ArrayList<com.eprosima.idl.parser.tree.Exception>();
        m_unresolvedExceptions = new ArrayList<String>();
    }

    public void setParent(Object obj)
    {
        m_parent = obj;
    }

    public Object getParent()
    {
        return m_parent;
    }

    public boolean isIsOperation()
    {
        return true;
    }

    @Override
    public boolean isIsException()
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

    public void setOneway(boolean b)
    {
        m_isOneway = b;
    }

    public boolean isOneway()
    {
        return m_isOneway;
    }

    public void add(Param param)
    {
        m_params.add(param);
        param.setParent(this);
    }

    public ArrayList<Param> getParameters()
    {
        return m_params;
    }

    public ArrayList<Param> getInputparam()
    {
        ArrayList<Param> input = new ArrayList<Param>();

        for(int count = 0; count < m_params.size(); ++count)
            if(m_params.get(count).isInput())
                input.add(m_params.get(count));

        return input;
    }

    public ArrayList<Param> getOutputparam()
    {
        ArrayList<Param> output = new ArrayList<Param>();

        for(int count = 0; count < m_params.size(); ++count)
            if(m_params.get(count).isOutput())
                output.add(m_params.get(count));

        return output;
    }

    public ArrayList<Param> getInoutputparam()
    {
        ArrayList<Param> inoutput = new ArrayList<Param>();

        for(int count = 0; count < m_params.size(); ++count)
            if(m_params.get(count).isInput() && m_params.get(count).isOutput())
                inoutput.add(m_params.get(count));

        return inoutput;
    }

    // TODO Delete when standard change.
    public void setRettype(TypeCode rettype)
    {
        if(rettype != null)
        {
            m_rettype = rettype;
            m_rettypeparam = new Param("return_", m_rettype, Param.Kind.OUT_PARAM);
        }
    }

    public TypeCode getRettype()
    {
        return m_rettype;
    }

    public Param getRettypeparam()
    {
        return m_rettypeparam;
    }

    public void addException(com.eprosima.idl.parser.tree.Exception exception)
    {
        m_exceptions.add(exception);
    }

    public ArrayList<com.eprosima.idl.parser.tree.Exception> getExceptions()
    {
        return m_exceptions;
    }

    public void addUnresolvedException(String ename)
    {
        m_unresolvedExceptions.add(ename);
    }

    @Override
    public boolean resolve(Context ctx)
    {
    	//Resolve unresolved exceptions. This unresolved exceptions should be exceptions of the parent interface.

        if(m_parent != null)
    	{
    	    if(m_parent instanceof Interface)
    		{
    		    Interface ifc = (Interface)m_parent;

    		    for(int count = 0; count < m_unresolvedExceptions.size(); ++count)
    			{
    			    com.eprosima.idl.parser.tree.Exception ex = ifc.getException(ctx.getScope(), m_unresolvedExceptions.get(count));

    			    if(ex != null)
    				{

    				    m_exceptions.add(ex);
    				}
    			    else
    				{
    				    System.out.println("ERROR: The definition of exception " + m_unresolvedExceptions.get(count) +
    							" was not found");
    					//TODO
    					//return false;
    				}
    			}

    		    return true;
    		}
    	    else
    		{
    		    System.out.println("ERROR<Operation::resolve>: Parent is not an interface");
    		}
    	}
        else
    	{
    	    System.out.println("ERROR<Operation::resolve>: Not parent for operation");
    	}

        return false;
    }

    private Object m_parent = null;
    private boolean m_isOneway = false;
    private ArrayList<Param> m_params;
    private ArrayList<com.eprosima.idl.parser.tree.Exception> m_exceptions;
    private ArrayList<String> m_unresolvedExceptions;
    private TypeCode m_rettype = null;
    private Param m_rettypeparam = null;
}
