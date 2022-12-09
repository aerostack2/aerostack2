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

import java.util.ArrayList;
import org.antlr.v4.runtime.Token;

public class DefinitionContainer extends TreeNode
{
    protected DefinitionContainer(String scopeFile, boolean isInScope, String scope, String name, Token token)
    {
        super(scopeFile, isInScope, scope, name, token);

        m_definitions = new ArrayList<Definition>();
    }
    
    public void add(Definition def)
    {
        m_definitions.add(def);
        def.setParent(this);
    }
    
    public ArrayList<Definition> getDefinitions()
    {
        return m_definitions;
    }
    
    private ArrayList<Definition> m_definitions;
}
