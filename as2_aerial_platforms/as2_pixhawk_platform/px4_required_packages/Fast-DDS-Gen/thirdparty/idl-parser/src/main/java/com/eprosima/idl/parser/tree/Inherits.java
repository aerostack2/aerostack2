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

import java.util.ArrayList;

public interface Inherits
{
    /*!
     * @brief This function links all super types to the object.
     * @param ctx Context used in the IDL parser. Can be useful for developers.
     * @param parent Super TypeCode to be linked.
     */
    public void addInheritance(Context ctx, TypeCode parent);

    /*!
     * @brief This function returns all super types linked with the object.
     * @return Map with the linked super types.
     */
    public ArrayList<TypeCode> getInheritances();
}
