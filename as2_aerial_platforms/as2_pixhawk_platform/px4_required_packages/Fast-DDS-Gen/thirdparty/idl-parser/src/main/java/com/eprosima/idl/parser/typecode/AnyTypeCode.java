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


/**
 * Temporal type that will be resolved at compilation time.
 */
public class AnyTypeCode extends TypeCode
{
    public AnyTypeCode()
    {
        super(Kind.KIND_NULL);
    }

    @Override
    public String getCppTypename()
    {
        return null;
    }

    @Override
    public String getCTypename()
    {
        return null;
    }

    @Override
    public String getJavaTypename()
    {
        return null;
    }

    @Override
    public String getIdlTypename()
    {
        return null;
    }

    @Override
    public boolean isPrimitive()
    {
        return false;
    }

    @Override
    public String getTypeIdentifier()
    {
        return null;
    }

    @Override
    public boolean isPrimitiveType() { return false; }

    @Override
    public String getInitialValue()
    {
        return "";
    }

    @Override
    public String getSize()
    {
        return null;
    }
}
