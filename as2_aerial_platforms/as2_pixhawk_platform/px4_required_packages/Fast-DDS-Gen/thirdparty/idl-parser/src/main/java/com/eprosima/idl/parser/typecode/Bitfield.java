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



public class Bitfield extends Member
{
    public Bitfield()
    {
        super();
    }

    public Bitfield(BitsetTypeCode typecode, BitfieldSpec spec, String name)
    {
        super(typecode, name);
        m_spec = spec;
    }

    public void setBasePosition(int position)
    {
        m_base = position;
    }

    public int getBasePosition()
    {
        return m_base;
    }

    public BitfieldSpec getSpec()
    {
        return m_spec;
    }

    @Override
    public String getName()
    {
        if (super.getName().isEmpty())
        {
            return null;
        }
        return super.getName();
    }

    private int m_base = -1;
    private BitfieldSpec m_spec = null;
}
