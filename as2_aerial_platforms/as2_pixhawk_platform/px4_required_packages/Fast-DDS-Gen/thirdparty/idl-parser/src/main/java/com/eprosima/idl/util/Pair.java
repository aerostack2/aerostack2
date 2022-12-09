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

package com.eprosima.idl.util;

public class Pair<F, S>
{
    public Pair(F first, S second)
    {
        m_first = first;
        m_second = second;
    }
    
    public F first()
    {
        return m_first;
    }
    
    public S second()
    {
        return m_second;
    }
    
    private F m_first = null;
    
    private S m_second = null;
}
